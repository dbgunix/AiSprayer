"""
纳博特(iNexbot) 机械臂官方 SDK (SWIG) 通信驱动封装

与官方 demo 对齐：third_party/demo_unzipped/main.py、nrc_c_interface.h

状态量（两套，勿混用）:
  get_servo_state — 伺服使能链（上电/示教器能否拖动手臂）:
    0 停止  1 就绪(待命)  2 报警  3 运行(已上电使能，非“正在走点”)
  get_robot_running_state — 程序/运动段是否在执行（demo 用其判断运动结束）:
    0 停止  1 暂停  2 运行中

运动完成判定：仅看 get_robot_running_state==0（见 _wait_motion_done / demo queue_send）。
上电完成判定：get_servo_state==3。

is_reachable：调用 get_pos_reachable；旧版 SDK 曾触发 25566，当前版本已升级可正常使用。
"""

import time
import math
import logging
import threading
from typing import Optional, List

from .inexbot_v24_03_py38 import nrc_interface as nrc
from .base_driver import BaseRobotDriver, RobotPose, PoseLike, _to_list

COORD_ACS = 0; COORD_MCS = 1
MODE_TEACH = 0; MODE_REMOTE = 1; MODE_RUN = 2



logger = logging.getLogger(__name__)


# ---------------------------------------------
# 伺服状态常量（get_servo_state，对应 demo Widget 伺服停止/就绪/报警/运行）
# ---------------------------------------------
SERVO_STATE_STOP       = 0  # 停止
SERVO_STATE_READY      = 1  # 就绪（待命，未上电使能）
SERVO_STATE_ALARM      = 2  # 报警
SERVO_STATE_RUNNING    = 3  # 运行（已上电使能，示教器可拖动；不等于手臂正在运动）


# ---------------------------------------------
# 机器人运行状态常量（get_robot_running_state，demo: jug=status[1]，jug!=0 表示仍在执行）
# ---------------------------------------------
RUNNING_STATE_STOP       = 0  # 停止（无运动程序段在执行）
RUNNING_STATE_PAUSE      = 1  # 暂停
RUNNING_STATE_RUNNING    = 2  # 运行中（正在执行 Move/队列等）


# ---------------------------------------------
# 返回值常量
# ---------------------------------------------
RESULT_TIMEOUT               = -6 # 超时
RESULT_EXCEPTION             = -5 # 异常
RESULT_OPERATION_NOT_ALLOWED = -4 # 操作不允许
RESULT_PARAM_ERR             = -3 # 参数错误
RESULT_DISCONNECT            = -2 # 断开连接
RESULT_RECEIVE_FAILED        = -1 # 接收失败
RESULT_SUCCESS               = 0 # 成功

DEFAULT_VELOCITY = 50       # MOVL 默认线速度 mm/s
DEFAULT_ACC = 80
DEFAULT_DEC = 80
DEFAULT_GLOBAL_SPEED = 80   # 示教/远程/运行模式全局速度百分比 (0,100]

class InexbotDriver(BaseRobotDriver):
    """
    Inexbot 机械臂驱动类。

    角度约定：姿态角 (A,B,C) 为弧度；位置 (X,Y,Z) 为毫米。

    **伺服状态** `get_servo_state`：上电/使能/报警，**不**用于判断单次 Move 是否走完。
    **运行状态** `get_robot_running_state`：与官方 demo 一致，**用于** wait 运动/队列结束
    （demo 中 while jug != 0: jug = get_robot_running_state(...)[1]，即等到 0）。

    使用示例：
        import math
        robot = InexbotDriver("192.168.2.14")
        robot.startup()
        robot.move_l(RobotPose(400, 0, 300, 0, math.pi, 0))
        robot.shutdown()
    """

    COORD      = 1    # 直角坐标系(0=关节坐标系 1=直角坐标系 2=工具坐标系 3=用户坐标系)
    MODE       = 0    # 默认示教模式(0=示教 1=远程 2=运行)，队列联调推荐保持 0
    ANGLE_UNIT = 1    # get_pos_reachable pose_vec[1]：0=度制，1=弧度制（与 RobotPose rad 一致）

    def __init__(
        self,
        ip: str = "192.168.2.14",
        port: str = "6001",
        toolnum: int = 0,
        reconnect: bool = True,
        mode: Optional[int] = None,
        global_speed: int = DEFAULT_GLOBAL_SPEED,
    ):
        self.ip           = ip
        self.port         = str(port)
        self.tool_num     = toolnum
        self.reconnect    = reconnect
        self.mode         = self.MODE if mode is None else int(mode)
        self.global_speed = int(global_speed)
        self.fd           = -1
        self._queue_size  = 0     # 本地队列已追加的指令条数
        self._queue_mode_active = False  # 队列模式是否已打开（避免重复 stop/release）
        self._sdk_lock    = threading.RLock()

    @staticmethod
    def _ok_or_already(result: int) -> bool:
        """SDK 返回 0 成功；-4 表示目标状态已满足（如无报警可清、伺服已在该状态）。"""
        return result == RESULT_SUCCESS or result == RESULT_OPERATION_NOT_ALLOWED

    @staticmethod
    def _decode_vector_char(vec) -> str:
        """将 SWIG VectorChar 转为字符串（兼容 int 与 str 元素）。"""
        parts = []
        for c in vec:
            if isinstance(c, int):
                if c == 0:
                    break
                parts.append(chr(c))
            elif not c or c == "\x00":
                break
            else:
                parts.append(str(c))
        return "".join(parts).rstrip("\x00")

    # ---------------------------------------
    # 连接 / 断开
    # ---------------------------------------
    def startup(self, timeout: float = 10.0) -> bool:
        """
        启动机器人连接并使能伺服（一键上电流程）：连接控制器 → 设置坐标系/模式 → 清错 → 
        伺服停止(0) → 就绪(1) → 上电(power on)。
        上电完成后自动打印系统信息快照.
        :param timeout: 每个等待步骤的最长超时时间（秒）。
        :return: True/False, 是否成功启动并上电, 或任意步骤失败。
        """
        logger.info(f"[startup] nrc lib version: {nrc.get_library_version()}")

        # 步骤0: 如果已经连接，直接返回
        if self.fd >= 0:
            logger.info("[startup] already started")
            return True

        # 步骤1: 连接控制器（断开后立即重连常被拒绝，短暂重试）
        self.fd = -1
        for attempt in range(5):
            self.fd = nrc.connect_robot(self.ip, self.port)
            if self.fd >= 0:
                break
            if attempt < 4:
                logger.warning(
                    f"[startup] connect attempt {attempt + 1} failed, retry in 2s..."
                )
                time.sleep(2.0)
        if self.fd < 0:
            logger.error(f"[startup] Failed to connect to robot: {self.ip}:{self.port}")
            return False
        logger.info(f"[startup] Connection established to {self.ip}:{self.port}")

        with self._sdk_lock:
            nrc.set_reconnect(self.fd, self.reconnect)
            # 注意：此时不启动心跳线程。主线程与后台线程同时调用 nrc 可能导致
            # RECEIVE_FAILED(-1) / Connect break，尤其在 set_servo_poweron 期间。

            # 步骤2：设置坐标系和运行模式
            result = nrc.set_current_coord(self.fd, self.COORD)
            if result != nrc.SUCCESS:
                logger.error(f"[startup] Failed to set coordinate system to {self.COORD}, result: {result}")
                return False
            logger.info(f"[startup] Set coordinate system to {self.COORD}")
            # 上电流程需在示教模式完成；远程/运行模式在伺服 RUNNING 后再切换
            boot_mode = 0
            result = nrc.set_current_mode(self.fd, boot_mode)
            if result != nrc.SUCCESS:
                logger.error(f"[startup] Failed to set boot mode {boot_mode}, result: {result}")
                return False
            logger.info(f"[startup] Set boot mode to {boot_mode} (teach, for power-on)")
            if self.tool_num > 0:
                result = nrc.set_tool_hand_number(self.fd, self.tool_num)
                if result != nrc.SUCCESS:
                    logger.error(f"[startup] Failed to set tool number to {self.tool_num}, result: {result}")
                    return False
                logger.info(f"[startup] Set tool number to {self.tool_num}")

            # 步骤3：清除错误（无报警时 SDK 常返回 -4，与 nabot_robot 一致视为可继续）
            result = nrc.clear_error(self.fd)
            if not self._ok_or_already(result):
                logger.error(f"[startup] Failed to clear error, result: {result}")
                return False
            if result == RESULT_OPERATION_NOT_ALLOWED:
                logger.info("[startup] No error to clear (result=-4), continue")
            else:
                logger.info("[startup] Cleared")
            time.sleep(0.3)

            # 步骤4：设置状态 0 → 1
            for state in [0, 1]:
                result = nrc.set_servo_state(self.fd, state)
                if not self._ok_or_already(result):
                    logger.error(f"[startup] Failed to set state to {state}, result: {result}")
                    return False
                logger.info(f"[startup] Set state to {state}")
                time.sleep(0.3 if state == 0 else 0.5)

            # 步骤5：上电并轮询至伺服 RUNNING(3)（与 nabot_robot 一致）
            deadline = time.time() + max(timeout, 20.0)
            servo_running = False
            last_servo = -1
            for attempt in range(3):
                if attempt > 0:
                    logger.warning("[startup] retry power-on cycle %s", attempt + 1)
                    nrc.clear_error(self.fd)
                    nrc.set_servo_poweroff(self.fd)
                    time.sleep(0.8)
                    for state in [0, 1]:
                        nrc.set_servo_state(self.fd, state)
                        time.sleep(0.3 if state == 0 else 0.6)

                result = nrc.set_servo_poweron(self.fd)
                if result != nrc.SUCCESS:
                    logger.warning(
                        "[startup] set_servo_poweron attempt %s returned %s",
                        attempt + 1, result,
                    )
                t0 = time.time()
                while time.time() < deadline and time.time() - t0 < 12.0:
                    st = nrc.get_servo_state(self.fd, 0)
                    if isinstance(st, (list, tuple)) and len(st) > 1 and int(st[0]) == nrc.SUCCESS:
                        last_servo = int(st[1])
                        if last_servo == SERVO_STATE_RUNNING:
                            servo_running = True
                            break
                    time.sleep(0.25)
                if servo_running:
                    break

            if not servo_running:
                logger.error(
                    "[startup] Timeout waiting for servo RUNNING(3), last_servo=%s "
                    "(0=停 1=就绪 2=报警；请在示教器上清错并上电)",
                    last_servo,
                )
                return False
            logger.info("[startup] Servo RUNNING(3)")

            if self.mode != boot_mode:
                result = nrc.set_current_mode(self.fd, self.mode)
                if result != nrc.SUCCESS:
                    logger.error(
                        "[startup] Failed to switch to mode %s after power-on, result: %s",
                        self.mode, result,
                    )
                    return False
                logger.info(f"[startup] Switched running mode to {self.mode}")
                time.sleep(0.5)
                # 切远程/运行后伺服常会回到就绪(1)，需再次上电到运行(3)
                st = nrc.get_servo_state(self.fd, 0)
                if isinstance(st, (list, tuple)) and len(st) > 1 and int(st[1]) != SERVO_STATE_RUNNING:
                    logger.info("[startup] Re-enable servo after mode switch (last=%s)", st[1])
                    nrc.set_servo_poweron(self.fd)
                    t0 = time.time()
                    while time.time() - t0 < 10.0:
                        st = nrc.get_servo_state(self.fd, 0)
                        if isinstance(st, (list, tuple)) and len(st) > 1 and int(st[1]) == SERVO_STATE_RUNNING:
                            logger.info("[startup] Servo RUNNING(3) after mode switch")
                            break
                        time.sleep(0.5)

        time.sleep(1)

        with self._sdk_lock:
            sp = nrc.set_speed(self.fd, self.global_speed)
        if sp == nrc.SUCCESS:
            logger.info("[startup] Set global speed to %s%%", self.global_speed)
        else:
            logger.warning("[startup] set_speed(%s) returned %s", self.global_speed, sp)

        logger.info(f"[startup] Robot startup successful and servo enabled")
        try:
            self.print_system_info()
        except Exception as e:
            logger.warning(f"[startup] print_system_info failed: {e}")
        return True

    def set_global_speed(self, speed: int) -> bool:
        """
        设置全局速度百分比，范围 (0, 100]
        """
        if self.fd < 0:
            logger.warning("[set_global_speed] Robot is not connected")
            return False
        
        speed = max(1, min(100, int(speed)))
        with self._sdk_lock:
            ret = nrc.set_speed(self.fd, speed)
        if ret == nrc.SUCCESS:
            self.global_speed = speed
            logger.info("Set global speed to %d%%", speed)
            return True
        else:
            logger.error("Failed to set global speed to %d%%, code: %d", speed, ret)
            return False

    def set_tool_number(self, tool_num: int) -> bool:
        """
        设置当前激活的工具编号。
        """
        if self.fd < 0:
            logger.warning("[set_tool_number] Robot is not connected")
            return False
        
        tool_num = max(0, int(tool_num))
        with self._sdk_lock:
            ret = nrc.set_tool_hand_number(self.fd, tool_num)
        if ret == nrc.SUCCESS:
            self.tool_num = tool_num
            logger.info("Set tool number to %d", tool_num)
            return True
        else:
            logger.error("Failed to set tool number to %d, code: %d", tool_num, ret)
            return False

    def shutdown(self) -> None:
        """
        关闭机器人连接，断开控制器。
        """
        if self.fd >= 0:
            try:
                self._queue_release()
            except Exception:
                pass
            with self._sdk_lock:
                # 断开前切回示教模式，避免上位机占用后示教器长期锁死
                r = nrc.set_current_mode(self.fd, 0)
                if r == nrc.SUCCESS:
                    logger.info("[shutdown] Restored running mode to 0 (teach)")
                nrc.set_servo_poweroff(self.fd)
            time.sleep(0.5)
            nrc.disconnect_robot(self.fd)
            self.fd = -1
        logger.info(f"[shutdown] Disconnected from {self.ip}:{self.port}")

    # -------------------------------------------------
    #  兼容性接口 (与 inexbot_driver.py 保持一致)
    # -------------------------------------------------
    def connect(self) -> bool:
        """兼容性别名，调用 startup() 一键启动并使能。"""
        logger.info("[connect] 兼容性接口调用 -> 执行 startup()")
        return self.startup()

    def disconnect(self) -> None:
        """兼容性别名，调用 shutdown() 断开连接。"""
        logger.info("[disconnect] 兼容性接口调用 -> 执行 shutdown()")
        self.shutdown()

    def servo_power_on(self) -> bool:
        """兼容性别名，对伺服上电。"""
        logger.info("[servo_power_on] 兼容性接口调用")
        with self._sdk_lock:
            ret = nrc.set_servo_poweron(self.fd)
        return ret == nrc.SUCCESS or ret == RESULT_OPERATION_NOT_ALLOWED

    def set_mode(self, mode: int) -> None:
        """兼容性别名，设置运行模式。"""
        logger.info("[set_mode] 兼容性接口调用 -> %s", mode)
        with self._sdk_lock:
            nrc.set_current_mode(self.fd, mode)

    def set_coord(self, coord: int) -> None:
        """兼容性别名，设置坐标系。"""
        logger.info("[set_coord] 兼容性接口调用 -> %s", coord)
        with self._sdk_lock:
            nrc.set_current_coord(self.fd, coord)

    def clear_error(self) -> None:
        """兼容性别名，清除控制器报警/错误。"""
        logger.info("[clear_error] 兼容性接口调用")
        with self._sdk_lock:
            nrc.clear_error(self.fd)

    def wait_motion_done(self, timeout: float = 60.0) -> bool:
        """兼容性别名，等待运动结束。"""
        return self._wait_motion_done(timeout=timeout)


    def print_system_info(self) -> None:
        """
        打印机器人系统信息快照。
        """
        fd = self.fd
        if fd < 0:
            logger.warning("[print_system_info] Robot is not connected")
            return

        with self._sdk_lock:
            self._print_system_info_locked(fd)

    def _print_system_info_locked(self, fd: int) -> None:
        """在已持有 _sdk_lock 时打印系统信息（供 print_system_info / 内部调用）。"""
        # 1-六轴串联多关节  2-四轴scara  3-四轴码垛  4-四轴串联多关节  5-单轴  
        # 6-五轴串联多关节  7-六轴协作  8-二轴scara  9-三轴scara  10-三轴直角
        # 11-三轴异性  12-七轴串联多关节  13-scara异性一  14-四轴码垛丝杆
        _ROBOT_TYPE = {
            1: "六轴串联多关节",  2: "四轴SCARA", 3: "四轴码垛", 
            4: "四轴串联多关节",  5: "单轴", 6: "五轴串联多关节", 
            7: "六轴协作",  8: "二轴scara", 9: "三轴scara",
            10: "三轴直角", 11: "三轴异性", 12: "七轴串联多关节",
            13: "scara异性一", 14: "四轴码垛丝杆"
        }

        _COORD_MAP = {0:"关节", 1:"直角", 2:"工具", 3:"用户"}
        _MODE_MAP  = {0:"示教", 1:"远程", 2:"运行"}
        _SERVO_MAP = {0:"停止", 1:"就绪", 2:"报警", 3:"运行"}

        def _get(call, *args):
            """
            统一处理 SWIG 出参列表形式的返回值。
            调用格式：call(fd, *args)，返回 result[0]=0表示成功, <0 表示出错, result[1]为实际参数
            查询失败（非 SUCCESS）时返回 None
            """
            result = call(fd, *args)
            if isinstance(result, (list, tuple)):
                # 如果成功，返回实际值 result[1]；如果失败，返回错误码 result[0]
                return result[1] if int(result[0]) == nrc.SUCCESS else int(result[0])
            # 如果返回的是整数，无论成功（0）还是失败（负数），直接返回该结果
            return result
        
        # 1. 获取控制器 ID (使用 VectorChar 版本, 比 char* 版本更稳定)
        ctrl_id_vec = nrc.VectorChar()
        id_result = nrc.get_controller_id_csharp(fd, ctrl_id_vec)
        if isinstance(id_result, (list, tuple)) and int(id_result[0]) == nrc.SUCCESS:
            ctrl_id = self._decode_vector_char(ctrl_id_vec) or "--"
        else:
            ctrl_id = "--"
        
        # 2. 示教器连接状态
        tb_connected = None
        if hasattr(nrc, "get_teachbox_connection_status"):
            tb_status = False
            tb_result = nrc.get_teachbox_connection_status(fd, tb_status)
            if isinstance(tb_result, (list, tuple)) and int(tb_result[0]) == nrc.SUCCESS:
                tb_connected = bool(tb_result[1])

        # 3. 机器人类型
        rtype = _get(nrc.get_robot_type, 0)
        rtype_str = _ROBOT_TYPE.get(rtype, f"未知({rtype})") if rtype is not None else "--"

        # 4. 坐标系
        coord = _get(nrc.get_current_coord, 0)
        coord_str = _COORD_MAP.get(coord, f"未知({coord})") if coord is not None else "--"

        # 5. 运行模式
        mode = _get(nrc.get_current_mode, 0)
        mode_str = _MODE_MAP.get(mode, f"未知({mode})") if mode is not None else "--"

        # 6. 当前全局速度
        speed = _get(nrc.get_speed, 0)
        speed_str = f"{speed}%" if speed is not None else "--"

        # 7. 伺服状态
        servo_status = _get(nrc.get_servo_state, 0)
        servo_status_str = _SERVO_MAP.get(servo_status, f"未知({servo_status})") if servo_status is not None else "--"

        # 8. 当前激活的工具编号
        toolnum = _get(nrc.get_tool_hand_number, 0)
        toolnum_str = str(toolnum) if toolnum is not None else "--"

        # 9. 当前关节坐标(上电后有效， 单位: °)
        raw_joint = nrc.VectorDouble()
        result = nrc.get_current_position(self.fd, 0, raw_joint)
        joint_pose_str = str(RobotPose.from_list(list(raw_joint))) if result == nrc.SUCCESS and raw_joint else "--"

        # 10. 当前末端位姿(直角坐标，单位: mm,°)
        raw_cart = nrc.VectorDouble()
        result = nrc.get_current_position(self.fd, 1, raw_cart)
        cart_pose_str = str(RobotPose.from_list(list(raw_cart))) if result == nrc.SUCCESS and raw_cart else "--"

        print(
            f"\n{'-' * 50}\n"
            f"控制器ID: {ctrl_id}\n"
            f"示教器连接状态: {tb_connected}\n"
            f"机器人类型: {rtype_str}\n"
            f"坐标系: {coord_str}\n"
            f"运行模式: {mode_str}\n"
            f"当前全局速度: {speed_str}\n"
            f"伺服状态: {servo_status_str}\n"
            f"当前激活的工具编号: {toolnum_str}\n"
            f"当前关节坐标(上电后有效， 单位: °): {joint_pose_str}\n"
            f"当前末端位姿(直角坐标，单位: mm,°): {cart_pose_str}\n"
            f"\n{'-' * 50}\n"
        )

    # -------------------------------------------------
    # 状态查询
    # -------------------------------------------------

    def get_servo_state(self) -> int:
        """
        获取伺服状态（使能链，见 nrc_c_interface.h / demo servo_status）。
        :return: -1=错误；0=停止；1=就绪(待命)；2=报警；3=运行(已上电使能，非正在运动)
        """
        if self.fd < 0:
            logger.warning("[get_servo_state] Robot is not connected")
            return -1

        with self._sdk_lock:
            result = nrc.get_servo_state(self.fd, 0)
        if result is None:
            logger.error("[get_servo_state] Failed to get servo state")
            return -1
        elif result[0] != nrc.SUCCESS:
            logger.error(f"[get_servo_state] Failed to get servo state, error code: {result[0]}")
            return -1
        else:
            return result[1]

    def get_running_state(self) -> int:
        """
        获取机器人运行状态（是否在执行运动程序段，见 get_robot_running_state）。
        与 third_party/demo_unzipped/main.py 中 queue_send / test 的等待循环一致：
        status[1]!=0 表示仍在执行，==0 表示本段运动结束。
        :return: -1=错误；0=停止；1=暂停；2=运行中
        """
        if self.fd < 0:
            logger.warning("[get_running_state] Robot is not connected")
            return -1

        with self._sdk_lock:
            result = nrc.get_robot_running_state(self.fd, 1)
        if isinstance(result, (list, tuple)) and len(result) > 1:
            if int(result[0]) == nrc.SUCCESS:
                return int(result[1])
            logger.error(f"[get_running_state] SDK error code: {result[0]}")
        return -1

    def is_servo_enabled(self) -> bool:
        """伺服是否已上电使能（get_servo_state==3），示教器可拖动臂。"""
        return self.get_servo_state() == SERVO_STATE_RUNNING

    def is_robot_idle(self) -> bool:
        """运动程序是否空闲（get_robot_running_state==0），与官方 demo 运动结束条件一致。"""
        return self.get_running_state() == RUNNING_STATE_STOP

    def recover_servo(self, timeout: float = 8.0) -> bool:
        """清错并重新上电。就绪(1)/运行(3) 直接返回；仅报警/停止时清错上电。"""
        if self.fd < 0:
            return False
        st = self.get_servo_state()
        if st in (SERVO_STATE_RUNNING, SERVO_STATE_READY):
            return True
        if st not in (SERVO_STATE_ALARM, SERVO_STATE_STOP):
            logger.warning("[recover_servo] unexpected servo state %s", st)
            return False
        with self._sdk_lock:
            nrc.clear_error(self.fd)
            nrc.set_servo_poweron(self.fd)
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.get_servo_state() == SERVO_STATE_RUNNING:
                logger.info("[recover_servo] servo RUNNING(3)")
                return True
            time.sleep(0.3)
        logger.error("[recover_servo] timeout, last=%s", self.get_servo_state())
        return False

    def get_current_pose(self) -> RobotPose:
        """
        获取当前位姿
        :param coord: 坐标系 0=关节；1=直角；2=工具；3=用户
        :return: RobotPose (平移单位 mm, 旋转单位 rad)
        """
        if self.fd < 0:
            logger.warning("[get_current_pose] Robot is not connected")
            return None

        with self._sdk_lock:
            raw_cart = nrc.VectorDouble()
            result = nrc.get_current_position(self.fd, self.COORD, raw_cart)
        if result != nrc.SUCCESS:
            logger.error(f"[get_current_pose] Failed to get current position, code: {result}")
            return None
        return RobotPose.from_list(list(raw_cart))
    
    # -------------------------------------------------
    #  可达性判断
    # -------------------------------------------------

    def is_reachable(self, pose: PoseLike, movetype: str = "MOVL") -> bool:
        """
        判断目标位姿是否可达(仅做运动学逆解检验， 不含碰撞检测)
        :param pose:  目标位姿，RobotPose 或 [x(mm), y(mm), z(mm), A(rad), B(rad), C(rad)]
        :param movetype: 运动类型, "MOVL"=直线运动, "MOVJ"=关节运动
        :return: True=可达，False=不可达或查询接口失败
        """
        # nrc.get_pos_reachable 需要长度为 14 的特殊向量，布局如下：
        # [0]=坐标系 [1]=角度制(0=角度/1=弧度) [2]=形态 [3]=工具号
        # [4]=用户坐标编号 [5][6]=备用 [7~13]=目标位姿 x,y,z,a,b,c,备用
        if self.fd < 0:
            logger.warning("[is_reachable] Robot is not connected")
            return False

        pose_vec = [0.0] * 14
        pose_vec[0] = float(self.COORD)        # 直角坐标系
        pose_vec[1] = float(self.ANGLE_UNIT)   # 弧度制
        pose_vec[3] = float(self.tool_num)
        for i, v in enumerate(_to_list(pose)[:6]):
            pose_vec[7+i] = float(v)

        result_bool = False
        with self._sdk_lock:
            res = nrc.get_pos_reachable(self.fd, pose_vec, movetype, result_bool)
        if isinstance(res, (list, tuple)) and len(res) > 1:
            if int(res[0]) == nrc.SUCCESS:
                return bool(res[1])
            logger.error(f"[is_reachable] SDK error code: {res[0]}")
        return False

    # -------------------------------------------------
    #  运动控制（内部工具方法）
    # -------------------------------------------------
    
    def _make_movecmd(
        self, 
        pose: PoseLike, 
        velocity: float,
        acc: float,
        dec: float,
        tool_num: int = 0,
        user_num: int = 0,
        pl: int = 0,
    ) -> nrc.MoveCmd:
        """
        根据目标位姿和运动参数构造 MoveCmd 对象。
        统一使用直角坐标系，位姿数组长度补齐到 7 位（本体 6 位 + 外部轴 1 位，外部轴默认始终为 0）
        :param pose: 目标位姿，RobotPose 或 [x(mm), y(mm), z(mm), A(rad), B(rad), C(rad)]
        :param velocity: 运动速度
        :param acc: 加速度
        :param dec: 减速度
        :param tool_num: 工具号
        :param user_num: 用户坐标号
        :param pl: 平滑过渡参数，范围[0, 5]；0=精确到位（有停顿），1〜5=数值越大越平滑（不停顿）
        :return: MoveCmd
        """
        # 1. 构造 14 位点位数组 (前 7 位本体，后 7 位外部轴)
        vec = nrc.VectorDouble()
        for v in _to_list(pose):
            vec.append(float(v))
        # MoveCmd.targetPosValue 要求长度至少为 7 （第 7 位是外部轴， 此处置 0）
        while len(vec) < 7:
            vec.append(0.0)

        cmd = nrc.MoveCmd()
        cmd.targetPosType = nrc.PosType_data
        cmd.targetPosValue = vec
        cmd.coord = self.COORD
        cmd.velocity = velocity
        cmd.acc = acc
        cmd.dec = dec
        cmd.toolNum = tool_num if tool_num > 0 else self.tool_num
        cmd.userNum = user_num
        cmd.pl = pl
        return cmd

    def _wait_motion_done(self, poll_interval: float = 0.025, timeout: float = 600.0) -> bool:
        """
        等待单次运动结束：仅轮询 get_robot_running_state，直到为 0（STOP）。
        不使用 get_servo_state（伺服 3 只表示已使能，运动中仍为 3）。
        逻辑同 demo/main.py: while jug != 0: jug = get_robot_running_state(...)[1]
        """
        time.sleep(0.1)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.is_robot_idle():
                return True
            time.sleep(poll_interval)
        logger.error("[wait_motion_done] timeout, running_state=%s", self.get_running_state())
        return False

    # -------------------------------------------------
    #  运动指令（外部调用接口）
    # -------------------------------------------------
    
    def move_j(
        self, 
        pose: PoseLike, 
        velocity: float = DEFAULT_VELOCITY, 
        acc: float = DEFAULT_ACC, 
        dec: float = DEFAULT_DEC,
        tool_num: int = 0,
        wait: bool = True,
    ) -> int:
        """
        关节运动(MOVJ)，目标位姿以直角坐标描述，由控制器内部做逆解。
        :param pose: 目标位姿 RobotPose 或 [x(mm),y(mm),z(mm),A(rad),B(rad),C(rad)]
        :param velocity: 速度，单位 %（百分比），范围 (0, 100]
        :param acc: 加速度，范围 (0, 100]
        :param dec: 减速度，范围 (0, 100]
        :param tool_num: 工具号
        :param wait: 是否等待运动完成，True=阻塞直到运动完成，False=立即返回
        :return: 接口返回码，（0=成功，负值=失败）
        """
        cmd = self._make_movecmd(pose, velocity, acc, dec, tool_num)
        with self._sdk_lock:
            ret = nrc.robot_movej(self.fd, cmd)
        if ret != nrc.SUCCESS:
            logger.error(f"[move_j] Failed to send movej command, error code: {ret}")
            return ret
        if wait:
            self._wait_motion_done()
            logger.info(f"[move_j] movej command sent successfully, target pose: {pose}")
        return ret

    def move_joint(
        self, 
        joints: List[float], 
        velocity: float = DEFAULT_VELOCITY, 
        acc: float = DEFAULT_ACC, 
        dec: float = DEFAULT_DEC,
        tool_num: int = 0,
        wait: bool = True,
    ) -> int:
        """
        关节空间关节运动(MOVJ)，目标位置由各关节角度确定。
        :param joints: 目标关节角度 [j1, j2, j3, j4, j5, j6]
        :param velocity: 速度，单位 %（百分比），范围 (0, 100]
        :param acc: 加速度，范围 (0, 100]
        :param dec: 减速度，范围 (0, 100]
        :param tool_num: 工具号
        :param wait: 是否等待运动完成，True=阻塞直到运动完成，False=立即返回
        :return: 接口返回码，（0=成功，负值=失败）
        """
        # 1. 构造 14 位点位数组 (前 7 位本体，后 7 位外部轴)
        vec = nrc.VectorDouble()
        for v in joints:
            vec.append(float(v))
        while len(vec) < 7:
            vec.append(0.0)

        cmd = nrc.MoveCmd()
        cmd.targetPosType = nrc.PosType_data
        cmd.targetPosValue = vec
        cmd.coord = 0 # 关节坐标系
        cmd.velocity = velocity
        cmd.acc = acc
        cmd.dec = dec
        cmd.toolNum = tool_num if tool_num > 0 else self.tool_num
        cmd.userNum = 0
        cmd.pl = 0

        with self._sdk_lock:
            ret = nrc.robot_movej(self.fd, cmd)
        if ret != nrc.SUCCESS:
            logger.error(f"[move_joint] Failed to send move_joint command, error code: {ret}")
            return ret
        if wait:
            self._wait_motion_done()
            logger.info(f"[move_joint] move_joint command sent successfully, target joints: {joints}")
        return ret

    def move_l(
        self, 
        pose: PoseLike, 
        velocity: float = DEFAULT_VELOCITY, 
        acc: float = DEFAULT_ACC, 
        dec: float = DEFAULT_DEC,
        tool_num: int = 0,
        wait: bool = True,
    ) -> int:
        """
        直线运动(MOVL)，目标位姿以直角坐标描述，末端在笛卡尔空间走直线运动。
        :param pose: 目标位姿 RobotPose 或 [x(mm),y(mm),z(mm),A(rad),B(rad),C(rad)]
        :param velocity: 线速度，单位 mm/s，范围 (0, 1000]
        :param acc: 加速度，范围 (0, 100]
        :param dec: 减速度，范围 (0, 100]
        :param tool_num: 工具号
        :param wait: 是否等待运动完成，True=阻塞直到运动完成，False=立即返回
        :return: 接口返回码，（0=成功，负值=失败）
        """
        cmd = self._make_movecmd(pose, velocity, acc, dec, tool_num)
        with self._sdk_lock:
            ret = nrc.robot_movel(self.fd, cmd)
        if ret != nrc.SUCCESS:
            logger.error(f"[move_l] Failed to send movel command, error code: {ret}")
            return ret
        if wait:
            self._wait_motion_done()
            logger.info(f"[move_l] movel command sent successfully, target pose: {pose}")
        return ret
    
    def go_home(self, wait: bool = True, velocity: Optional[float] = None, acc: Optional[float] = None, target_joints: Optional[List[float]] = None) -> int:
        """
        返回原点运动(GO HOME)
        :param wait: 是否等待运动完成，True=阻塞直到运动完成，False=立即返回
        :param velocity: 速度百分比（兼容参数，若提供可临时调整速度）
        :param acc: 加速度百分比
        :param target_joints: 目标原点关节角 (度, 6关节)
        :return: 接口返回码，（0=成功，负值=失败）
        """
        if velocity is not None:
            with self._sdk_lock:
                nrc.set_speed(self.fd, int(velocity))
        if acc is not None:
            with self._sdk_lock:
                nrc.set_acc(self.fd, int(acc))
        if target_joints is not None:
            return self.move_joint(target_joints, velocity=velocity, acc=acc, wait=wait)
        with self._sdk_lock:
            ret = nrc.robot_go_home(self.fd)
        if ret != nrc.SUCCESS:
            logger.error(f"[go_home] Failed to send go_home command, error code: {ret}")
            return ret
        if wait:
            self._wait_motion_done()
            logger.info(f"[go_home] go_home command sent successfully")
        return ret

    # -------------------------------------------------
    #  队列运动
    # -------------------------------------------------

    def _queue_local_size(self) -> int:
        """SDK 本地队列长度（与 queue_motion_size 一致）。"""
        with self._sdk_lock:
            res = nrc.queue_motion_size(self.fd, 0)
        if isinstance(res, (list, tuple)) and int(res[0]) == nrc.SUCCESS:
            return int(res[1])
        return -1

    def queue_start(self) -> int:
        """
        打开队列运动模式，并清空控制器端已存的旧队列数据.
        调用后一次通过 queue_push_l / queue_push_j 向本队列追加点位，
        所有点位准备就绪后调用 queue_send 一次性下发并触发运动。
        :return: 接口返回码，（0=成功，负值=失败）
        """
        self._queue_size = 0
        with self._sdk_lock:
            # 仅在上次队列仍打开时 stop，避免连续子项反复 stop 引发伺服不一致
            if self._queue_mode_active:
                nrc.queue_motion_stop_not_power_off(self.fd)
            if hasattr(nrc, "queue_motion_clear_Data"):
                nrc.queue_motion_clear_Data(self.fd)
            ret = nrc.queue_motion_set_status(self.fd, True)
        if ret != nrc.SUCCESS:
            logger.error(f"[queue_start] Failed to send queue_motion_set_status command, error code: {ret}")
        else:
            self._queue_mode_active = True
            logger.info(f"[queue_start] queue motion started successfully")
        return ret

    def queue_push_l(
        self, 
        pose: PoseLike,
        velocity: float = DEFAULT_VELOCITY,
        acc: float = DEFAULT_ACC,
        dec: float = DEFAULT_DEC,
        tool_num: int = 0,
        pl: int = 0,
    ) -> int:
        """
        向队列追加一条直线运动点位(MOVL)。
        :param pose: 目标位姿 RobotPose 或 [x(mm),y(mm),z(mm),A(rad),B(rad),C(rad)]
        :param velocity: 线速度，单位 mm/s，范围 (0, 1000]
        :param acc: 加速度，范围 (0, 100]
        :param dec: 减速度，范围 (0, 100]
        :param tool_num: 工具号
        :param pl: 平滑过渡参数，范围[0, 5]；0=精确到位（有停顿），1〜5=数值越大越平滑（不停顿）
        :return: 接口返回码，（0=成功，负值=失败）
        """
        cmd = self._make_movecmd(pose, velocity, acc, dec, tool_num, pl=pl)
        with self._sdk_lock:
            ret = nrc.queue_motion_push_back_moveL(self.fd, cmd)
        if ret != nrc.SUCCESS:
            logger.error(f"[queue_push_l] Failed to send queue_motion_push_back_moveL command, error code: {ret}")
            return ret
        self._queue_size += 1
        return ret

    def queue_push_j(
        self, 
        pose: PoseLike,
        velocity: float = DEFAULT_VELOCITY,
        acc: float = DEFAULT_ACC,
        dec: float = DEFAULT_DEC,
        tool_num: int = 0,
        pl: int = 0,
    ) -> int:
        """
        向队列追加一条关节运动点位(MOVJ)。
        :param pose: 目标位姿 RobotPose 或 [x(mm),y(mm),z(mm),A(rad),B(rad),C(rad)]
        :param velocity: 速度，单位 %（百分比），范围 (0, 100]
        :param acc: 加速度，范围 (0, 100]
        :param dec: 减速度，范围 (0, 100]
        :param tool_num: 工具号
        :param pl: 平滑过渡参数，范围[0, 5]；0=精确到位（有停顿），1〜5=数值越大越平滑（不停顿）
        :return: 接口返回码，（0=成功，负值=失败）
        """
        cmd = self._make_movecmd(pose, velocity, acc, dec, tool_num, pl=pl)
        with self._sdk_lock:
            ret = nrc.queue_motion_push_back_moveJ(self.fd, cmd)
        if ret != nrc.SUCCESS:
            logger.error(f"[queue_push_j] Failed to send queue_motion_push_back_moveJ command, error code: {ret}")
            return ret
        self._queue_size += 1
        return ret

    def queue_send(self, wait: bool = True) -> int:
        """
        将本地队列数据全部发送到控制器并触发运动。
        超过 31 条时自动分批发送（中间批次使用 isContinue = True 保证连续性，最后一批 isContinue=False 触发执行）。
        :param wait: 是否等待运动完成，True=阻塞直到运动完成，False=立即返回
        :return: 接口返回码，（0=成功，负值=失败）
        """
        if self._queue_size == 0:
            logger.warning("[queue_send] No motion data in queue.")
            return nrc.SUCCESS
        
        ret = self._queue_send_batched(wait)
        self._queue_size = 0
        return ret

    def queue_suspend(self) -> int:
        """ 暂停正在执行的队列运动，机器人平滑减速停止 """
        with self._sdk_lock:
            ret = nrc.queue_motion_suspend(self.fd)
        if ret != nrc.SUCCESS:
            logger.error(f"[queue_suspend] Failed to send queue_motion_suspend command, error code: {ret}")
        else:
            logger.info(f"[queue_suspend] queue motion suspended successfully")
        return ret

    def queue_resume(self) -> int:
        """ 从暂停状态恢复，继续执行剩余队列运动 """
        with self._sdk_lock:
            ret = nrc.queue_motion_restart(self.fd)
        if ret != nrc.SUCCESS:
            logger.error(f"[queue_resume] Failed to send queue_motion_restart command, error code: {ret}")
        else:
            logger.info(f"[queue_resume] queue motion restarted successfully")
        return ret

    def _queue_release(self) -> int:
        """
        退出队列运动模式（SDK：queue_motion_set_status(False)），释放示教器通行权。
        运动自然结束后也应调用；仅 stop 不停用队列模式时示教器可能仍显示队列占用。
        """
        if self.fd < 0:
            return RESULT_DISCONNECT
        with self._sdk_lock:
            nrc.queue_motion_stop_not_power_off(self.fd)
            if hasattr(nrc, "queue_motion_clear_Data"):
                nrc.queue_motion_clear_Data(self.fd)
            ret = nrc.queue_motion_set_status(self.fd, False)
        self._queue_mode_active = False
        if ret != nrc.SUCCESS:
            logger.error("[queue_release] set_status(False) failed, code=%s", ret)
        else:
            logger.info("[queue_release] queue motion mode closed (set_status False)")
        return ret

    def queue_stop(self) -> int:
        """
        停止队列运动、清空剩余指令并关闭队列模式（保持上电）。
        调用后本地队列计数也会被清零。
        """
        self._queue_size = 0
        if self.fd < 0:
            return RESULT_DISCONNECT
        # 已 release 且控制器无剩余时，避免再次 stop/set_status(False) 脉冲，
        # 全量测试末尾多次 queue_stop 易诱发「各轴伺服状态不一致」。
        if not self._queue_mode_active:
            rem = self.queue_get_remaining()
            if rem == 0:
                return nrc.SUCCESS
            if rem < 0:
                logger.warning("[queue_stop] queuelen 查询异常 ret=%s，仍执行 release", rem)
        ret = self._queue_release()
        return ret

    def queue_get_remaining(self) -> int:
        """ 
        查询控制器端队列中尚未执行的指令数量。
        :return: 剩余指令数量，查询失败时返回错误码（ < 0 ）
        """
        qlen = 0
        with self._sdk_lock:
            qlen = nrc.queue_motion_get_queuelen(self.fd, qlen)
        if qlen[0] != nrc.SUCCESS:
            logger.error(f"[queue_get_remaining] Failed to get queue length, error code: {qlen[0]}")
            return qlen[0]
        return qlen[1]

    # -------------------------------------------------
    #  队列运动(内部工具方法)
    # -------------------------------------------------

    def _queue_send_batched(self, wait: bool) -> int:
        """
        将本地队列发送到控制器（对齐 SDK 文档与官方 demo）。
        - size=0：发送本地队列全部指令
        - isContinue=True：继续缓存，机械臂暂不运动
        - isContinue=False：接收本批后立刻开始运动
        超过 31 条时：先发 31+continue，再发剩余（size=0 或剩余条数）+continue=False。
        """
        BATCH = 31
        ret = nrc.SUCCESS

        local_n = self._queue_local_size()
        if local_n < 0:
            logger.error("[queue_send_batched] queue_motion_size failed")
            return RESULT_RECEIVE_FAILED
        if local_n != self._queue_size:
            logger.warning(
                "[queue_send_batched] local size %s != _queue_size %s, use local",
                local_n, self._queue_size,
            )

        while local_n > 0:
            if local_n > BATCH:
                send_size, is_continue = BATCH, True
            else:
                # 最后一批：size=0 表示发送本地全部剩余（SDK 文档）
                send_size, is_continue = 0, False

            with self._sdk_lock:
                ret = nrc.queue_motion_send_to_controller(
                    self.fd, send_size, is_continue
                )
            if ret != nrc.SUCCESS:
                logger.error(
                    "[queue_send_batched] send failed size=%s continue=%s ret=%s",
                    send_size, is_continue, ret,
                )
                return ret
            logger.info(
                "[queue_send_batched] sent size=%s continue=%s (local before=%s)",
                send_size, is_continue, local_n,
            )
            local_n = self._queue_local_size()
            if local_n < 0:
                return RESULT_RECEIVE_FAILED
            if is_continue and local_n <= 0:
                logger.error("[queue_send_batched] local queue empty before final batch")
                return RESULT_EXCEPTION

        if wait:
            if not self._wait_queue_done():
                self._queue_release()
                return RESULT_TIMEOUT
            self._queue_release()

        return ret

    def _wait_queue_done(
        self,
        poll_interval: float = 0.025,
        timeout: float = 1800.0,
        stable_stop_sec: float = 0.35,
        pose_log_interval: float = 1.0,
        assume_motion_already_started: bool = False,
    ) -> bool:
        """
        等待队列执行完毕（对齐 demo：get_robot_running_state==0 且 queuelen==0）。
        默认须先观察到 running==2（执行中），再接受完成，避免 send 后未起步就误判完成。

        suspend/resume 后再次等待时，臂往往在暂停前已起步；若控制器在 resume 后
        长时间保持 queuelen=0、running=0（不再上报 running=2），应设
        assume_motion_already_started=True，否则会空等到 motion_deadline。
        """
        time.sleep(0.1)
        deadline = time.time() + timeout
        last_log = 0.0
        stop_since: Optional[float] = None
        motion_seen = bool(assume_motion_already_started)
        motion_wait_sec = 30.0
        motion_deadline = time.time() + motion_wait_sec
        resume_tried = False
        stall_ref_xyz: Optional[tuple] = None
        stall_since: Optional[float] = None
        prev_qlen_stall: int = -999
        ghost_ref_xyz: Optional[tuple] = None
        ghost_since: Optional[float] = None

        while time.time() < deadline:
            servo_st = self.get_servo_state()
            if servo_st == SERVO_STATE_ALARM:
                logger.error("[wait_queue_done] servo ALARM, abort")
                return False

            qlen = self.queue_get_remaining()
            run_st = self.get_running_state()
            now = time.time()

            if now - last_log >= pose_log_interval:
                last_log = now
                pose = self.get_current_pose()
                logger.info(
                    "[wait_queue_done] queuelen=%s running=%s pose=%s",
                    qlen, run_st, pose,
                )

            if qlen < 0:
                logger.error("[wait_queue_done] queue_get_remaining error: %s", qlen)
                return False

            if run_st == RUNNING_STATE_RUNNING:
                motion_seen = True

            if prev_qlen_stall != qlen:
                prev_qlen_stall = qlen
                stall_ref_xyz = None
                stall_since = None

            if motion_seen and run_st == RUNNING_STATE_RUNNING and qlen > 0:
                pose_s = self.get_current_pose()
                if pose_s is not None:
                    xyz = (float(pose_s.x), float(pose_s.y), float(pose_s.z))
                    if stall_ref_xyz is None:
                        stall_ref_xyz = xyz
                        stall_since = now
                    else:
                        d = math.sqrt(
                            (xyz[0] - stall_ref_xyz[0]) ** 2
                            + (xyz[1] - stall_ref_xyz[1]) ** 2
                            + (xyz[2] - stall_ref_xyz[2]) ** 2
                        )
                        if d >= 2.5:
                            stall_ref_xyz = xyz
                            stall_since = now
                        elif stall_since is not None and (now - stall_since) > 12.0:
                            logger.error(
                                "[wait_queue_done] 位姿停滞 %.1fs（running=2 queuelen=%s），"
                                "疑 suspend/队列状态异常，queue_stop 退出",
                                now - stall_since,
                                qlen,
                            )
                            self.queue_stop()
                            return False

            in_ghost = motion_seen and run_st == RUNNING_STATE_RUNNING and qlen == 0
            if in_ghost:
                pose_g = self.get_current_pose()
                if pose_g is not None:
                    xyzg = (float(pose_g.x), float(pose_g.y), float(pose_g.z))
                    if ghost_ref_xyz is None:
                        ghost_ref_xyz = xyzg
                        ghost_since = now
                    else:
                        dg = math.sqrt(
                            (xyzg[0] - ghost_ref_xyz[0]) ** 2
                            + (xyzg[1] - ghost_ref_xyz[1]) ** 2
                            + (xyzg[2] - ghost_ref_xyz[2]) ** 2
                        )
                        if dg >= 2.5:
                            ghost_ref_xyz = xyzg
                            ghost_since = now
                        elif ghost_since is not None and (now - ghost_since) > 10.0:
                            logger.error(
                                "[wait_queue_done] running=2 且 queuelen=0 位姿停滞 %.1fs"
                                "（疑 large 分批+resume），queue_stop",
                                now - ghost_since,
                            )
                            self.queue_stop()
                            return False
            if not in_ghost:
                ghost_ref_xyz = None
                ghost_since = None

            if not motion_seen and now > motion_deadline:
                logger.error(
                    "[wait_queue_done] no motion within %.0fs, abort", motion_wait_sec
                )
                return False

            if qlen > 0 and run_st == RUNNING_STATE_STOP:
                if not resume_tried:
                    logger.warning(
                        "[wait_queue_done] queuelen=%s but STOP, try queue_resume once",
                        qlen,
                    )
                    self.queue_resume()
                    resume_tried = True
                stop_since = None
                time.sleep(poll_interval)
                continue

            if motion_seen and run_st == RUNNING_STATE_STOP and qlen == 0:
                if stop_since is None:
                    stop_since = now
                elif now - stop_since >= stable_stop_sec:
                    logger.info("[wait_queue_done] Queue motion completed.")
                    return True
            else:
                stop_since = None

            time.sleep(poll_interval)

        logger.error("[wait_queue_done] timeout")
        return False

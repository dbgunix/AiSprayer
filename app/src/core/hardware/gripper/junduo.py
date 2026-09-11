import os
import sys
import time
import logging
import argparse
import threading
from enum import Enum
from typing import Tuple, Optional

# 路径设置：确保直接运行时能找到核心包
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
for p in [os.path.join(PROJECT_ROOT, "app/src"), os.path.join(PROJECT_ROOT, "src")]:
    if p not in sys.path:
        sys.path.insert(0, p)

# 导入越疆 V3 API 中的 Dashboard 类
try:
    from ..robot.dobot_api import DobotApiDashboard
except (ImportError, ValueError):
    try:
        from core.hardware.robot.dobot_api import DobotApiDashboard
    except ImportError:
        from aisprayer.core.hardware.robot.dobot_api import DobotApiDashboard

# 模块级日志器（不在模块层调用 basicConfig，避免覆盖主应用的日志配置）
logger = logging.getLogger(__name__)

class GripperState(Enum):
    MOVING = 0      # 运动中
    CLAMPED = 1     # 夹持成功 (夹到工件并保持力矩)
    ARRIVED = 2     # 到位 (未夹到物料/空抓闭合)
    FAULT = 3       # 故障/未初始化
    UNKNOWN = -1    # 通信未知/解析异常

class JunduoGripper:
    """
    钧舵 EPG50-060 平行夹爪工业级 API 控制封装类 (基于越疆 CR 控制器)
    
    协议参考: 钧舵 EPG/RG 系列 Modbus RTU 寄存器协议
    控制寄存器 (写):
        0x03E8 (1000) - 控制字: 0x0001=使能/回零, 0x0009=运动到目标位置
        0x03E9 (1001) - 目标位置: 高字节有效, 即 value << 8, 范围 0-255
        0x03EA (1002) - 速度+力: 高字节=力(0-255), 低字节=速度(0-255)
    状态寄存器 (读):
        0x07D0 (2000) - 状态字: 57=运动中, 241=正常到位, 177=夹持到位
        0x07D1 (2001) - 高8位=实时位置(0-255), 低8位=故障码
        0x07D2 (2002) - 高8位=保持力(0-255→0-60N), 低8位=瞬时驱动力
    """
    # --- 控制寄存器 (写) ---
    REG_CTRL = 0x03E8        # 控制字 (1000)
    REG_POSITION = 0x03E9    # 目标位置 (1001)
    REG_SPEED_FORCE = 0x03EA # 速度+力 (1002)
    # --- 状态寄存器 (读) ---
    REG_STATUS = 0x07D0      # 状态字 (2000)
    REG_POS_FAULT = 0x07D1   # 位置+故障 (2001)
    # --- 控制字常量 ---
    CMD_ENABLE = 0x0001      # 使能/回零
    CMD_MOVE = 0x0009        # 运动到目标位置

    # 越疆 CR 控制器 RTU 透传端口 (固定 60000)
    RTU_TRANSPARENT_PORT = 60000

    # -------------------------------------------------------------
    # 硬件规格参数 (Hardware Specifications) - 单一真实源
    # -------------------------------------------------------------
    MODEL = "EPG50-060"
    TOTAL_STROKE_MM = 50.0            # 夹爪最大开度/总有效行程 (mm)
    SINGLE_FINGER_STROKE_MM = 25.0    # 单侧手指对称滑动行程 (mm)
    MIN_STROKE_MM = 0.0               # 完全闭合行程 (mm)
    MAX_STROKE_MM = 50.0              # 完全张开行程 (mm)
    DEFAULT_STROKE_MM = 0.0           # 默认初始处于闭合状态 (mm)
    
    MAX_FORCE_N = 60.0                # 单侧最大持续保持夹持力 (N)
    MIN_FORCE_N = 0.0                 # 最小夹持力 (N)
    DEFAULT_FORCE_PERCENT = 50        # 默认推荐夹持力百分比 (1-100%)
    
    DEFAULT_SPEED_PERCENT = 50        # 默认推荐开合速度百分比 (1-100%)
    OPEN_SPEED_PERCENT = 80           # 张开快速动作推荐速度百分比 (1-100%)
    CLAMP_SPEED_PERCENT = 50          # 夹持动作推荐速度百分比 (1-100%)
    FULL_STROKE_TIME = 0.65           # 全行程开合时间 (s, 最大速度下)

    @classmethod
    def get_specs(cls) -> dict:
        """获取夹爪硬件规格与配置参数字典 (供上层服务与前端 UI 读取)"""
        return {
            "model": cls.MODEL,
            "total_stroke_mm": cls.TOTAL_STROKE_MM,
            "single_finger_stroke_mm": cls.SINGLE_FINGER_STROKE_MM,
            "min_stroke_mm": cls.MIN_STROKE_MM,
            "max_stroke_mm": cls.MAX_STROKE_MM,
            "default_stroke_mm": cls.DEFAULT_STROKE_MM,
            "max_force_n": cls.MAX_FORCE_N,
            "min_force_n": cls.MIN_FORCE_N,
            "default_force_percent": cls.DEFAULT_FORCE_PERCENT,
            "default_speed_percent": cls.DEFAULT_SPEED_PERCENT,
            "open_speed_percent": cls.OPEN_SPEED_PERCENT,
            "clamp_speed_percent": cls.CLAMP_SPEED_PERCENT,
            "full_stroke_time_s": cls.FULL_STROKE_TIME,
        }

    def __init__(self, dashboard: DobotApiDashboard, slave_id: int = 9):
        """
        :param dashboard: 已实例化的 DobotApiDashboard 对象
        :param slave_id: 夹爪 Modbus 从站号，钧舵 EPG 系列出厂默认为 9
        """
        self.dashboard = dashboard
        self.slave_id = slave_id
        self.device_index = -1  # ModbusCreate 返回的内部索引 (0-4)
        self.is_initialized = False
        # 心跳线程 (保持 Modbus 通信活跃，防止夹爪通信超时蓝灯闪烁)
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._heartbeat_stop = threading.Event()
        self.last_state: GripperState = GripperState.UNKNOWN
        self.last_position_mm: float = self.DEFAULT_STROKE_MM  # 默认初始处于闭合状态 (0.0 mm)
        self.last_force_n: float = 0.0       # 实时检测到的保持力 (N)
        self._io_lock = threading.Lock()     # 保护 Modbus RTU 读写防并发冲突

    def connect(self) -> bool:
        """
        建立与夹爪的 Modbus RTU 通信连接。
        必须在任何读写操作之前调用。
        
        越疆 CR 控制器通过末端 RS485 透传端口 (60000) 与 Modbus RTU 从站通信，
        ModbusCreate 会返回一个内部设备索引 (0-4)，后续所有 SetHoldRegs/GetHoldRegs
        的第一个参数都必须使用这个索引。
        
        :return: 是否连接成功
        """
        logger.info(f"正在建立 Modbus RTU 连接 (从站号={self.slave_id}, 端口={self.RTU_TRANSPARENT_PORT})...")
        try:
            with self._io_lock:
                res = self.dashboard.ModbusCreate(
                    "127.0.0.1", self.RTU_TRANSPARENT_PORT, self.slave_id, 1
                )
            # 返回格式: "0,{index},ModbusCreate(...)"  或  "-1,{},ModbusCreate(...)"
            if res and res.startswith("0"):
                # 解析返回的设备索引
                parts = res.split("{")
                if len(parts) > 1:
                    idx_str = parts[1].split("}")[0].strip()
                    self.device_index = int(idx_str)
                    logger.info(f"Modbus RTU 连接成功，设备索引: {self.device_index}")
                    return True
            logger.error(f"ModbusCreate 失败，返回: {res}")
            logger.error("请检查: 1) 夹爪RS485接线 2) 24V供电 3) 从站号是否正确")
            return False
        except Exception as e:
            logger.error(f"ModbusCreate 通信异常: {e}")
            return False

    def disconnect(self):
        """断开 Modbus RTU 连接"""
        self.stop_heartbeat()
        if self.device_index >= 0:
            try:
                with self._io_lock:
                    self.dashboard.ModbusClose(self.device_index)
                logger.info(f"已断开 Modbus RTU 连接 (索引={self.device_index})")
            except Exception as e:
                logger.warning(f"ModbusClose 异常: {e}")
            self.device_index = -1

    def start_heartbeat(self, idle_interval: float = 0.25, moving_interval: float = 0.03, interval: Optional[float] = None):
        """
        自适应心跳与状态轮询线程：
        - 运动中 (MOVING)：采用 30ms (~33Hz) 极速轮询，提供平滑轨迹采样；
        - 静止时 (IDLE/ARRIVED/CLAMPED)：自动回退至 250ms 低频保活，避免总线拥堵。
        :param idle_interval: 静止状态下的心跳保活间隔 (s)，默认 0.25s
        :param moving_interval: 运动状态下的高频采样间隔 (s)，默认 0.03s (30ms)
        :param interval: 兼容旧接口传参
        """
        if interval is not None:
            idle_interval = interval
        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            return
        self._heartbeat_stop.clear()

        def _poll():
            while not self._heartbeat_stop.is_set():
                if self.device_index >= 0:
                    state, pos, force = self.update_telemetry()
                    logger.debug(f"夹爪状态: {state.name}, 位置: {pos}mm, 保持力: {force}N")
                    curr_interval = moving_interval if state == GripperState.MOVING else idle_interval
                else:
                    curr_interval = idle_interval
                self._heartbeat_stop.wait(curr_interval)

        self._heartbeat_thread = threading.Thread(target=_poll, daemon=True, name="gripper-heartbeat")
        self._heartbeat_thread.start()
        logger.debug("夹爪自适应高频心跳线程已启动")

    def stop_heartbeat(self):
        """停止心跳线程"""
        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            self._heartbeat_stop.set()
            self._heartbeat_thread.join(timeout=2.0)
            logger.debug("夹爪心跳线程已停止")

    def _write_regs(self, start_addr: int, count: int, values: list) -> bool:
        """底层写入封装：下发 Modbus 控制参数"""
        if self.device_index < 0:
            logger.error("尚未建立 Modbus RTU 连接，请先调用 connect()")
            return False
        val_str = "{" + ",".join(map(str, values)) + "}"
        try:
            with self._io_lock:
                # 第一个参数是 ModbusCreate 返回的设备索引 (0-4)，不是从站号！
                res = self.dashboard.SetHoldRegs(self.device_index, start_addr, count, val_str, "U16", timeout=2.0)
            # 越疆返回 0 代表下发成功，格式通常为: "0,{},SetHoldRegs(...)"
            if res and res.startswith("0"):
                return True
            logger.error(f"SetHoldRegs 失败，返回: {res}")
            return False
        except Exception as e:
            logger.error(f"SetHoldRegs 通信异常: {e}")
            return False

    def _read_reg(self, addr: int, timeout: float = 1.5) -> int:
        """底层读取封装：读取单个保持寄存器"""
        if self.device_index < 0:
            return -1
        try:
            with self._io_lock:
                res = self.dashboard.GetHoldRegs(self.device_index, addr, 1, "U16", timeout=timeout)
            # 返回格式: "0,{val},GetHoldRegs(...)"
            if res and res.startswith("0"):
                parts = res.split("{")
                if len(parts) > 1:
                    raw_val = parts[1].split("}")[0].strip()
                    if "," in raw_val:
                        raw_val = raw_val.split(",")[0].strip()
                    try:
                        return int(raw_val)
                    except ValueError:
                        logger.warning(f"GetHoldRegs(0x{addr:04X}) 无法解析整数: '{raw_val}'")
            else:
                logger.debug(f"GetHoldRegs(0x{addr:04X}) 返回: {res}")
        except Exception as e:
            logger.warning(f"GetHoldRegs(0x{addr:04X}) 异常: {e}")
        return -1

    def _read_regs(self, addr: int, count: int, timeout: float = 1.5) -> list:
        """底层批量读取封装：读取连续保持寄存器 (单包 Modbus 批量查询)"""
        if self.device_index < 0:
            return []
        try:
            with self._io_lock:
                res = self.dashboard.GetHoldRegs(self.device_index, addr, count, "U16", timeout=timeout)
            if res and res.startswith("0"):
                parts = res.split("{")
                if len(parts) > 1:
                    raw_str = parts[1].split("}")[0].strip()
                    if raw_str:
                        result = []
                        for v in raw_str.split(","):
                            v = v.strip()
                            if v and (v.isdigit() or (v.startswith('-') and v[1:].isdigit())):
                                result.append(int(v))
                        return result
            else:
                logger.debug(f"GetHoldRegs(0x{addr:04X}, count={count}) 返回: {res}")
        except Exception as e:
            logger.warning(f"GetHoldRegs(0x{addr:04X}, count={count}) 异常: {e}")
        return []

    def update_telemetry(self) -> Tuple[GripperState, float, float]:
        """
        单包合并读取状态(0x07D0)、位置(0x07D1)、保持力(0x07D2)连续 3 个寄存器，
        大幅削减 Modbus 通信往返与网络开销。
        """
        regs = self._read_regs(self.REG_STATUS, 3)
        if len(regs) >= 3:
            # 1. 状态解析 (0x07D0)
            raw_status = regs[0]
            if not (raw_status & 0x0080):
                state = GripperState.MOVING
            elif raw_status & 0x0040:
                state = GripperState.ARRIVED
            else:
                state = GripperState.CLAMPED
            self.last_state = state

            # 2. 位置解析 (0x07D1 高 8 位: 0=全开 50mm, 255=全闭 0mm)
            pos_raw = (regs[1] >> 8) & 0xFF
            stroke_mm = round((255 - pos_raw) * (self.TOTAL_STROKE_MM / 255.0), 1)
            self.last_position_mm = stroke_mm

            # 3. 保持力解析 (0x07D2 高 8 位)
            force_raw = (regs[2] >> 8) & 0xFF
            force_n = round(force_raw * self.MAX_FORCE_N / 255.0, 1)
            self.last_force_n = force_n

            return state, stroke_mm, force_n

        # 单包批量读取失败时，不继续执行 3 次独立的单寄存器查询，避免在 RS485 未就绪时长时间锁死 29999 端口
        return self.last_state, self.last_position_mm, self.last_force_n
    def init_gripper(self, timeout: float = 5.0) -> bool:
        """
        夹爪初始化 / 上电回零 (动作前必须调用一次)
        会自动建立 Modbus RTU 连接 (如果尚未连接)。
        使能操作: 向 0x03E8 写入 0x0001，夹爪执行行程搜索 (全开→全闭)。
        :param timeout: 超时时间 (秒)
        :return: 是否初始化成功
        """
        # 确保已建立 Modbus RTU 连接
        if self.device_index < 0:
            if not self.connect():
                logger.error("夹爪初始化失败: 无法建立 Modbus RTU 连接")
                return False

        logger.info("正在初始化钧舵夹爪 (使能与回零)...")
        if not self._write_regs(self.REG_CTRL, 1, [self.CMD_ENABLE]):
            return False

        # 等待回零完成 (回零以默认最大速度运行，约 FULL_STROKE_TIME + 余量)
        homing_time = self.FULL_STROKE_TIME + 0.5  # ≈1.15s
        start_time = time.time()
        while time.time() - start_time < timeout:
            state = self.get_state()
            if state in (GripperState.CLAMPED, GripperState.ARRIVED):
                self.is_initialized = True
                logger.info(f"钧舵夹爪初始化成功！状态: {state.name}")
                return True
            elif state == GripperState.FAULT:
                logger.error("夹爪初始化报故障！")
                return False
            # 超过预期回零时间，视为完成
            if time.time() - start_time >= homing_time:
                break
            time.sleep(0.2)

        self.is_initialized = True
        logger.info("钧舵夹爪使能完成")
        return True

    def move(self, position: int, force_percent: int = 50, speed: int = 50, wait_complete: bool = True, timeout: float = 3.0) -> Tuple[bool, GripperState]:
        """
        控制夹爪运动的主函数
        
        协议流程:
            1. 写 0x03E9: 目标位置 (value << 8)
            2. 写 0x03EA: (force << 8) + speed
            3. 写 0x03E8: 0x0009 (触发运动)
        
        :param position: 目标位置 0~1000 (0=完全闭合, 1000=完全张开)
        :param force_percent: 最大允许力 0~100 (%), 对应 0~60N
        :param speed: 目标速度 0~100 (%)
        :param wait_complete: 是否等待动作完成
        :param timeout: 等待超时时间 (秒)
        :return: (是否成功执行, 最终夹爪状态)
        """
        if not self.is_initialized:
            logger.warning("夹爪尚未初始化，尝试自动初始化...")
            if not self.init_gripper():
                return False, GripperState.FAULT

        # 参数转换: API范围(0-1000) → 硬件范围(0-255)
        # 注意: 硬件协议 0=完全打开, 255=完全闭合，与 API 方向相反，需取反
        pos_raw = max(0, min(255, 255 - int(position * 255 / 1000)))
        force_raw = max(0, min(255, int(force_percent * 255 / 100)))
        speed_raw = max(0, min(255, int(speed * 255 / 100)))

        logger.info(f"夹爪运动 -> 位置: {position}({pos_raw}/255), "
                    f"力: {force_percent}%({force_raw}), 速度: {speed}%({speed_raw})")

        # Step 1: 写目标位置到 0x03E9 (高字节有效: value << 8)
        pos_reg = pos_raw << 8
        if not self._write_regs(self.REG_POSITION, 1, [pos_reg]):
            return False, GripperState.FAULT

        # Step 2: 写速度+力到 0x03EA (高字节=力, 低字节=速度)
        sf_reg = (force_raw << 8) | speed_raw
        if not self._write_regs(self.REG_SPEED_FORCE, 1, [sf_reg]):
            return False, GripperState.FAULT

        # Step 3: 写控制字 0x0009 到 0x03E8 触发运动
        if not self._write_regs(self.REG_CTRL, 1, [self.CMD_MOVE]):
            return False, GripperState.FAULT

        # 立即更新状态为 MOVING，触发后台自适应心跳轮询瞬间切入 30ms 极速采样模式
        self.last_state = GripperState.MOVING

        # 如果无需阻塞等待，下发完成后直接返回
        if not wait_complete:
            return True, GripperState.MOVING

        # 根据速度计算预期运动时间: 全行程时间 × (255 / speed_raw)
        if speed_raw > 0:
            expected_time = self.FULL_STROKE_TIME * (255.0 / speed_raw) + 0.3  # +0.3s 余量
        else:
            expected_time = 20.0  # speed=0 时用最大等待
        # 限制在合理范围内
        expected_time = max(1.0, min(expected_time, 30.0))
        actual_timeout = max(timeout, expected_time)

        logger.debug(f"预期运动时间: {expected_time:.1f}s (speed_raw={speed_raw})")

        # 阻塞等待运动完成
        start_time = time.time()
        while time.time() - start_time < actual_timeout:
            state = self.get_state()
            if state in (GripperState.CLAMPED, GripperState.ARRIVED):
                elapsed = time.time() - start_time
                logger.info(f"夹爪动作完成，状态: {state.name} (用时 {elapsed:.2f}s)")
                return True, state
            elif state == GripperState.FAULT:
                logger.error("夹爪运动过程中报故障！")
                return False, state
            # 超过预期时间且状态未更新，视为完成
            if time.time() - start_time >= expected_time:
                logger.info(f"夹爪运动完成 (等待 {expected_time:.1f}s, 状态: {state.name})")
                return True, state if state != GripperState.UNKNOWN else GripperState.ARRIVED
            time.sleep(0.1)

        # 超时处理: 写入均已成功，视为完成
        logger.info("夹爪指令已下发完成 (等待超时但写入成功)")
        return True, GripperState.ARRIVED

    def open(self, force_percent: int = 50, speed: int = 80, wait_complete: bool = True) -> bool:
        """快捷调用：完全张开夹爪"""
        success, _ = self.move(position=1000, force_percent=force_percent, speed=speed, wait_complete=wait_complete)
        return success

    def clamp(self, force_percent: int = 50, speed: int = 50, wait_complete: bool = True) -> Tuple[bool, bool, float]:
        """
        快捷调用：执行夹持动作
        通过状态寄存器直接判断: 177(0x00B1)=夹持到位, 241(0x00F1)=空抓到位
        :param force_percent: 最大允许夹持力 0~100 (%), 对应 0~60N
        :param speed: 闭合速度 0~100 (%)
        :return: (指令是否成功发送, 是否成功夹持到工件, 保持力/N)
        """
        success, state = self.move(position=0, force_percent=force_percent, speed=speed, wait_complete=wait_complete)
        is_object_clamped = (state == GripperState.CLAMPED)
        # 读取夹持后的保持力 (0x07D2 高字节, 0-255 对应 0-60N)
        sf = self._read_reg(0x07D2)
        hold_force_raw = (sf >> 8) & 0xFF if sf >= 0 else 0
        hold_force_n = round(hold_force_raw * self.MAX_FORCE_N / 255.0, 1)
        logger.info(f"夹持结果: state={state.name}, "
                    f"保持力={hold_force_raw}({hold_force_n}N), "
                    f"{'夹持成功' if is_object_clamped else '空抓'}")
        return success, is_object_clamped, hold_force_n

    def get_state(self) -> GripperState:
        """
        获取当前夹爪状态 (读取 0x07D0 状态寄存器)
        
        EPG 系列实际状态编码 (经实测验证):
            57  (0x0039) = 运动中       (bit7=0)
            241 (0x00F1) = 正常到位     (bit7=1, bit6=1)
            177 (0x00B1) = 夹持到位     (bit7=1, bit6=0) —— 遇到阻力停止
        判断逻辑:
            bit7=0 → MOVING
            bit7=1, bit6=1 → ARRIVED (到达目标)
            bit7=1, bit6=0 → CLAMPED (夹到物体)
        """
        raw = self._read_reg(self.REG_STATUS)
        if raw < 0:
            return GripperState.UNKNOWN
        if not (raw & 0x0080):  # bit7=0: 运动中
            return GripperState.MOVING
        # bit7=1: 已停止
        if raw & 0x0040:  # bit6=1: 正常到达目标
            return GripperState.ARRIVED
        else:  # bit6=0: 被物体阻挡停止
            return GripperState.CLAMPED

    def get_position(self) -> float:
        """
        获取当前夹爪实时张开行程 (0.0 ~ 50.0 mm)
        读取 0x07D1 寄存器高 8 位 (0-255)
        0 = 完全张开 (50.0mm), 255 = 完全闭合 (0.0mm)
        """
        raw = self._read_reg(self.REG_POS_FAULT)
        if raw < 0:
            return self.last_position_mm
        pos_raw = (raw >> 8) & 0xFF
        stroke_mm = round((255 - pos_raw) * (self.TOTAL_STROKE_MM / 255.0), 1)
        self.last_position_mm = stroke_mm
        return stroke_mm

    def get_hold_force(self) -> float:
        """获取当前夹持力 (0.0 ~ 60.0 N)"""
        sf = self._read_reg(0x07D2)
        if sf < 0:
            return self.last_force_n
        hold_force_raw = (sf >> 8) & 0xFF
        force_n = round(hold_force_raw * self.MAX_FORCE_N / 255.0, 1)
        self.last_force_n = force_n
        return force_n

    def move_stroke(self, stroke_mm: float, force_percent: Optional[int] = None, speed: Optional[int] = None, wait_complete: bool = False, timeout: float = 3.0) -> Tuple[bool, GripperState]:
        """
        按物理行程控制夹爪开合
        :param stroke_mm: 目标开度 (MIN_STROKE_MM ~ TOTAL_STROKE_MM)
        :param force_percent: 夹持力比例 1~100 (%)，默认使用 DEFAULT_FORCE_PERCENT
        :param speed: 运动速度 1~100 (%)，默认使用 DEFAULT_SPEED_PERCENT
        :param wait_complete: 是否等待到位
        :param timeout: 超时时间 (s)
        :return: (是否成功, 最终状态)
        """
        fp = self.DEFAULT_FORCE_PERCENT if force_percent is None else max(1, min(100, int(force_percent)))
        sp = self.DEFAULT_SPEED_PERCENT if speed is None else max(1, min(100, int(speed)))
        stroke = max(self.MIN_STROKE_MM, min(self.TOTAL_STROKE_MM, float(stroke_mm)))
        pos_1000 = int(round((stroke / self.TOTAL_STROKE_MM) * 1000))
        return self.move(position=pos_1000, force_percent=fp, speed=sp, wait_complete=wait_complete, timeout=timeout)

    def get_status_dict(self) -> dict:
        """获取夹爪完整状态字典"""
        return {
            "connected": self.device_index >= 0,
            "initialized": self.is_initialized,
            "state": self.last_state.name,
            "position_mm": self.last_position_mm,
            "force_n": self.last_force_n,
            "specs": self.get_specs(),
        }


# ==========================================
# 工业场景集成与测试示例
# ==========================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="钧舵 EPG 电动夹爪控制工具")
    parser.add_argument("--slave-id", type=int, default=9,
                        help="Modbus 从站号 (钧舵 EPG 出厂默认 9)")
    parser.add_argument("--ip", type=str, default="192.168.5.1",
                        help="越疆控制器 IP 地址")
    parser.add_argument("--port", type=int, default=29999,
                        help="越疆控制器 Dashboard 端口")
    args = parser.parse_args()

    # 配置日志输出格式（仅在直接运行时生效）
    logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')

    try:
        # 1. 建立与越疆控制器的 Dashboard 端口连接
        logger.info(f"正在连接越疆控制器 {args.ip}:{args.port} ...")
        dashboard = DobotApiDashboard(args.ip, args.port)

        # 2. 实例化钧舵夹爪对象
        gripper = JunduoGripper(dashboard=dashboard, slave_id=args.slave_id)

        # 3. 初始化夹爪 (内部会自动调用 ModbusCreate 建立 RTU 连接)
        if not gripper.init_gripper():
            logger.error("夹爪初始化失败，程序退出")
            sys.exit(1)

        # 4. 启动心跳线程保持通信活跃 (防止蓝灯闪烁)
        gripper.start_heartbeat(interval=0.5)

        # 5. 交互式键盘控制循环
        print("\n" + "=" * 40)
        print("  钧舵 EPG50-060 夹爪键盘控制")
        print("=" * 40)
        print("  o = 张开    c = 夹取    q = 退出")
        print("  数字 0-100 = 移动到指定位置 (%)")
        print("=" * 40 + "\n")

        while True:
            try:
                cmd = input("夹爪> ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                print("\n退出")
                break

            if cmd == 'q':
                print("退出")
                break
            elif cmd == 'o':
                gripper.open(speed=80)
            elif cmd == 'c':
                ok, clamped, force_n = gripper.clamp(force_percent=60, speed=4)
                if ok and clamped:
                    print(f"  → 夹持成功 (检测到工件, 保持力={force_n}N)")
                elif ok:
                    print("  → 闭合完成 (未检测到工件)")
            elif cmd.isdigit():
                pos = int(cmd) * 10  # 0-100 → 0-1000
                gripper.move(position=pos, speed=60)
            elif cmd == '':
                continue
            else:
                print("  无效指令: o=张开, c=夹取, 0-100=位置, q=退出")

    except Exception as e:
        logger.critical(f"系统运行异常: {e}")
    finally:
        # 无论成功失败，都释放 Modbus 连接 (避免占用控制器槽位)
        try:
            gripper.disconnect()
        except Exception:
            pass
        try:
            dashboard.close()
        except Exception:
            pass
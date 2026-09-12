import logging
import time
import threading
import math
from typing import Optional, List

from .base_driver import BaseRobotDriver, RobotPose, PoseLike, _to_list
from .dobot_api import (
    DobotApiDashboard, DobotApiMove, DobotApiFeedBack,
    DobotApiError, DobotTimeoutError, parse_response,
    parse_error_id, alarmAlarmJsonFile, _alarm_index, _describe_alarm
)

logger = logging.getLogger(__name__)

class DobotDriver(BaseRobotDriver):
    def __init__(self, ip: str = "192.168.5.1", dashboard_port: int = 29999, move_port: int = 30003, toolnum: int = 0):
        self.ip = ip
        self.dashboard_port = dashboard_port
        self.move_port = move_port
        self.tool_num = toolnum
        self.user = 0
        self.dashboard: Optional[DobotApiDashboard] = None
        self.move: Optional[DobotApiMove] = None
        self.feedback: Optional[DobotApiFeedBack] = None
        self._connected = False
        self._global_speed = 10
        # 30004 real-time data cache
        self._cached_joint: Optional[List[float]] = None   # degrees
        self._cached_pose: Optional[List[float]] = None    # mm, degrees (tool_vector_actual)
        self._cached_tcp_speed_actual: Optional[List[float]] = None # 前三分量 m/s, 后三分量 rad/s (单位换算见 get_feedback_diagnostics)
        self._cached_qd_actual: Optional[List[float]] = None        # deg/s (qd_actual)
        self._cached_load: float = 0.0                             # kg (load)
        self._cached_error_status: int = 0                         # int (error_status)
        self._cached_speed_l: float = 10.0
        self._cached_acc_l: float = 10.0
        self._cached_speed_j: float = 10.0
        self._cached_acc_j: float = 10.0
        self._cached_running_status: int = 0
        self._cached_hand_type: List[int] = [0, 0, 0, 0]   # 1008~1011 手系 (int8 x4)
        self._cached_tool_index: int = 0                    # 1013 当前工具坐标系索引
        self._cached_run_queued_cmd: int = 0                # 1014 算法队列运行标志 / 当前执行段序号
        self._cached_velocity_ratio: int = 0                # 1016 关节速度比例 (%)
        self._cached_digital_output_bits: int = 0           # 0016~0023 数字输出端子状态（按位）
        self._feedback_thread: Optional[threading.Thread] = None
        self._stop_feedback = False

    def startup(self, timeout: float = 10.0) -> bool:
        try:
            self.dashboard = DobotApiDashboard(self.ip, self.dashboard_port)
            self.move = DobotApiMove(self.ip, self.move_port)
            self._connected = True
            
            # Enable robot
            r = self.dashboard.ClearError()
            logger.info(f"ClearError: {r}")
            time.sleep(0.5)
            r = self.dashboard.EnableRobot()
            logger.info(f"EnableRobot: {r}")
            
            if self.tool_num is not None and self.tool_num >= 0:
                self.set_tool_number(self.tool_num)
                logger.info(f"Dobot tool number set to {self.tool_num}")

            # Start 30004 real-time feedback thread
            self._start_feedback_thread()
                
            return True
        except Exception as e:
            logger.error(f"Failed to start Dobot: {e}")
            self._connected = False
            return False

    def shutdown(self) -> None:
        self._stop_feedback_thread()
        if self._connected and self.dashboard:
            try:
                r = self.dashboard.DisableRobot()
                logger.info(f"DisableRobot: {r}")
                self.dashboard.close()
            except Exception as e:
                logger.warning(f"Error closing Dobot dashboard: {e}")
        if self._connected and self.move:
            try:
                self.move.close()
            except Exception as e:
                logger.warning(f"Error closing Dobot move socket: {e}")
        self._connected = False

    def _start_feedback_thread(self):
        """Connect to port 30004 and start real-time data reading thread."""
        try:
            self.feedback = DobotApiFeedBack(self.ip, 30004)
            self._stop_feedback = False
            self._feedback_thread = threading.Thread(
                target=self._feedback_loop, daemon=True, name="dobot-feedback-30004"
            )
            self._feedback_thread.start()
            logger.info("Dobot 30004 real-time feedback thread started")
        except Exception as e:
            logger.warning(f"Could not connect to 30004 feedback port: {e}. Falling back to polling.")
            self.feedback = None

    def _stop_feedback_thread(self):
        self._stop_feedback = True
        if self._feedback_thread:
            self._feedback_thread.join(timeout=1.0)
            self._feedback_thread = None
        if self.feedback:
            try:
                self.feedback.close()
            except Exception as e:
                logger.warning(f"Error closing feedback socket: {e}")
            self.feedback = None

    def _feedback_loop(self):
        """Continuously read from 30004 at ~200Hz and cache joint/pose/dynamics data."""
        while not self._stop_feedback and self._connected:
            try:
                data = self.feedback.feedBackData()
                if data is not None and len(data) > 0:
                    d = data[0]
                    # q_actual: joint angles in degrees
                    self._cached_joint = d['q_actual'].tolist()
                    # tool_vector_actual: [X(mm), Y(mm), Z(mm), Rx(deg), Ry(deg), Rz(deg)]
                    self._cached_pose = d['tool_vector_actual'].tolist()
                    
                    if 'TCP_speed_actual' in d.dtype.names:
                        tcp_spd = d['TCP_speed_actual']
                        raw_spd = tcp_spd.tolist() if hasattr(tcp_spd, 'tolist') else [float(x) for x in tcp_spd]
                        self._cached_tcp_speed_actual = [0.0 if abs(x) < 1e-3 else float(x) for x in raw_spd]
                    if 'qd_actual' in d.dtype.names:
                        qd = d['qd_actual']
                        raw_qd = qd.tolist() if hasattr(qd, 'tolist') else [float(x) for x in qd]
                        self._cached_qd_actual = [0.0 if abs(x) < 1e-3 else float(x) for x in raw_qd]
                    if 'load' in d.dtype.names:
                        load_val = float(d['load'].item())
                        self._cached_load = 0.0 if abs(load_val) < 1e-3 else load_val
                    if 'error_status' in d.dtype.names:
                        self._cached_error_status = int(d['error_status'].item())

                    if 'xyz_velocity_ratio' in d.dtype.names:
                        self._cached_speed_l = float(d['xyz_velocity_ratio'].item())
                    if 'xyz_acceleration_ratio' in d.dtype.names:
                        self._cached_acc_l = float(d['xyz_acceleration_ratio'].item())
                    if 'r_velocity_ratio' in d.dtype.names:
                        self._cached_speed_j = float(d['r_velocity_ratio'].item())
                    if 'r_acceleration_ratio' in d.dtype.names:
                        self._cached_acc_j = float(d['r_acceleration_ratio'].item())
                    if 'running_status' in d.dtype.names:
                        self._cached_running_status = int(d['running_status'].item())
                    if 'hand_type' in d.dtype.names:
                        ht = d['hand_type']
                        self._cached_hand_type = [int(x) for x in (ht.tolist() if hasattr(ht, 'tolist') else list(ht))]
                    if 'tool_index' in d.dtype.names:
                        self._cached_tool_index = int(d['tool_index'].item())
                    if 'run_queued_cmd' in d.dtype.names:
                        self._cached_run_queued_cmd = int(d['run_queued_cmd'].item())
                    if 'velocity_ratio' in d.dtype.names:
                        self._cached_velocity_ratio = int(d['velocity_ratio'].item())
                    if 'digital_output_bits' in d.dtype.names:
                        self._cached_digital_output_bits = int(d['digital_output_bits'].item())
            except DobotTimeoutError as e:
                # 反馈端口每 8ms 推一包，偶发超时视为可恢复，连续失败才退出
                if self._stop_feedback:
                    break
                logger.warning(f"30004 feedback timeout: {e}")
            except Exception as e:
                if not self._stop_feedback:
                    logger.error(f"Feedback read error: {e}")

    def get_feedback_diagnostics(self) -> dict:
        """获取实时动力学与诊断反馈数据 (TCP 笛卡尔实际速度, 实际关节速度, 负载重量, 报警状态, DO 状态等)"""
        tcp_spd = self._cached_tcp_speed_actual or [0.0]*6
        # 计算 TCP 线速度合量：控制器反馈的 Vx/Vy/Vz 为 m/s (与前端 HUD 约定一致)，
        # 取欧几里得范数后 x 1000 换算为 mm/s
        tcp_speed_mm_s = math.sqrt(sum(v**2 for v in tcp_spd[:3])) * 1000
        do_bits = self._cached_digital_output_bits
        digital_outputs = [(do_bits >> i) & 1 for i in range(16)]
        return {
            "tcp_speed_actual": tcp_spd,
            "tcp_speed_mm_s": tcp_speed_mm_s,  # 预计算标量，单位 mm/s
            "qd_actual": self._cached_qd_actual or [0.0]*6,
            "load": self._cached_load,
            "error_status": self._cached_error_status,
            "tool_vector_actual": self._cached_pose or [0.0]*6,
            "hand_type": self._cached_hand_type,         # int8 x4，手系配置
            "tool_index": self._cached_tool_index,       # 当前工具坐标系索引
            "run_queued_cmd": self._cached_run_queued_cmd,  # 算法队列当前执行段序号
            "velocity_ratio": self._cached_velocity_ratio,      # 1016 关节速度比例 (%)
            "xyz_velocity_ratio": int(self._cached_speed_l),    # 1019 笛卡尔位置速度比例 (%)
            "r_velocity_ratio": int(self._cached_speed_j),      # 1020 笛卡尔姿态速度比例 (%)
            "digital_outputs": digital_outputs,                 # 16路数字输出状态 [DO1..DO16]
            "digital_output_bits": do_bits,                     # 64位数字输出端子状态位掩码
        }

    def get_running_state(self) -> int:
        if not self._connected:
            return 0
        try:
            mode = self._cached_running_status
            return mode
        except Exception as e:
            return 0

    def get_error_details(self) -> List[str]:
        if not self._connected or not self.dashboard:
            return []
        try:
            reply = self.dashboard.GetErrorID()
            controller, servos = parse_error_id(reply)
            
            controller_table, servo_table = alarmAlarmJsonFile()
            controller_index = _alarm_index(controller_table)
            servo_index = _alarm_index(servo_table)
            
            msgs = []
            for err_id in controller:
                msgs.append(f"Controller[{err_id}]: {_describe_alarm(err_id, controller_index, 'zh_CN')}")
                
            for idx, s_list in enumerate(servos):
                for err_id in s_list:
                    msgs.append(f"Servo{idx+1}[{err_id}]: {_describe_alarm(err_id, servo_index, 'zh_CN')}")
                    
            return msgs
        except Exception as e:
            logger.error(f"Failed to get error details: {e}")
            return []

    def is_robot_idle(self) -> bool:
        return self.get_running_state() == 0

    def get_current_pose(self) -> Optional[RobotPose]:
        if not self._connected:
            return None
        # Fast path: use real-time 30004 cache
        if self._cached_pose is not None:
            try:
                x, y, z, rx, ry, rz = self._cached_pose[:6]
                # tool_vector_actual: mm, degrees → RobotPose expects mm, radians
                return RobotPose(x, y, z, math.radians(rx), math.radians(ry), math.radians(rz))
            except Exception:
                pass
        # Fallback: poll via dashboard
        try:
            res = self.dashboard.GetPose()
            if res:
                if res.startswith("Error"):
                    return None
                start_idx = res.find('{')
                end_idx = res.find('}')
                if start_idx != -1 and end_idx != -1:
                    vals_str = res[start_idx+1:end_idx].split(',')
                else:
                    parts = res.split('{')
                    vals_str = parts[1].split('}')[0].split(',') if len(parts) > 1 else []
                if len(vals_str) >= 6:
                    x, y, z, rx, ry, rz = [float(v) for v in vals_str[:6]]
                    return RobotPose(x, y, z, math.radians(rx), math.radians(ry), math.radians(rz))
        except Exception as e:
            logger.error(f"Error parsing Dobot pose: {e}")
        return None

    def get_current_joint(self) -> Optional[List[float]]:
        if not self._connected:
            return None
        # Fast path: use real-time 30004 cache (degrees)
        if self._cached_joint is not None:
            return list(self._cached_joint)
        # Fallback: poll via dashboard
        try:
            res = self.dashboard.GetAngle()
            if res:
                if res.startswith("Error"):
                    return None
                start_idx = res.find('{')
                end_idx = res.find('}')
                if start_idx != -1 and end_idx != -1:
                    vals_str = res[start_idx+1:end_idx].split(',')
                else:
                    parts = res.split('{')
                    vals_str = parts[1].split('}')[0].split(',') if len(parts) > 1 else []
                if len(vals_str) >= 6:
                    return [float(v) for v in vals_str[:6]]
        except Exception as e:
            logger.error(f"Error parsing Dobot joint angles: {e}")
        return None

    def is_reachable(self, pose: PoseLike, movetype: str = "MOVJ") -> bool:
        if not self._connected or not self.dashboard:
            return False
        lst = _to_list(pose)
        # Convert radians back to degrees for checking
        rx_deg = math.degrees(lst[3])
        ry_deg = math.degrees(lst[4])
        rz_deg = math.degrees(lst[5])
        try:
            # InverseSolution(x,y,z,rx,ry,rz,user,tool)
            res = self.dashboard.InverseSolution(lst[0], lst[1], lst[2], rx_deg, ry_deg, rz_deg, 0, self.tool_num)
            # 逆解成功时 ErrorID 为 0 且返回六个关节角
            response = parse_response(res)
            if response.ok and len(response.values) >= 6:
                return True
        except Exception:
            pass
        return False

    def _wait_motion_done(self, timeout: float = 600.0) -> bool:
        # Wait up to 1 second for the robot to register the motion and leave Idle state
        start_wait = time.time()
        while time.time() - start_wait < 1.0:
            if not self.is_robot_idle():
                break
            time.sleep(0.05)
            
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.is_robot_idle():
                return True
            time.sleep(0.05)
        return False

    def move_j(self, pose: PoseLike, velocity: float = 10.0, acc: float = 20.0, dec: float = 20.0, tool_num: Optional[int] = None, wait: bool = True) -> int:
        if not self._connected or not self.move:
            return -2
        lst = _to_list(pose)
        x, y, z = lst[0], lst[1], lst[2]
        rx_deg, ry_deg, rz_deg = math.degrees(lst[3]), math.degrees(lst[4]), math.degrees(lst[5])
        tool = self.tool_num if tool_num is None else tool_num
        
        try:
            # Fix Dobot firmware parsing bug where "-0.000000" causes -30004 error
            # Adding 0.0 converts -0.0 to 0.0 in Python.
            rx_deg, ry_deg, rz_deg = rx_deg + 0.0, ry_deg + 0.0, rz_deg + 0.0
            r = self.move.MovJ(x, y, z, rx_deg, ry_deg, rz_deg, tool=tool, speedJ=velocity, accJ=acc)
            logger.info(f"MovJ({x:.2f},{y:.2f},{z:.2f},{rx_deg:.2f},{ry_deg:.2f},{rz_deg:.2f},tool={tool},speedJ={velocity},accJ={acc}): {r}")
            resp = parse_response(r)
            if not resp.ok:
                return resp.error_id
        except DobotApiError as e:
            logger.error(f"MovJ failed: {e}")
            return -1
        
        if wait:
            self._wait_motion_done()
        return 0

    def move_joint(self, joints: List[float], velocity: float = 10.0, acc: float = 20.0, dec: float = 20.0, tool_num: Optional[int] = None, wait: bool = True) -> int:
        if not self._connected or not self.move:
            return -2
        if len(joints) < 6:
            joints.extend([0.0]*(6-len(joints)))
            
        try:
            j = [val + 0.0 for val in joints]
            r = self.move.JointMovJ(j[0], j[1], j[2], j[3], j[4], j[5], speedJ=velocity, accJ=acc)
            logger.info(f"JointMovJ({[round(v,2) for v in j]},speed={velocity},acc={acc}): {r}")
            resp = parse_response(r)
            if not resp.ok:
                return resp.error_id
        except DobotApiError as e:
            logger.error(f"JointMovJ failed: {e}")
            return -1
        
        if wait:
            self._wait_motion_done()
        return 0

    def move_l(self, pose: PoseLike, velocity: float = 10.0, acc: float = 20.0, dec: float = 20.0, tool_num: Optional[int] = None, wait: bool = True) -> int:
        """单段直线运动 (MoveL)，velocity 为笛卡尔线速度 mm/s。"""
        if not self._connected or not self.move:
            return -2
        lst = _to_list(pose)
        x, y, z = lst[0], lst[1], lst[2]
        rx_deg, ry_deg, rz_deg = math.degrees(lst[3]), math.degrees(lst[4]), math.degrees(lst[5])
        tool = self.tool_num if tool_num is None else tool_num
        
        try:
            if self.dashboard:
                r = self.dashboard.TCPSpeed(velocity)
                logger.debug(f"TCPSpeed({velocity}): {r}")
                #r = self.dashboard.AccL(int(acc))
                #logger.debug(f"AccL({int(acc)}): {r}")
            # Fix -0.0 bug
            x, y, z = x + 0.0, y + 0.0, z + 0.0
            rx_deg, ry_deg, rz_deg = rx_deg + 0.0, ry_deg + 0.0, rz_deg + 0.0
            r = self.move.MovL(x, y, z, rx_deg, ry_deg, rz_deg, tool=tool, speedL=velocity, accL=acc)
            logger.info(f"MovL({x:.2f},{y:.2f},{z:.2f},{rx_deg:.2f},{ry_deg:.2f},{rz_deg:.2f},tool={tool},speedL={velocity},accL={acc}): {r}")
            resp = parse_response(r)
            if not resp.ok:
                return resp.error_id
        except DobotApiError as e:
            logger.error(f"MovL failed: {e}")
            return -1
        
        if wait:
            self._wait_motion_done()
            r = self.dashboard.TCPSpeedEnd()
            logger.debug(f"TCPSpeedEnd: {r}")
        return 0

    def move_j_queue(
        self, 
        poses: List[PoseLike], 
        velocity: float = 10.0, 
        acc: float = 20.0, 
        dec: float = 20.0,
        tool_num: Optional[int] = None,
        wait: bool = True,
        cp_ratio: int = 50
    ) -> int:
        if not self._connected or not self.move:
            return -2
            
        tool = self.tool_num if tool_num is None else tool_num
        try:
            if self.dashboard:
                r = self.dashboard.CP(cp_ratio)
                logger.debug(f"CP({cp_ratio}): {r}")
            for pose in poses:
                lst = _to_list(pose)
                x, y, z = lst[0] + 0.0, lst[1] + 0.0, lst[2] + 0.0
                rx_deg, ry_deg, rz_deg = math.degrees(lst[3]) + 0.0, math.degrees(lst[4]) + 0.0, math.degrees(lst[5]) + 0.0
                
                r = self.move.MovJ(x, y, z, rx_deg, ry_deg, rz_deg, tool=tool, speedJ=velocity, accJ=acc)
                # Parse response to ensure it was accepted
                resp = parse_response(r)
                if not resp.ok:
                    logger.error(f"MovJ queue rejected: {r}")
                    return resp.error_id
                
            logger.info(f"Sent {len(poses)} MovJ commands sequentially via move.MovJ (tool={tool})")
                
        except DobotApiError as e:
            logger.error(f"MovJ queue failed: {e}")
            return -1
            
        if wait:
            self._wait_motion_done()
            
        return 0

    def move_l_queue(
        self, 
        poses: List[PoseLike], 
        velocity: float = 10.0, 
        acc: float = 20.0, 
        dec: float = 20.0,
        tool_num: Optional[int] = None,
        wait: bool = True,
        cp_ratio: int = 50,
        speeds: Optional[List[float]] = None
    ) -> int:
        if not self._connected or not self.move:
            return -2
            
        tool = self.tool_num if tool_num is None else tool_num
        # speeds: 逐航点目标 TCP 线速度 (mm/s, 与 poses 等长), 仅在奇异降速使能时由上层传入。
        # 与现有"每个 segment 重新下发 TCPSpeed 而不 End"的用法同源: 队列内覆写当前速度不阻断 CP。
        # None / 长度不匹配 / 无 dashboard 时完全回退到整批单一 TCPSpeed(velocity) 行为 (零回归)。
        use_speeds = bool(speeds) and len(speeds) == len(poses) and self.dashboard is not None
        try:
            if self.dashboard:
                r = self.dashboard.CP(cp_ratio)
                logger.debug(f"CP({cp_ratio}): {r}")
                if not use_speeds:
                    r = self.dashboard.TCPSpeed(velocity)
                    logger.debug(f"TCPSpeed({velocity}): {r}")
            cur_v: Optional[int] = None
            for idx, pose in enumerate(poses):
                if use_speeds:
                    # 近奇异航点按可操作度比例压低线速度; 同一整数速度不重复下发
                    raw = speeds[idx] if (idx < len(speeds) and speeds[idx] and speeds[idx] > 0) else velocity
                    v_i = max(1, int(round(raw)))
                    if v_i != cur_v:
                        r = self.dashboard.TCPSpeed(v_i)
                        logger.debug(f"TCPSpeed({v_i}) [waypoint {idx}]: {r}")
                        cur_v = v_i
                lst = _to_list(pose)
                x, y, z = lst[0] + 0.0, lst[1] + 0.0, lst[2] + 0.0
                rx_deg, ry_deg, rz_deg = math.degrees(lst[3]) + 0.0, math.degrees(lst[4]) + 0.0, math.degrees(lst[5]) + 0.0
                
                r = self.move.MovL(x, y, z, rx_deg, ry_deg, rz_deg, tool=tool, accL=acc)
                resp = parse_response(r)
                if not resp.ok:
                    logger.error(f"MovL queue rejected: {r}")
                    return resp.error_id
                logger.info(f"MovL sent: {x,y,z,rx_deg,ry_deg,rz_deg}, tool={tool}, response {r}")
                
            logger.info(f"Sent {len(poses)} MovL commands sequentially via move.MovL (tool={tool}, per_waypoint_speed={use_speeds})")
                
        except DobotApiError as e:
            if self.dashboard:
                r = self.dashboard.TCPSpeedEnd()
                logger.debug(f"TCPSpeedEnd (on error): {r}")
            logger.error(f"MovL queue failed: {e}")
            return -1
            
        if wait:
            time.sleep(0.001)
            logger.info("waiting for dobot motion done ... ")
            if not self._wait_motion_done():
                logger.warning("dobot motion time out.")
            else:
                logger.info("dobot motion done.")
            r = self.dashboard.TCPSpeedEnd()
            logger.debug(f"TCPSpeedEnd: {r}")
            
        return 0

    def set_tool_number(self, tool_num: int) -> bool:
        if not self._connected or not self.dashboard:
            return False
        try:
            # 按协议解析 ErrorID，不能用 "0" in res 判断（-10000 里也含 "0"）
            if parse_response(self.dashboard.Tool(tool_num)).ok:
                self.tool_num = tool_num
                return True
        except Exception as e:
            logger.error(f"Error setting tool number: {e}")
        return False

    def set_global_speed(self, speed: int) -> bool:
        if not self._connected or not self.dashboard:
            return False
        try:
            r = self.dashboard.SpeedFactor(int(speed))
            logger.info(f"SpeedFactor({int(speed)}): {r}")
        except DobotApiError as e:
            logger.error(f"Error setting global speed: {e}")
            return False
        self._global_speed = int(speed)
        return True

    def get_global_speed(self) -> tuple[float, float, float, float]:
        if not self._connected:
            return 20.0, 20.0, 20.0, 20.0
        return self._cached_speed_l, self._cached_acc_l, self._cached_speed_j, self._cached_acc_j

    def go_home(self, wait: bool = True, velocity: Optional[float] = None, acc: Optional[float] = None, target_joints: Optional[List[float]] = None) -> int:
        vel = velocity if velocity is not None else 20.0
        acc_val = acc if acc is not None else 20.0
        # Force-discard any stale data that may have accumulated on the 30003 move port
        # after MoveJog. Even though MoveJog() stop was sent, the firmware may have pushed
        # extra data into the TCP stream. Discarding here prevents protocol desync on JointMovJ.
        if self.move:
            try:
                self.move._discard_pending()
                logger.info("go_home: Cleared move port (30003) receive buffer")
            except Exception as e:
                logger.warning(f"go_home: Failed to clear move port buffer: {e}")
        # Move to configured or fallback home position for Dobot
        joints = target_joints if target_joints is not None else [0.0, 0.0, -90.0, -90.0, -90.0, 0.0]
        return self.move_joint(joints, velocity=vel, acc=acc_val, wait=wait)

    def pause(self) -> bool:
        if not self._connected:
            return False
        try:
            resp = self.dashboard.pause()
            logger.info(f"Dobot pause response: {resp}")
            return True
        except DobotApiError as e:
            logger.error(f"Error pausing robot: {e}")
            return False

    def resume(self) -> bool:
        if not self._connected:
            return False
        try:
            resp = self.dashboard.Continue()
            logger.info(f"Dobot resume response: {resp}")
            return True
        except DobotApiError as e:
            logger.error(f"Error resuming robot: {e}")
            return False

    def estop(self) -> bool:
        if not self._connected:
            return False
        try:
            r = self.dashboard.EmergencyStop()
            logger.info(f"EmergencyStop: {r}")
            # 注: 急停后算法队列会被挂起，队列形式的 DO 指令不会再生效，
            # 关闭喷涂 DO 由 RobotService.estop() 统一使用立即指令完成。
            return True
        except Exception as e:
            logger.error(f"Error emergency stopping robot: {e}")
            return False

    def move_jog_joint(self, axis_id: str = "") -> bool:
        """
        关节点动 (J1+ ~ J6+, J1- ~ J6-) 或停止点动 ("")。
        """
        if not self._connected or not self.move:
            return False
        try:
            if not axis_id:
                self._do_move_jog_stop()
            else:
                r = self.move.MoveJog(axis_id)
                logger.info(f"MoveJog({axis_id}): {r}")
            return True
        except DobotApiError as e:
            logger.error(f"move_jog_joint failed: {e}")
            return False

    def move_jog_cartesian(self, axis_id: str = "", coord_type: int = 2, user: Optional[int] = None, tool_num: Optional[int] = None) -> bool:
        """
        笛卡尔坐标点动 (X+, Y+, Z+, Rx+, Ry+, Rz+ 等) 或停止点动 ("")。
        coord_type: 0 (用户/基坐标系), 2 (工具坐标系), 默认 2。
        tool_num: 工具坐标系编号，默认使用 self.tool_num。
        """
        if not self._connected or not self.move:
            return False
        tool = self.tool_num if tool_num is None else tool_num
        user = self.user if user is None else user
        try:
            if not axis_id:
                self._do_move_jog_stop()
            else:
                r = self.move.MoveJog(axis_id, coordType=coord_type, user=user, tool=tool)
                logger.info(f"MoveJog({axis_id}, coordType={coord_type}, user={user}, tool={tool}): {r}")
            return True
        except DobotApiError as e:
            logger.error(f"move_jog_cartesian failed: {e}")
            return False

    def _do_move_jog_stop(self):
        """
        统一的 move_jog_stop 逻辑。
        """
        r = self.move.MoveJog("")
        logger.info(f"MoveJog: {r}")
        time.sleep(0.3)
        if self.dashboard:
            try:
                self.dashboard.ClearError()
                r3 = self.dashboard.EnableRobot()
                logger.info(f"MoveJog stop: EnableRobot: {r3}")
            except Exception as e:
                logger.warning(f"MoveJog stop: EnableRobot failed: {e}")

    def clear_error(self) -> bool:
        """Dobot 报警清除与重新使能"""
        if not self._connected:
            return False
        try:
            r = self.dashboard.ClearError()
            logger.info(f"ClearError: {r}")
            # Dobot V3 protocol requires calling EnableRobot again to reopen the motion queue after an alarm is cleared.
            time.sleep(0.5)
            r2 = self.dashboard.EnableRobot()
            logger.info(f"EnableRobot after clear: {r2}")
            return True
        except Exception as e:
            logger.error(f"Dobot clear_error failed: {e}")
            return False

    def sync(self) -> bool:
        """
        同步
        """
        if not self._connected:
            return False
        try:
            r = self.move.Sync()
            logger.info(f"Sync: {r}")
            return True
        except DobotApiError as e:
            logger.error(f"Sync failed: {e}")
            return False

    def set_do(self, index: int, status: int, immediate: bool = False) -> bool:
        """
        设置机械臂数字输出端口 (DO) 状态。
        :param index: DO 端子编号 (1-16)
        :param status: 1 有信号(开)，0 无信号(关)
        :param immediate: True 为立即指令 (DOExecute, 忽略队列立刻执行),
                          False 为队列指令 (DO, 进入后台算法队列排队执行)
        :return: 是否设置成功
        """
        if not self._connected or not self.dashboard:
            logger.warning("Dobot not connected or dashboard unavailable, cannot set DO")
            return False
        # 不同控制器固件对两类指令的支持情况不一致，主用指令被拒时回退到另一条
        calls = (
            (("DOExecute", self.dashboard.DOExecute), ("DO", self.dashboard.DO))
            if immediate else
            (("DO", self.dashboard.DO), ("DOExecute", self.dashboard.DOExecute))
        )
        for name, send in calls:
            try:
                r = send(index, status)
                logger.info(f"{name}({index}, {status}): {r}")
                if parse_response(r).ok:
                    return True
                logger.warning(f"{name}({index}, {status}) 被拒, 回退尝试另一种 DO 指令")
            except Exception as e:
                logger.error(f"{name}({index}, {status}) 发送失败: {e}")
        return False


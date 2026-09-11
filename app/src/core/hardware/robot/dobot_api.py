"""Dobot 工业机器人控制柜 V3 TCP/IP 二次开发接口。

实现严格对照 ``app/docs/dobot_tcp_ip_v3_protocol.md``：

- 29999：Dashboard 指令（设置、获取、IO、Modbus）
- 30003：运动指令（均为队列指令）
- 30004 / 30005 / 30006：实时反馈端口，每包 1440 字节，小端存储

协议约定的消息格式为 ASCII 字符串：

    下发：消息名称(Param1,Param2,...,ParamN)
    应答：ErrorID,{value,...,valueN},消息名称(Param1,...,ParamN);

可选参数一律以 ``Key=Value`` 形式携带（如 ``SpeedJ=50``、``User=1``），
因此本模块的运动指令同时提供关键字参数（推荐）和 ``Key=Value`` 字符串两种写法。
"""

from __future__ import annotations

import datetime
import json
import logging
import os
import socket
import threading
import time
from typing import Any, Dict, Iterable, List, NamedTuple, Optional, Sequence, Tuple

import numpy as np

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# 端口与超时
# ---------------------------------------------------------------------------

DASHBOARD_PORT = 29999
MOVE_PORT = 30003
# 控制器 3.5.2 及以上支持 30004/30005/30006 三个实时反馈端口
FEEDBACK_PORTS = (30004, 30005, 30006)
SUPPORTED_PORTS = (DASHBOARD_PORT, MOVE_PORT) + FEEDBACK_PORTS

REPLY_TERMINATOR = b";"

DEFAULT_CONNECT_TIMEOUT = 3.0
DEFAULT_REPLY_TIMEOUT = 5.0
# 上电/使能类指令，协议说明上电到使能完成约需 10 秒
STARTUP_REPLY_TIMEOUT = 30.0
# Sync 会阻塞到队列最后一条指令执行完毕，耗时取决于轨迹长度
QUEUE_REPLY_TIMEOUT = 600.0

FEEDBACK_PACKAGE_SIZE = 1440
# 协议给出的内存结构测试标准值，用于校验数据包是否按包头对齐
FEEDBACK_TEST_VALUE = 0x0123456789ABCDEF
_FEEDBACK_TEST_VALUE_BYTES = FEEDBACK_TEST_VALUE.to_bytes(8, "little")
_FEEDBACK_TEST_VALUE_OFFSET = 48

alarmControllerFile = "files/alarm_controller.json"
alarmServoFile = "files/alarm_servo.json"

# tkinter.END 的实际值就是字符串 "end"。这里直接使用字面量，避免为了一个可选的
# GUI 日志框而让整个驱动强依赖 tkinter（无 X11 的工业机/容器里 import 会失败）。
_TK_END = "end"


# ---------------------------------------------------------------------------
# 异常
# ---------------------------------------------------------------------------


class DobotApiError(Exception):
    """本模块所有异常的基类。"""


class DobotConnectionError(DobotApiError):
    """连接建立失败、发送失败或对端关闭连接。"""


class DobotTimeoutError(DobotApiError):
    """在超时时间内没有收到完整应答。"""


class DobotParamError(DobotApiError, ValueError):
    """参数不符合协议要求。"""


class DobotProtocolError(DobotApiError):
    """应答内容不符合协议格式，无法解析。"""


class DobotCommandError(DobotApiError):
    """控制器返回了非 0 的 ErrorID。"""

    def __init__(self, response: "DobotResponse"):
        self.response = response
        super().__init__(
            f"指令 {response.command or '?'} 执行失败："
            f"ErrorID={response.error_id}（{describe_error_id(response.error_id)}）")


# ---------------------------------------------------------------------------
# 实时反馈数据结构（30004 / 30005 / 30006）
# ---------------------------------------------------------------------------

# 字节偏移严格对应协议“4 实时反馈信息”一节，总长 1440 字节。
# 协议中类型为 char 的状态/比例/位掩码字段按无符号语义使用（例如 CRRobotType 取值
# 可达 160，BrakeStatus 为位掩码），因此统一用 uint8；只有 HandType 保存手系的
# 1/-1、Config6 的 -1/-2 等负值，必须保持有符号。
MyType = np.dtype([
    ('message_size', np.uint16),                    # 0000~0001 消息字节总长度，固定 1440
    ('reserve1', np.uint16, (3,)),                  # 0002~0007 保留位
    ('digital_input_bits', np.uint64),              # 0008~0015 数字输入端子状态（按位）
    ('digital_output_bits', np.uint64),             # 0016~0023 数字输出端子状态（按位）
    ('robot_mode', np.uint64),                      # 0024~0031 机器人模式，含义同 RobotMode
    ('time_stamp', np.uint64),                      # 0032~0039 Unix 时间戳（ms）
    ('time_stamp_reserve_bit', np.uint64),          # 0040~0047 保留位
    ('test_value', np.uint64),                      # 0048~0055 内存结构测试标准值
    ('test_value_keep_bit', np.float64),            # 0056~0063 保留位
    ('speed_scaling', np.float64),                  # 0064~0071 速度比例
    ('linear_momentum_norm', np.float64),           # 0072~0079 保留位
    ('v_main', np.float64),                         # 0080~0087 控制板电压
    ('v_robot', np.float64),                        # 0088~0095 机器人电压
    ('i_robot', np.float64),                        # 0096~0103 机器人电流
    ('i_robot_keep_bit1', np.float64),              # 0104~0111 保留位
    ('i_robot_keep_bit2', np.float64),              # 0112~0119 保留位
    ('tool_accelerometer_values', np.float64, (3,)),  # 0120~0143 保留位
    ('elbow_position', np.float64, (3,)),           # 0144~0167 保留位
    ('elbow_velocity', np.float64, (3,)),           # 0168~0191 保留位
    ('q_target', np.float64, (6,)),                 # 0192~0239 目标关节位置
    ('qd_target', np.float64, (6,)),                # 0240~0287 目标关节速度
    ('qdd_target', np.float64, (6,)),               # 0288~0335 目标关节加速度
    ('i_target', np.float64, (6,)),                 # 0336~0383 目标关节电流
    ('m_target', np.float64, (6,)),                 # 0384~0431 目标关节扭矩
    ('q_actual', np.float64, (6,)),                 # 0432~0479 实际关节位置
    ('qd_actual', np.float64, (6,)),                # 0480~0527 实际关节速度
    ('i_actual', np.float64, (6,)),                 # 0528~0575 实际关节电流
    ('actual_TCP_force', np.float64, (6,)),         # 0576~0623 TCP 传感器力值
    ('tool_vector_actual', np.float64, (6,)),       # 0624~0671 TCP 笛卡尔实际坐标值
    ('TCP_speed_actual', np.float64, (6,)),         # 0672~0719 TCP 笛卡尔实际速度值 (m/s + rad/s)
    ('TCP_force', np.float64, (6,)),                # 0720~0767 TCP 力值（关节电流计算）
    ('Tool_vector_target', np.float64, (6,)),       # 0768~0815 TCP 笛卡尔目标坐标值
    ('TCP_speed_target', np.float64, (6,)),         # 0816~0863 TCP 笛卡尔目标速度值
    ('motor_temperatures', np.float64, (6,)),       # 0864~0911 关节温度
    ('joint_modes', np.float64, (6,)),              # 0912~0959 关节控制模式
    ('v_actual', np.float64, (6,)),                 # 0960~1007 关节电压
    ('hand_type', np.int8, (4,)),                   # 1008~1011 手系，取值含 -1/-2，需有符号
    ('user_index', np.uint8),                       # 1012 当前用户坐标系索引
    ('tool_index', np.uint8),                       # 1013 当前工具坐标系索引
    ('run_queued_cmd', np.uint8),                   # 1014 算法队列运行标志
    ('pause_cmd_flag', np.uint8),                   # 1015 算法队列暂停标志
    ('velocity_ratio', np.uint8),                   # 1016 关节速度比例
    ('acceleration_ratio', np.uint8),               # 1017 关节加速度比例
    ('jerk_ratio', np.uint8),                       # 1018 关节加加速度比例
    ('xyz_velocity_ratio', np.uint8),               # 1019 笛卡尔位置速度比例
    ('r_velocity_ratio', np.uint8),                 # 1020 笛卡尔姿态速度比例
    ('xyz_acceleration_ratio', np.uint8),           # 1021 笛卡尔位置加速度比例
    ('r_acceleration_ratio', np.uint8),             # 1022 笛卡尔姿态加速度比例
    ('xyz_jerk_ratio', np.uint8),                   # 1023 笛卡尔位置加加速度比例
    ('r_jerk_ratio', np.uint8),                     # 1024 笛卡尔姿态加加速度比例
    ('brake_status', np.uint8),                     # 1025 抱闸状态（按位）
    ('enable_status', np.uint8),                    # 1026 使能状态
    ('drag_status', np.uint8),                      # 1027 拖拽状态
    ('running_status', np.uint8),                   # 1028 运行状态
    ('error_status', np.uint8),                     # 1029 报警状态
    ('jog_status', np.uint8),                       # 1030 点动状态
    ('robot_type', np.uint8),                       # 1031 机器人型号，取值可达 160
    ('drag_button_signal', np.uint8),               # 1032 末端按钮拖拽信号
    ('enable_button_signal', np.uint8),             # 1033 末端按钮使能信号
    ('record_button_signal', np.uint8),             # 1034 末端按钮录制信号
    ('reappear_button_signal', np.uint8),           # 1035 末端按钮复现信号
    ('jaw_button_signal', np.uint8),                # 1036 末端按钮夹爪控制信号
    ('six_force_online', np.uint8),                 # 1037 六维力在线状态
    ('reserve2', np.uint8, (82,)),                  # 1038~1119 保留位
    ('m_actual', np.float64, (6,)),                 # 1120~1167 六个关节的实际扭矩
    ('load', np.float64),                           # 1168~1175 末端负载重量（kg）
    ('center_x', np.float64),                       # 1176~1183 负载 X 方向偏心距离（mm）
    ('center_y', np.float64),                       # 1184~1191 负载 Y 方向偏心距离（mm）
    ('center_z', np.float64),                       # 1192~1199 负载 Z 方向偏心距离（mm）
    ('user_frame', np.float64, (6,)),               # 1200~1247 用户坐标系坐标值
    ('tool_frame', np.float64, (6,)),               # 1248~1295 工具坐标系坐标值
    ('trace_index', np.float64),                    # 1296~1303 轨迹复现运行索引
    ('six_force_value', np.float64, (6,)),          # 1304~1351 六维力数据原始值
    ('target_quaternion', np.float64, (4,)),        # 1352~1383 目标四元数
    ('actual_quaternion', np.float64, (4,)),        # 1384~1415 实际四元数
    ('reserve3', np.uint8, (24,)),                  # 1416~1439 保留位
])

if MyType.itemsize != FEEDBACK_PACKAGE_SIZE:  # pragma: no cover - 结构体定义保护
    raise RuntimeError(
        f"反馈结构体长度应为 {FEEDBACK_PACKAGE_SIZE} 字节，当前为 {MyType.itemsize}")


# ---------------------------------------------------------------------------
# 告警描述文件
# ---------------------------------------------------------------------------

_alarm_cache: Optional[Tuple[Any, Any]] = None


def alarmAlarmJsonFile():
    """读取控制器与伺服的告警描述文件，结果会被缓存。"""
    global _alarm_cache
    if _alarm_cache is None:
        currentDirectory = os.path.dirname(__file__)
        with open(os.path.join(currentDirectory, alarmControllerFile), encoding='utf-8') as f:
            dataController = json.load(f)
        with open(os.path.join(currentDirectory, alarmServoFile), encoding='utf-8') as f:
            dataServo = json.load(f)
        _alarm_cache = (dataController, dataServo)
    return _alarm_cache


def _alarm_index(table: Iterable[Dict[str, Any]]) -> Dict[int, Dict[str, Any]]:
    return {int(item["id"]): item for item in table if "id" in item}


def _describe_alarm(alarm_id: int, table: Dict[int, Dict[str, Any]], lang: str) -> str:
    # 协议特别说明：碰撞检测为 -2，电子皮肤碰撞检测为 -3
    if alarm_id == -2:
        return "碰撞检测触发" if lang == "zh_CN" else "Collision detected"
    if alarm_id == -3:
        return "电子皮肤碰撞检测触发" if lang == "zh_CN" else "Safe skin collision detected"
    item = table.get(alarm_id)
    if item is None:
        return f"未知告警 id={alarm_id}"
    detail = item.get(lang) or item.get("en") or {}
    return detail.get("description", f"告警 id={alarm_id}")


# ---------------------------------------------------------------------------
# 通用错误码（协议“5 通用错误码”）
# ---------------------------------------------------------------------------

_COMMON_ERROR_CODES = {
    0: "Success (No error)",
    -1: "Command execution failed",
    -10000: "Command error (command does not exist)",
    -20000: "Parameter count error",
}


def describe_error_id(error_id: int) -> str:
    """Translate ErrorID into a human-readable English description."""
    if error_id in _COMMON_ERROR_CODES:
        return _COMMON_ERROR_CODES[error_id]
    if -30099 <= error_id <= -30001:
        return f"Parameter {abs(error_id) - 30000} type error"
    if -40099 <= error_id <= -40001:
        return f"Parameter {abs(error_id) - 40000} value range error"
    return f"Undefined error code {error_id}"


class DobotResponse(NamedTuple):
    """解析后的应答：``ErrorID,{value,...},消息名称(...);``"""

    error_id: int
    values: List[str]
    command: str
    raw: str

    @property
    def ok(self) -> bool:
        return self.error_id == 0

    def as_floats(self) -> List[float]:
        return [float(item) for item in self.values]


def parse_response(reply: str) -> DobotResponse:
    """按协议格式解析应答字符串。"""
    text = (reply or "").strip()
    if not text:
        raise DobotProtocolError("应答为空，无法解析（可能是超时或连接已断开）")
    text = text.rstrip(";").strip()
    head, _, rest = text.partition(",")
    try:
        error_id = int(head.strip())
    except ValueError as exc:
        raise DobotProtocolError(f"应答不符合协议格式：{reply!r}") from exc

    start = rest.find("{")
    end = rest.rfind("}")
    if start == -1 or end < start:
        raise DobotProtocolError(f"应答缺少返回值大括号：{reply!r}")
    body = rest[start + 1:end].strip()
    values = [item.strip() for item in body.split(",")] if body else []
    command = rest[end + 1:].lstrip(",").strip()
    return DobotResponse(error_id, values, command, reply)


def parse_error_id(reply: str) -> Tuple[List[int], List[List[int]]]:
    """解析 GetErrorID 的应答。

    返回 ``(控制器与算法报警 id 列表, 六个伺服的报警 id 列表)``。
    """
    text = (reply or "").strip().rstrip(";")
    start = text.find("{")
    end = text.rfind("}")
    if start == -1 or end < start:
        raise DobotProtocolError(f"GetErrorID 应答不符合协议格式：{reply!r}")
    try:
        groups = json.loads(text[start + 1:end])
    except json.JSONDecodeError as exc:
        raise DobotProtocolError(f"GetErrorID 返回值无法解析：{reply!r}") from exc
    if not isinstance(groups, list) or not groups:
        raise DobotProtocolError(f"GetErrorID 返回值结构异常：{reply!r}")
    controller = [int(i) for i in groups[0]]
    servos = [[int(i) for i in group] for group in groups[1:]]
    return controller, servos


ROBOT_MODES: Dict[int, str] = {
    1: "INIT (ROBOT_MODE_INIT)",
    2: "BRAKE_OPEN (ROBOT_MODE_BRAKE_OPEN)",
    3: "POWEROFF (ROBOT_MODE_POWEROFF)",
    4: "DISABLED (ROBOT_MODE_DISABLED)",
    5: "ENABLE / IDLE (ROBOT_MODE_ENABLE)",
    6: "DRAG (ROBOT_MODE_DRAG)",
    7: "RUNNING (ROBOT_MODE_RUNNING)",
    8: "RECORDING (ROBOT_MODE_RECORDING)",
    9: "ERROR (ROBOT_MODE_ERROR)",
    10: "PAUSE (ROBOT_MODE_PAUSE)",
    11: "JOG (ROBOT_MODE_JOG)",
}


def parse_robot_mode(reply: Union[str, DobotResponse]) -> Tuple[int, str]:
    """Parse RobotMode response into ``(mode_id, mode_description)``."""
    if isinstance(reply, str):
        resp = parse_response(reply)
    else:
        resp = reply
    if not resp.ok or not resp.values:
        return -1, f"Failed to get mode (ErrorID={resp.error_id}: {describe_error_id(resp.error_id)})"
    try:
        mode = int(resp.values[0])
        desc = ROBOT_MODES.get(mode, f"UNKNOWN_MODE({mode})")
        return mode, desc
    except (ValueError, IndexError):
        return -1, f"Invalid mode value: {resp.values}"


def describe_response(reply: Union[str, DobotResponse]) -> str:
    """Format any response into a human-readable English string."""
    if isinstance(reply, str):
        resp = parse_response(reply)
    else:
        resp = reply

    if not resp.ok:
        return f"Failed (ErrorID={resp.error_id}: {describe_error_id(resp.error_id)})"

    cmd_name = resp.command.split("(")[0].strip() if resp.command else ""

    if cmd_name == "RobotMode" and resp.values:
        mode, desc = parse_robot_mode(resp)
        return f"RobotMode {mode}: {desc}"

    if cmd_name == "GetErrorID":
        try:
            ctrl, servos = parse_error_id(resp.raw)
            if not ctrl and not any(servos):
                return "No alarm (OK)"
            return f"Controller alarms: {ctrl}, Servo alarms: {servos}"
        except Exception:
            pass

    if resp.values:
        return f"Success: [{', '.join(resp.values)}]"
    return "Success (OK)"


# ---------------------------------------------------------------------------
# 参数格式化工具
# ---------------------------------------------------------------------------


def _as_int(value: Any, name: str) -> int:
    """转换为协议要求的整数，允许传入 50.0 / "50" 这类等价写法。"""
    if isinstance(value, bool):
        raise DobotParamError(f"参数 {name} 不能是布尔值，收到 {value!r}")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise DobotParamError(f"参数 {name} 需要整数，收到 {value!r}") from exc
    result = int(round(number))
    if abs(number - result) > 1e-6:
        logger.debug("参数 %s 的值 %s 已取整为 %d", name, value, result)
    return result


def _as_float(value: Any, name: str) -> float:
    """转换为协议要求的浮点数。"""
    if isinstance(value, bool):
        raise DobotParamError(f"参数 {name} 不能是布尔值，收到 {value!r}")
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise DobotParamError(f"参数 {name} 需要数值，收到 {value!r}") from exc


def _as_name(value: Any, name: str) -> str:
    """字符串型参数（工程名、轨迹文件名、托盘名等）。"""
    text = str(value).strip()
    if not text:
        raise DobotParamError(f"参数 {name} 不能为空")
    if any(ch in text for ch in "(),"):
        raise DobotParamError(f"参数 {name} 不能包含括号或逗号：{value!r}")
    return text


def _floats(values: Sequence[Any], name: str) -> str:
    return ",".join(f"{_as_float(v, name):f}" for v in values)


def _number_text(value: Any, name: str) -> str:
    """表参数里的单个数值，整数不带小数点，浮点保留原始精度。"""
    if isinstance(value, int) and not isinstance(value, bool):
        return str(value)
    number = _as_float(value, name)
    return str(int(number)) if number.is_integer() else repr(number)


def _table(value: Any, name: str) -> str:
    """格式化 ``{x,y,z,rx,ry,rz}`` 形式的表参数。

    允许传入序列，也允许传入调用者自己拼好的字符串（可带或不带大括号）。
    """
    if isinstance(value, str):
        text = value.strip().replace(" ", "")
        if not text:
            raise DobotParamError(f"参数 {name} 不能为空")
        return text if text.startswith("{") else "{" + text + "}"
    try:
        items = list(value)
    except TypeError as exc:
        raise DobotParamError(f"参数 {name} 需要序列或字符串，收到 {value!r}") from exc
    if not items:
        raise DobotParamError(f"参数 {name} 不能为空")
    return "{" + ",".join(_number_text(item, name) for item in items) + "}"


def _int_group(values: Sequence[Any], name: str, size: Optional[int] = None) -> str:
    items = list(values)
    if size is not None and len(items) != size:
        raise DobotParamError(f"参数 {name} 需要 {size} 个整数，收到 {len(items)} 个")
    return "{" + ",".join(str(_as_int(item, name)) for item in items) + "}"


def _raw_optional(item: Any) -> str:
    text = str(item).strip()
    if "=" not in text:
        raise DobotParamError(
            f"可选参数必须写成 Key=Value 形式（例如 \"SpeedJ=50\"），收到 {item!r}。"
            "推荐直接使用本方法的关键字参数，如 speedJ=50。")
    return text.replace(" ", "")


def _optional_parts(keys: Sequence[str], dynParams: Sequence[Any],
                    explicit: Dict[str, Any],
                    allow_positional: bool = False) -> List[str]:
    """把可选参数统一整理成协议要求的 ``Key=Value`` 列表。

    keys             按协议顺序列出该指令支持的可选参数名，如 ``("SpeedJ", "AccJ", "User")``
    dynParams        位置形式传入的可选参数，始终接受 ``"SpeedJ=50"`` 这类字符串
    explicit         关键字形式传入的可选参数，优先级高于 dynParams
    allow_positional 是否允许按 keys 的顺序传裸数值。仅 RelMov*Tool / RelMov*User
                     这几个历史上就按顺序取值的指令开启，其余指令若传裸数值会拼出
                     位置参数并被控制器判为 -20000，因此直接拒绝并给出提示。
    """
    positional: List[Any] = []
    raw: List[str] = []
    for item in dynParams:
        if isinstance(item, str):
            raw.append(_raw_optional(item))
        elif isinstance(item, (tuple, list)):
            positional.extend(item)
        else:
            positional.append(item)

    if positional and not allow_positional:
        raise DobotParamError(
            f"可选参数必须写成 Key=Value 形式（例如 \"{keys[0]}=50\"）或使用关键字参数，"
            f"收到裸数值 {positional!r}。协议规定可选参数只能按 Key=Value 携带，"
            "直接拼成位置参数会被控制器判为 -20000（参数数量错误）。")

    if len(positional) > len(keys):
        raise DobotParamError(
            f"可选参数最多 {len(keys)} 个（按顺序为 {'、'.join(keys)}），"
            f"收到 {len(positional)} 个")

    values: Dict[str, Any] = dict(zip(keys, positional))
    values.update({k: v for k, v in explicit.items() if v is not None})
    parts = [f"{key}={_as_int(values[key], key)}"
             for key in keys if values.get(key) is not None]
    return parts + raw


def _finish(string: str, parts: Iterable[str]) -> str:
    """拼上可选参数并补右括号。"""
    return string + "".join("," + part for part in parts) + ")"


# ---------------------------------------------------------------------------
# 基础连接类
# ---------------------------------------------------------------------------


class DobotApi:
    """29999 / 30003 / 30004~30006 端口的公共连接与收发实现。"""

    def __init__(self, ip: str, port: int, *args, verbose: bool = False,
                 connect_timeout: float = DEFAULT_CONNECT_TIMEOUT,
                 reply_timeout: float = DEFAULT_REPLY_TIMEOUT,
                 auto_reconnect: bool = True):
        if port not in SUPPORTED_PORTS:
            raise DobotParamError(
                f"端口 {port} 不是协议支持的端口。可用端口："
                f"{DASHBOARD_PORT}（Dashboard 指令）、{MOVE_PORT}（运动指令）、"
                f"{'/'.join(str(p) for p in FEEDBACK_PORTS)}（实时反馈，控制器 3.5.2 及以上）")

        self.ip = ip
        self.port = port
        self.socket_dobot: Optional[socket.socket] = None
        self.verbose = verbose  # 为 True 时把 socket 通信日志同时打印到控制台
        self.connect_timeout = connect_timeout
        self.reply_timeout = reply_timeout
        self.auto_reconnect = auto_reconnect
        self._is_closed = False
        # 兼容旧用法：位置参数可传入一个 tkinter Text 控件用于显示日志
        self.text_log = args[0] if args else None
        self._globalLock = threading.Lock()
        self._recv_buffer = bytearray()

        self._connect()

    def _connect(self) -> None:
        """建立底层 socket 连接。"""
        self._close_socket()
        sock: Optional[socket.socket] = None
        try:
            logger.info("Connecting to Dobot at %s:%d (timeout=%.1fs)...", self.ip, self.port, self.connect_timeout)
            sock = socket.create_connection((self.ip, self.port), timeout=self.connect_timeout)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            if hasattr(socket, "TCP_KEEPIDLE"):
                try:
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 5)
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 2)
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
                except OSError:
                    pass
            sock.settimeout(self.reply_timeout)
            self.socket_dobot = sock
            self._recv_buffer.clear()
            logger.info("Successfully connected to Dobot at %s:%d", self.ip, self.port)
        except OSError as exc:
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
            self.socket_dobot = None
            raise DobotConnectionError(f"连接 {self.ip}:{self.port} 失败：{exc}") from exc

    def reconnect(self, max_retries: int = 3, retry_interval: float = 1.0) -> bool:
        """尝试重新连接。如果是主动调用 close() 则不执行重连。"""
        if self._is_closed:
            logger.info("Dobot %s:%d was actively closed; skipping auto-reconnect.", self.ip, self.port)
            return False
        logger.warning("Connection lost to Dobot %s:%d. Starting auto-reconnect (up to %d attempts)...",
                       self.ip, self.port, max_retries)
        for attempt in range(1, max_retries + 1):
            try:
                self._connect()
                logger.info("Successfully reconnected to Dobot %s:%d on attempt %d/%d.",
                            self.ip, self.port, attempt, max_retries)
                return True
            except DobotConnectionError as exc:
                logger.warning("Auto-reconnect attempt %d/%d to %s:%d failed: %s",
                               attempt, max_retries, self.ip, self.port, exc)
                if attempt < max_retries:
                    time.sleep(retry_interval)
        logger.error("Failed to auto-reconnect to Dobot %s:%d after %d attempts.",
                     self.ip, self.port, max_retries)
        return False

    # -- 日志 ---------------------------------------------------------------

    def log(self, text: str) -> None:
        logger.debug("%s", text)
        if self.text_log is not None:
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S ")
            self.text_log.insert(_TK_END, timestamp + text + "\n")
        elif self.verbose:
            print(text)

    # -- 收发 ---------------------------------------------------------------

    def _require_socket(self) -> socket.socket:
        if self.socket_dobot is None:
            if not self._is_closed and self.auto_reconnect:
                logger.warning("Socket for %s:%d is not connected, attempting auto-reconnect...", self.ip, self.port)
                if self.reconnect():
                    return self.socket_dobot
            raise DobotConnectionError(f"{self.ip}:{self.port} 的连接已关闭")
        return self.socket_dobot

    def send_data(self, string: str) -> None:
        """下发一条指令。发送失败时若启用了 auto_reconnect 会尝试自动重连并重试一次。"""
        sock = self._require_socket()
        self.log(f"Send to {self.ip}:{self.port}: {string}")
        try:
            sock.sendall(string.encode("utf-8"))
        except OSError as exc:
            logger.warning("Failed to send data to %s:%d (%s). Socket may be broken.", self.ip, self.port, exc)
            self._close_socket()
            if not self._is_closed and self.auto_reconnect:
                if self.reconnect():
                    try:
                        self.log(f"Resending after reconnect to {self.ip}:{self.port}: {string}")
                        self.socket_dobot.sendall(string.encode("utf-8"))
                        return
                    except OSError as retry_exc:
                        raise DobotConnectionError(
                            f"向 {self.ip}:{self.port} 重发指令失败：{retry_exc}") from retry_exc
            raise DobotConnectionError(
                f"向 {self.ip}:{self.port} 下发指令失败：{exc}") from exc

    def wait_reply(self, timeout: Optional[float] = None) -> str:
        """读取一条完整应答。

        协议规定应答以英文分号结尾，因此这里按分号切分而不是固定读一次 1024 字节，
        既能拼接被分片的半包，也能把同时到达的多条应答留在缓冲区里逐条取用。
        """
        sock = self._require_socket()
        limit = self.reply_timeout if timeout is None else timeout
        deadline = time.monotonic() + limit
        try:
            while REPLY_TERMINATOR not in self._recv_buffer:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    logger.warning("Timeout waiting for reply from %s:%d (%.1fs). Forcibly closing socket to prevent protocol desync.", self.ip, self.port, limit)
                    self._close_socket()
                    raise DobotTimeoutError(
                        f"等待 {self.ip}:{self.port} 应答超时（{limit}s），"
                        f"已收到 {bytes(self._recv_buffer)!r}")
                sock.settimeout(remaining)
                try:
                    chunk = sock.recv(1024)
                except socket.timeout as exc:
                    logger.warning("Socket timeout waiting for reply from %s:%d (%.1fs). Forcibly closing socket to prevent protocol desync.", self.ip, self.port, limit)
                    self._close_socket()
                    raise DobotTimeoutError(
                        f"等待 {self.ip}:{self.port} 应答超时（{limit}s）") from exc
                except OSError as exc:
                    logger.warning("Error receiving from %s:%d (%s). Connection dropped.", self.ip, self.port, exc)
                    self._close_socket()
                    if not self._is_closed and self.auto_reconnect:
                        self.reconnect()
                    raise DobotConnectionError(
                        f"从 {self.ip}:{self.port} 接收数据失败：{exc}") from exc
                if not chunk:
                    logger.warning("Connection to %s:%d closed by peer (EOF).", self.ip, self.port)
                    self._close_socket()
                    if not self._is_closed and self.auto_reconnect:
                        self.reconnect()
                    raise DobotConnectionError(f"{self.ip}:{self.port} 连接已被对端关闭")
                self._recv_buffer.extend(chunk)
        finally:
            if self.socket_dobot is not None:
                try:
                    self.socket_dobot.settimeout(self.reply_timeout)
                except OSError:
                    pass

        end = self._recv_buffer.index(REPLY_TERMINATOR) + 1
        reply = bytes(self._recv_buffer[:end]).decode("utf-8", errors="replace")
        del self._recv_buffer[:end]
        self.log(f"Receive from {self.ip}:{self.port}: {reply}")
        return reply

    def _discard_pending(self) -> None:
        """下发新指令前清空残留数据。

        协议是严格的一问一答，正常情况下发送前缓冲区应为空。若上一条指令超时，
        它迟到的应答会残留在缓冲区里，导致后续所有指令与应答错位，这里主动丢弃。
        """
        if self.socket_dobot is None:
            return
        sock = self.socket_dobot
        discarded = bytearray(self._recv_buffer)
        self._recv_buffer.clear()
        try:
            sock.setblocking(False)
            while True:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                discarded.extend(chunk)
        except (BlockingIOError, InterruptedError):
            pass
        except OSError:
            pass
        finally:
            if self.socket_dobot is not None:
                try:
                    self.socket_dobot.settimeout(self.reply_timeout)
                except OSError:
                    pass
        if discarded:
            self.log(f"Discard stale data from {self.ip}:{self.port}: {bytes(discarded)!r}")

    def sendRecvMsg(self, string: str, timeout: Optional[float] = None) -> str:
        """下发指令并返回原始应答字符串（一问一答，全程持锁）。连接异常断开时自动重连并重试一次。"""
        with self._globalLock:
            for attempt in range(2):
                try:
                    self._discard_pending()
                    self.send_data(string)
                    return self.wait_reply(timeout)
                except DobotConnectionError as err:
                    if attempt == 0 and not self._is_closed and self.auto_reconnect:
                        logger.warning(
                            "sendRecvMsg to %s:%d failed with connection error (%s); attempting auto-reconnect and retry...",
                            self.ip, self.port, err
                        )
                        if self.reconnect():
                            time.sleep(0.1)
                            continue
                    raise

    def sendRecvChecked(self, string: str, timeout: Optional[float] = None) -> DobotResponse:
        """下发指令并校验 ErrorID，非 0 时抛出 DobotCommandError。"""
        response = parse_response(self.sendRecvMsg(string, timeout))
        if not response.ok:
            raise DobotCommandError(response)
        return response

    def sendCmd(self, string: str) -> None:
        """下发协议中明确“返回：无”的指令（ServoJ / ServoP），不等待应答。"""
        with self._globalLock:
            self._discard_pending()
            self.send_data(string)

    def sendWithoutReply(self, string: str) -> None:
        """下发协议中明确“返回：无”的指令，不等待应答。"""
        with self._globalLock:
            self._discard_pending()
            self.send_data(string)

    # -- 生命周期 -----------------------------------------------------------

    def _close_socket(self) -> None:
        """关闭当前的底层 socket（不标记 _is_closed）。"""
        sock, self.socket_dobot = self.socket_dobot, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def close(self) -> None:
        """主动关闭连接。标记 _is_closed = True，不会再自动重连。"""
        self._is_closed = True
        self._close_socket()
        logger.info("Closed connection to Dobot %s:%d (active close)", self.ip, self.port)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def __del__(self):
        try:
            self.close()
        except Exception:  # 解释器退出阶段属性可能已被回收
            pass


# ---------------------------------------------------------------------------
# Dashboard 指令（29999 端口）
# ---------------------------------------------------------------------------


class DobotApiDashboard(DobotApi):
    """29999 端口：设置、获取、IO 与 Modbus 相关指令。"""

    # ================== 2.1 控制相关指令 ==================

    def PowerOn(self):
        """机器人上电。上电到完成约需 10 秒，之后才能使能。"""
        return self.sendRecvMsg("PowerOn()", timeout=STARTUP_REPLY_TIMEOUT)

    def EnableRobot(self, load=None, centerX=None, centerY=None, centerZ=None):
        """使能机器人。

        协议规定可携带的参数数量只能是 0、1 或 4：

        - 不传参数：使能时不设置负载重量与偏心参数
        - 只传 load：设置负载重量（kg）
        - 四个参数全传：同时设置负载重量与 X/Y/Z 偏心距离（-500~500mm）

        注意 load=0 也是合法取值，因此这里用 None 而不是 0 判断参数是否携带。
        """
        centers = (centerX, centerY, centerZ)
        if load is None:
            if any(c is not None for c in centers):
                raise DobotParamError("设置偏心参数时必须同时给出 load")
            string = "EnableRobot()"
        elif all(c is None for c in centers):
            string = "EnableRobot({:f})".format(_as_float(load, "load"))
        elif all(c is not None for c in centers):
            string = "EnableRobot({:f},{:f},{:f},{:f})".format(
                _as_float(load, "load"),
                _as_float(centerX, "centerX"),
                _as_float(centerY, "centerY"),
                _as_float(centerZ, "centerZ"))
        else:
            raise DobotParamError(
                "协议只允许携带 0、1 或 4 个参数，centerX/centerY/centerZ 必须同时给出")
        return self.sendRecvMsg(string, timeout=STARTUP_REPLY_TIMEOUT)

    def DisableRobot(self):
        """下使能机器人。"""
        return self.sendRecvMsg("DisableRobot()", timeout=STARTUP_REPLY_TIMEOUT)

    def ClearError(self):
        """清除机器人报警。清除后需重新调用 EnableRobot 才能下发运动指令。"""
        return self.sendRecvMsg("ClearError()")

    def ResetRobot(self):
        """停止机器人，清空已规划的指令队列。"""
        return self.sendRecvMsg("ResetRobot()")

    def RunScript(self, project_name, quoted: bool = False):
        """运行指定工程。

        project_name 工程文件名称。图形编程工程需带 "blockly_" 前缀。
        quoted       是否给工程名加双引号。协议的 RunScript 示例写作
                     ``RunScript("demo")``，但同一份协议里其它字符串参数
                     （ModbusCreate 的 ip、StartTrace 的 traceName 等）示例
                     都不带引号，故默认不加，需要时显式置 True。

        注意：该指令会把全局速度、用户/工具坐标系、AccJ/AccL/SpeedJ/SpeedL/CP
        重置为控制软件里的设置值。
        """
        name = _as_name(project_name, "project_name")
        payload = f'"{name}"' if quoted else name
        return self.sendRecvMsg(f"RunScript({payload})", timeout=STARTUP_REPLY_TIMEOUT)

    def StopScript(self):
        """停止正在运行的工程。"""
        return self.sendRecvMsg("StopScript()")

    def PauseScript(self):
        """暂停正在运行的工程。"""
        return self.sendRecvMsg("PauseScript()")

    def ContinueScript(self):
        """继续已暂停的工程。"""
        return self.sendRecvMsg("ContinueScript()")

    def Pause(self):
        """暂停非工程下发的运动指令，不清空运动队列。"""
        return self.sendRecvMsg("Pause()")

    # 协议不区分大小写，保留小写别名以兼容既有调用
    pause = Pause

    def Continue(self):
        """与 Pause 对应，继续运行被暂停的运动指令。"""
        return self.sendRecvMsg("Continue()")

    def EmergencyStop(self):
        """紧急停止机器人。急停后会下电并报警，需清除报警后重新上电使能。"""
        return self.sendRecvMsg("EmergencyStop()")

    def BrakeControl(self, axisID, value):
        """控制指定关节的抱闸（仅下使能状态下有效，控制器 3.5.2 及以上）。

        axisID 关节轴序号，1 表示 J1 轴，依此类推
        value  0 表示抱闸锁死，1 表示松开抱闸
        """
        string = "BrakeControl({:d},{:d})".format(
            _as_int(axisID, "axisID"), _as_int(value, "value"))
        return self.sendRecvMsg(string)

    def StartDrag(self):
        """机器人进入拖拽模式（控制器 3.5.2 及以上；报错状态下无法进入）。"""
        return self.sendRecvMsg("StartDrag()")

    def StopDrag(self):
        """机器人退出拖拽模式（控制器 3.5.2 及以上）。"""
        return self.sendRecvMsg("StopDrag()")

    def SetCollideDrag(self, status):
        """强制进入或退出拖拽模式，报错状态下同样有效（控制器 3.5.2 及以上）。

        status 0 表示强制退出，1 表示强制进入
        """
        return self.sendRecvMsg("SetCollideDrag({:d})".format(_as_int(status, "status")))

    def SetSafeSkin(self, status):
        """开启或关闭电子皮肤功能（队列指令，需先安装并开启安全皮肤插件）。

        status 0 表示关闭，1 表示开启
        """
        return self.sendRecvMsg("SetSafeSkin({:d})".format(_as_int(status, "status")))

    def Wait(self, time_ms):
        """指令队列延时一段时间（控制器 3.5.5 及以上）。

        time_ms 延时毫秒数，取值范围 (0, 3600*1000)
        """
        return self.sendRecvMsg("wait({:d})".format(_as_int(time_ms, "time")))

    wait = Wait

    # ================== 2.2 设置相关指令 ==================

    def SpeedFactor(self, ratio):
        """设置全局速度比例，取值范围 1~100。

        注意：之后若调用 EnableRobot 或 RunScript，全局速度会被重置。
        """
        return self.sendRecvMsg("SpeedFactor({:d})".format(_as_int(ratio, "ratio")))

    def User(self, index):
        """设置全局用户坐标系，index 为已标定的用户坐标系索引，取值范围 0~9。"""
        return self.sendRecvMsg("User({:d})".format(_as_int(index, "index")))

    def Tool(self, index):
        """设置全局工具坐标系，index 为已标定的工具坐标系索引，取值范围 0~9。"""
        return self.sendRecvMsg("Tool({:d})".format(_as_int(index, "index")))

    def PayLoad(self, weight, inertia):
        """设置机器人末端负载（队列指令）。

        weight  负载重量，单位 kg，不能超过机型的负载范围
        inertia 负载惯量，单位 kgm²
        """
        string = "PayLoad({:f},{:f})".format(
            _as_float(weight, "weight"), _as_float(inertia, "inertia"))
        return self.sendRecvMsg(string)

    # 协议说明：LoadSet 与 PayLoad 等效
    LoadSet = PayLoad

    def LoadSwitch(self, status):
        """开关负载设置，开启后可提高碰撞检测灵敏度。

        status 0 表示关闭，1 表示开启
        """
        return self.sendRecvMsg("LoadSwitch({:d})".format(_as_int(status, "status")))

    def AccJ(self, R):
        """设置关节运动方式的加速度比例，取值范围 1~100（默认 50）。"""
        return self.sendRecvMsg("AccJ({:d})".format(_as_int(R, "R")))

    def AccL(self, R):
        """设置直线和弧线运动方式的加速度比例，取值范围 1~100（默认 50）。"""
        return self.sendRecvMsg("AccL({:d})".format(_as_int(R, "R")))

    def SpeedJ(self, R):
        """设置关节运动方式的速度比例，取值范围 1~100（默认 50）。"""
        return self.sendRecvMsg("SpeedJ({:d})".format(_as_int(R, "R")))

    def SpeedL(self, R):
        """设置直线和弧线运动方式的速度比例，取值范围 1~100（默认 50）。"""
        return self.sendRecvMsg("SpeedL({:d})".format(_as_int(R, "R")))

    def CP(self, R):
        """设置平滑过渡比例，取值范围 0~100（默认 50）。"""
        return self.sendRecvMsg("CP({:d})".format(_as_int(R, "R")))

    def SetArmOrientation(self, LorR, UorD, ForN, Config6):
        """设置运动目标点的手系（四个参数均为必选）。

        LorR    大臂朝向，1 向前（J2 为正），-1 向后
        UorD    肘关节朝向，1 向上（J3 为正），-1 向下
        ForN    腕关节是否翻转，1 不翻转（J5 为正），-1 翻转
        Config6 J6 轴角度范围，1 表示 [0,90]，2 表示 [90,180]，
                -1 表示 [0,-90]，-2 表示 [-90,-180]，依此类推
        """
        string = "SetArmOrientation({:d},{:d},{:d},{:d})".format(
            _as_int(LorR, "LorR"), _as_int(UorD, "UorD"),
            _as_int(ForN, "ForN"), _as_int(Config6, "Config6"))
        return self.sendRecvMsg(string)

    def SetCollisionLevel(self, level):
        """设置碰撞检测等级，0 表示关闭，1~5 数字越大灵敏度越高。"""
        return self.sendRecvMsg("SetCollisionLevel({:d})".format(_as_int(level, "level")))

    def TCPSpeed(self, vt):
        """设置绝对速度，单位 mm/s，取值范围 [0,100000)（控制器 3.5.5 及以上）。

        设置后 SpeedL 不再生效，需用 TCPSpeedEnd 关闭。
        """
        return self.sendRecvMsg("TCPSpeed({:d})".format(_as_int(vt, "vt")))

    def TCPSpeedEnd(self):
        """关闭绝对速度设置，与 TCPSpeed 配合使用（控制器 3.5.5 及以上）。"""
        return self.sendRecvMsg("TCPSpeedEnd()")

    def SetUser(self, index, table):
        """修改指定的用户坐标系（控制器 3.5.7 及以上）。

        index 用户坐标系索引，取值范围 [0,9]
        table 坐标系值，格式 {x,y,z,rx,ry,rz}，可传序列或字符串
        """
        string = "SetUser({:d},{:s})".format(
            _as_int(index, "index"), _table(table, "table"))
        return self.sendRecvMsg(string)

    def CalcUser(self, index, matrix_direction, table):
        """计算用户坐标系（控制器 3.5.7 及以上）。

        matrix_direction 1 表示左乘（沿基坐标系偏转），0 表示右乘（沿自身偏转）
        """
        string = "CalcUser({:d},{:d},{:s})".format(
            _as_int(index, "index"),
            _as_int(matrix_direction, "matrix_direction"),
            _table(table, "table"))
        return self.sendRecvMsg(string)

    def SetTool(self, index, table):
        """修改指定的工具坐标系（控制器 3.5.7 及以上）。"""
        string = "SetTool({:d},{:s})".format(
            _as_int(index, "index"), _table(table, "table"))
        return self.sendRecvMsg(string)

    def CalcTool(self, index, matrix_direction, table):
        """计算工具坐标系（控制器 3.5.7 及以上）。

        matrix_direction 1 表示左乘（沿法兰坐标系偏转），0 表示右乘（沿自身偏转）
        """
        string = "CalcTool({:d},{:d},{:s})".format(
            _as_int(index, "index"),
            _as_int(matrix_direction, "matrix_direction"),
            _table(table, "table"))
        return self.sendRecvMsg(string)

    # ================== 2.3 计算和获取相关指令 ==================

    def RobotMode(self):
        """获取机器人当前状态。

        返回值含义：1 初始化，2 有关节抱闸松开，3 本体未上电，4 未使能，
        5 使能且空闲，6 拖拽模式，7 运行中，8 轨迹录制，9 有未清除的报警，
        10 暂停，11 点动中。
        """
        return self.sendRecvMsg("RobotMode()")

    def GetRobotMode(self) -> Tuple[int, str]:
        """获取机器人当前状态，并返回解析后的 ``(模式代号, 模式中文描述)``。"""
        return parse_robot_mode(self.RobotMode())

    def HandleTrajPoints(self, traceName=None):
        """预处理轨迹文件（控制器 3.5.2 及以上）。

        traceName 轨迹文件名（含后缀）。不传参数表示查询上一次预处理的结果：
                  -3 文件内容错误，-2 文件不存在，-1 预处理未完成，
                  0 预处理完成，大于 0 表示对应序号的点位有问题。
        """
        if traceName is None:
            string = "HandleTrajPoints()"
        else:
            string = "HandleTrajPoints({:s})".format(_as_name(traceName, "traceName"))
        return self.sendRecvMsg(string, timeout=STARTUP_REPLY_TIMEOUT)

    def GetTraceStartPose(self, traceName):
        """获取轨迹拟合中首个点位（笛卡尔坐标点，控制器 3.5.2 及以上）。"""
        return self.sendRecvMsg(
            "GetTraceStartPose({:s})".format(_as_name(traceName, "traceName")))

    def GetPathStartPose(self, traceName):
        """获取轨迹复现中首个点位（关节坐标点，控制器 3.5.2 及以上）。"""
        return self.sendRecvMsg(
            "GetPathStartPose({:s})".format(_as_name(traceName, "traceName")))

    def PositiveSolution(self, J1, J2, J3, J4, J5, J6, User, Tool):
        """正解运算：给定各关节角度，计算末端在指定坐标系下的笛卡尔坐标。"""
        string = "PositiveSolution({:s},{:d},{:d})".format(
            _floats((J1, J2, J3, J4, J5, J6), "J"),
            _as_int(User, "User"), _as_int(Tool, "Tool"))
        return self.sendRecvMsg(string)

    def InverseSolution(self, X, Y, Z, Rx, Ry, Rz, User, Tool,
                        isJointNear=None, JointNear=None):
        """逆解运算：给定末端笛卡尔坐标，计算各关节角度。

        isJointNear 可选。0 或不填表示按当前关节角度就近选解，1 表示按 JointNear 就近选解
        JointNear   可选。用于就近选解的关节坐标，格式 {j1,j2,j3,j4,j5,j6}
        """
        string = "InverseSolution({:s},{:d},{:d}".format(
            _floats((X, Y, Z, Rx, Ry, Rz), "pose"),
            _as_int(User, "User"), _as_int(Tool, "Tool"))
        if isJointNear is not None:
            string += ",{:d}".format(_as_int(isJointNear, "isJointNear"))
            if JointNear is not None:
                string += "," + _table(JointNear, "JointNear")
        elif JointNear is not None:
            raise DobotParamError("指定 JointNear 时必须同时把 isJointNear 设为 1")
        return self.sendRecvMsg(string + ")")

    def GetSixForceData(self):
        """获取六维力数据，返回 {Fx,Fy,Fz,Mx,My,Mz,isErr,isOnline,isOpen}。"""
        return self.sendRecvMsg("GetSixForceData()")

    def GetAngle(self):
        """获取机器人当前位姿的关节坐标 {J1~J6}。"""
        return self.sendRecvMsg("GetAngle()")

    def GetPose(self, user=None, tool=None):
        """获取机器人当前位姿的笛卡尔坐标 {X,Y,Z,Rx,Ry,Rz}。

        user/tool 可选，指定已标定的坐标系索引；不指定时使用全局坐标系。
        """
        parts = _optional_parts(("User", "Tool"), (), {"User": user, "Tool": tool})
        return self.sendRecvMsg("GetPose(" + ",".join(parts) + ")")

    def GetErrorID(self):
        """获取机器人当前报错的错误码（控制器 3.5.2 及以上）。

        返回 {[[控制器与算法报警], [伺服1], ..., [伺服6]]}，可用
        parse_error_id() 解析，或直接调用 GetErrorDescription() 取可读描述。
        """
        return self.sendRecvMsg("GetErrorID()")

    def GetErrorDescription(self, lang: str = "zh_CN") -> Dict[str, Any]:
        """获取当前报警的可读描述。

        把 GetErrorID 返回的 id 与 files/alarm_controller.json、
        files/alarm_servo.json 中的描述关联起来。
        """
        controller_ids, servo_ids = parse_error_id(self.GetErrorID())
        controller_table, servo_table = alarmAlarmJsonFile()
        controller_index = _alarm_index(controller_table)
        servo_index = _alarm_index(servo_table)
        return {
            "controller": [
                {"id": i, "description": _describe_alarm(i, controller_index, lang)}
                for i in controller_ids],
            "servo": {
                axis: [{"id": i, "description": _describe_alarm(i, servo_index, lang)}
                       for i in ids]
                for axis, ids in enumerate(servo_ids, start=1) if ids},
        }

    def PalletCreate(self, P1, P2, P3, P4, row, col, Palletname):
        """创建托盘（控制器 3.5.7 及以上，最多 20 个）。

        P1~P4      托盘四角的笛卡尔坐标，格式 {X,Y,Z,Rx,Ry,Rz}
        row / col  托盘的行数和列数
        Palletname 托盘名称，不可重复
        """
        string = "PalletCreate({:s},{:s},{:s},{:s},row={:d},col={:d},{:s})".format(
            _table(P1, "P1"), _table(P2, "P2"), _table(P3, "P3"), _table(P4, "P4"),
            _as_int(row, "row"), _as_int(col, "col"),
            _as_name(Palletname, "Palletname"))
        return self.sendRecvMsg(string)

    def GetPalletPose(self, Palletname, index):
        """获取已创建托盘的指定点位（控制器 3.5.7 及以上，index 从 1 开始）。"""
        string = "GetPalletPose({:s},{:d})".format(
            _as_name(Palletname, "Palletname"), _as_int(index, "index"))
        return self.sendRecvMsg(string)

    # ================== 2.4 IO 相关指令 ==================

    def DO(self, index, status):
        """设置数字输出端口状态（队列指令）。

        index  DO 端子编号，取值范围 [1,16] 或 [100,1000]
               （取 [100,1000] 时需要扩展 IO 模块的硬件支持）
        status 1 有信号，0 无信号
        """
        string = "DO({:d},{:d})".format(_as_int(index, "index"), _as_int(status, "status"))
        return self.sendRecvMsg(string)

    def DOExecute(self, index, status):
        """设置数字输出端口状态（立即指令，无视指令队列）。

        index  DO 端子编号，取值范围 [1,16] 或 [100,1000]
        status 1 有信号，0 无信号
        """
        string = "DOExecute({:d},{:d})".format(
            _as_int(index, "index"), _as_int(status, "status"))
        return self.sendRecvMsg(string)

    def DOGroup(self, *dynParams):
        """设置多个数字输出端口状态（立即指令）。

        参数为 index/value 交替出现的序列，支持两种写法：
            DOGroup(4, 1, 6, 0)
            DOGroup((4, 1), (6, 0))
        """
        values: List[Any] = []
        for item in dynParams:
            if isinstance(item, (tuple, list)):
                values.extend(item)
            else:
                values.append(item)
        if not values or len(values) % 2 != 0:
            raise DobotParamError(
                f"DOGroup 需要成对的 index/value 参数，收到 {len(values)} 个值")
        string = "DOGroup(" + ",".join(
            str(_as_int(v, "DOGroup")) for v in values) + ")"
        return self.sendRecvMsg(string)

    def ToolDO(self, index, status):
        """设置末端数字输出端口状态（队列指令，使用前需先 EnableRobot）。

        index  末端 DO 端子编号，取值范围 1/2
        status 1 有信号，0 无信号
        """
        string = "ToolDO({:d},{:d})".format(
            _as_int(index, "index"), _as_int(status, "status"))
        return self.sendRecvMsg(string)

    def ToolDOExecute(self, index, status):
        """设置末端数字输出端口状态（立即指令）。

        index  末端 DO 端子编号，取值范围 1/2
        status 1 有信号，0 无信号
        """
        string = "ToolDOExecute({:d},{:d})".format(
            _as_int(index, "index"), _as_int(status, "status"))
        return self.sendRecvMsg(string)

    def AO(self, index, value):
        """设置模拟输出端口电压值（队列指令）。

        index AO 端子编号，取值范围 1/2
        value 输出电压，取值范围 0~10V
        """
        string = "AO({:d},{:f})".format(
            _as_int(index, "index"), _as_float(value, "value"))
        return self.sendRecvMsg(string)

    def AOExecute(self, index, value):
        """设置模拟输出端口电压值（立即指令）。

        index AO 端子编号，取值范围 1/2
        value 输出电压，取值范围 0~10V
        """
        string = "AOExecute({:d},{:f})".format(
            _as_int(index, "index"), _as_float(value, "value"))
        return self.sendRecvMsg(string)

    def DI(self, index):
        """获取 DI 端口状态，返回 {value}，0 无信号，1 有信号。"""
        return self.sendRecvMsg("DI({:d})".format(_as_int(index, "index")))

    def DIGroup(self, *dynParams):
        """获取多个 DI 端口状态。

        支持 DIGroup(4, 6, 2, 7) 与 DIGroup((4, 6, 2, 7)) 两种写法。
        """
        values: List[Any] = []
        for item in dynParams:
            if isinstance(item, (tuple, list)):
                values.extend(item)
            else:
                values.append(item)
        if not values:
            raise DobotParamError("DIGroup 至少需要一个 DI 端子编号")
        string = "DIGroup(" + ",".join(
            str(_as_int(v, "DIGroup")) for v in values) + ")"
        return self.sendRecvMsg(string)

    def ToolDI(self, index):
        """获取末端 DI 端口状态，index 取值范围 1/2。"""
        return self.sendRecvMsg("ToolDI({:d})".format(_as_int(index, "index")))

    def AI(self, index):
        """获取 AI 端口的电压值，index 取值范围 1/2。"""
        return self.sendRecvMsg("AI({:d})".format(_as_int(index, "index")))

    def ToolAI(self, index):
        """获取末端 AI 端口的电压值，index 取值范围 1/2。"""
        return self.sendRecvMsg("ToolAI({:d})".format(_as_int(index, "index")))

    def SetTerminal485(self, baudRate, dataLen=None, parityBit=None, stopBit=None):
        """设置末端 485 端口参数（控制器 3.5.7 及以上）。

        baudRate  波特率
        dataLen   数据位长度，目前固定为 8
        parityBit 奇偶校验位，目前固定为 "N"
        stopBit   停止位长度，目前固定为 1

        除 baudRate 外均可省略，协议示例即 SetTerminal485(115200)。
        """
        parts = [str(_as_int(baudRate, "baudRate"))]
        if dataLen is not None:
            parts.append(str(_as_int(dataLen, "dataLen")))
            if parityBit is not None:
                parts.append(_as_name(parityBit, "parityBit"))
                if stopBit is not None:
                    parts.append(str(_as_int(stopBit, "stopBit")))
            elif stopBit is not None:
                raise DobotParamError("指定 stopBit 时必须同时给出 parityBit")
        elif parityBit is not None or stopBit is not None:
            raise DobotParamError("指定 parityBit/stopBit 时必须同时给出 dataLen")
        return self.sendRecvMsg("SetTerminal485(" + ",".join(parts) + ")")

    def GetTerminal485(self):
        """获取末端 485 端口参数，返回 {baudRate, parityBit, stopBit}。"""
        return self.sendRecvMsg("GetTerminal485()")

    # ================== 2.5 Modbus 相关指令 ==================

    def ModbusCreate(self, ip, port, slave_id, isRTU=None):
        """创建 Modbus 主站并连接从站（控制器 3.5.2 及以上，最多 5 个设备）。

        ip       从站 IP。不指定或为 127.0.0.1 / 0.0.0.1 时表示本机从站
        port     从站端口
        slave_id 从站 ID
        isRTU    可选。不填或 0 建立 ModbusTCP，1 建立 ModbusRTU

        返回值中的 index 为主站索引（0~4），后续 Modbus 指令都要用它。
        """
        string = "ModbusCreate({:s},{:d},{:d}".format(
            _as_name(ip, "ip"), _as_int(port, "port"), _as_int(slave_id, "slave_id"))
        if isRTU is not None:
            string += ",{:d}".format(_as_int(isRTU, "isRTU"))
        return self.sendRecvMsg(string + ")")

    def ModbusClose(self, index):
        """和 Modbus 从站断开连接，释放主站（控制器 3.5.2 及以上）。"""
        return self.sendRecvMsg("ModbusClose({:d})".format(_as_int(index, "index")))

    def GetInBits(self, index, addr, count):
        """读取触点寄存器（离散输入）的值，count 取值范围 1~16。"""
        string = "GetInBits({:d},{:d},{:d})".format(
            _as_int(index, "index"), _as_int(addr, "addr"), _as_int(count, "count"))
        return self.sendRecvMsg(string)

    def GetInRegs(self, index, addr, count, valType=None):
        """读取输入寄存器的值。

        index   创建主站时返回的主站索引
        addr    输入寄存器起始地址
        count   连续读取的数量，取值范围 [1,4]
        valType 可选。为空或 U16 表示 16 位无符号整数，另可取 U32/F32/F64
        """
        string = "GetInRegs({:d},{:d},{:d}".format(
            _as_int(index, "index"), _as_int(addr, "addr"), _as_int(count, "count"))
        if valType is not None:
            string += ",{:s}".format(_as_name(valType, "valType"))
        return self.sendRecvMsg(string + ")")

    def GetCoils(self, index, addr, count):
        """读取线圈寄存器的值，count 取值范围 [1,16]。"""
        string = "GetCoils({:d},{:d},{:d})".format(
            _as_int(index, "index"), _as_int(addr, "addr"), _as_int(count, "count"))
        return self.sendRecvMsg(string)

    def SetCoils(self, index, addr, count, valTab):
        """将指定的值写入线圈寄存器。

        valTab 要写入的值，数量与 count 相同，格式 {1,0,1}，可传序列或字符串
        """
        string = "SetCoils({:d},{:d},{:d},{:s})".format(
            _as_int(index, "index"), _as_int(addr, "addr"), _as_int(count, "count"),
            _table(valTab, "valTab"))
        return self.sendRecvMsg(string)

    def GetHoldRegs(self, index, addr, count, valType=None, timeout: Optional[float] = None):
        """读取保持寄存器的值。

        index   主站索引，取值范围 [0,4]
        addr    保持寄存器起始地址
        count   连续读取的数量，取值范围 [1,4]
        valType 可选。为空或 U16 表示 16 位无符号整数，另可取 U32/F32/F64
        timeout 等待应答超时（秒），为 None 时使用默认 reply_timeout
        """
        string = "GetHoldRegs({:d},{:d},{:d}".format(
            _as_int(index, "index"), _as_int(addr, "addr"), _as_int(count, "count"))
        if valType is not None:
            string += ",{:s}".format(_as_name(valType, "valType"))
        return self.sendRecvMsg(string + ")", timeout=timeout)

    def SetHoldRegs(self, index, addr, count, valTab, valType=None, timeout: Optional[float] = None):
        """将指定的值写入保持寄存器。

        count   连续写入的数量，取值范围 [1,4]
        valTab  要写入的值，数量与 count 相同，格式 {6000,300}
        valType 可选。为空或 U16 表示 16 位无符号整数，另可取 U32/F32/F64
        timeout 等待应答超时（秒），为 None 时使用默认 reply_timeout
        """
        string = "SetHoldRegs({:d},{:d},{:d},{:s}".format(
            _as_int(index, "index"), _as_int(addr, "addr"), _as_int(count, "count"),
            _table(valTab, "valTab"))
        if valType is not None:
            string += ",{:s}".format(_as_name(valType, "valType"))
        return self.sendRecvMsg(string + ")", timeout=timeout)

    # ---- 以下指令未在控制柜 V3 六轴 TCP/IP 协议中定义 ----
    # 保留是为了兼容既有调用，但控制器可能返回 -10000（命令不存在）。
    # 使用前请先确认控制器版本与工艺包是否支持。

    def Arch(self, index):
        """【非 V3 协议指令】设置 Jump 门型参数索引，四轴/Magician 系列遗留接口。"""
        return self.sendRecvMsg("Arch({:d})".format(_as_int(index, "index")))

    def LimZ(self, value):
        """【非 V3 协议指令】设置门型参数的最大抬升高度，四轴系列遗留接口。"""
        return self.sendRecvMsg("LimZ({:d})".format(_as_int(value, "value")))

    def SetObstacleAvoid(self, status):
        """【非 V3 协议指令】开关避障功能。"""
        return self.sendRecvMsg("SetObstacleAvoid({:d})".format(_as_int(status, "status")))

    def SetTerminalKeys(self, status):
        """【非 V3 协议指令】开关末端按键。CR 系列在 TCP/IP 模式下无法使用末端按键。"""
        return self.sendRecvMsg("SetTerminalKeys({:d})".format(_as_int(status, "status")))

    def SetPayload(self, load, *dynParams):
        """【非 V3 协议指令】设置末端负载，建议改用协议指令 PayLoad(weight,inertia)。

        原实现会把参数拼成 ``SetPayload(1.5000000.4,)``（缺逗号且多逗号），此处已修正。
        """
        string = "SetPayload({:f}".format(_as_float(load, "load"))
        for params in dynParams:
            values = params if isinstance(params, (tuple, list)) else (params,)
            for value in values:
                string += ",{:f}".format(_as_float(value, "dynParams"))
        return self.sendRecvMsg(string + ")")


# ---------------------------------------------------------------------------
# 运动指令（30003 端口，均为队列指令）
# ---------------------------------------------------------------------------


class DobotApiMove(DobotApi):
    """30003 端口：运动相关指令。

    协议规定运动指令的可选参数只能以 ``Key=Value`` 形式携带，因此各方法都提供了
    关键字参数（推荐，如 ``speedJ=50``）；``*dynParams`` 仍兼容按顺序传值以及
    直接传 ``"SpeedJ=50"`` 字符串的历史写法。

    另外协议明确：TCP 运动指令不支持在可选参数中携带 CP 与 SYNC，
    平滑过渡请用 29999 端口的 CP(R)，同步请用 Sync()。
    """

    def MovJ(self, x, y, z, rx, ry, rz, *dynParams,
             user=None, tool=None, speedJ=None, accJ=None):
        """从当前位置以关节运动方式运动至笛卡尔坐标目标点。

        x/y/z    目标点位置，单位 mm
        rx/ry/rz 目标点姿态，单位度
        可选参数 User、Tool、SpeedJ、AccJ
        """
        string = "MovJ({:s}".format(_floats((x, y, z, rx, ry, rz), "pose"))
        parts = _optional_parts(
            ("User", "Tool", "SpeedJ", "AccJ"), dynParams,
            {"User": user, "Tool": tool, "SpeedJ": speedJ, "AccJ": accJ})
        return self.sendRecvMsg(_finish(string, parts))

    def MovL(self, x, y, z, rx, ry, rz, *dynParams,
             user=None, tool=None, speedL=None, accL=None):
        """从当前位置以直线运动方式运动至笛卡尔坐标目标点。

        可选参数 User、Tool、SpeedL、AccL
        """
        string = "MovL({:s}".format(_floats((x, y, z, rx, ry, rz), "pose"))
        parts = _optional_parts(
            ("User", "Tool", "SpeedL", "AccL"), dynParams,
            {"User": user, "Tool": tool, "SpeedL": speedL, "AccL": accL})
        return self.sendRecvMsg(_finish(string, parts))

    def JointMovJ(self, j1, j2, j3, j4, j5, j6, *dynParams, speedJ=None, accJ=None):
        """从当前位置以关节运动方式运动至关节坐标目标点。

        j1~j6 各关节目标位置，单位度
        可选参数 SpeedJ、AccJ
        """
        string = "JointMovJ({:s}".format(_floats((j1, j2, j3, j4, j5, j6), "joint"))
        parts = _optional_parts(("SpeedJ", "AccJ"), dynParams,
                                {"SpeedJ": speedJ, "AccJ": accJ})
        return self.sendRecvMsg(_finish(string, parts))

    def MovLIO(self, x, y, z, rx, ry, rz, *ioGroups,
               user=None, tool=None, speedL=None, accL=None):
        """直线运动并在运动过程中并行设置数字输出端口状态。

        每个 ioGroup 为 (Mode, Distance, Index, Status) 四元组，会按协议拼成
        ``{Mode,Distance,Index,Status}``，可设置多组：

            MovLIO(-500, 100, 200, 150, 0, 90, (0, 50, 1, 0), (1, 1, 2, 1))

        Mode     0 表示距离百分比，1 表示距离数值
        Distance 正数表示离起点的距离，负数表示离目标点的距离；
                 Mode 为 0 时取值范围 (0,100]，Mode 为 1 时单位为 mm
        Index    DO 端子编号，取值范围 [1,24]
        Status   0 无信号，1 有信号
        """
        string = "MovLIO({:s}".format(_floats((x, y, z, rx, ry, rz), "pose"))
        for group in ioGroups:
            string += "," + _int_group(group, "ioGroup", size=4)
        parts = _optional_parts(
            ("User", "Tool", "SpeedL", "AccL"), (),
            {"User": user, "Tool": tool, "SpeedL": speedL, "AccL": accL})
        return self.sendRecvMsg(_finish(string, parts))

    def MovJIO(self, x, y, z, rx, ry, rz, *ioGroups,
               user=None, tool=None, speedJ=None, accJ=None):
        """关节运动并在运动过程中并行设置数字输出端口状态。

        ioGroup 的含义与 MovLIO 相同，可选参数为 User、Tool、SpeedJ、AccJ。
        """
        string = "MovJIO({:s}".format(_floats((x, y, z, rx, ry, rz), "pose"))
        for group in ioGroups:
            string += "," + _int_group(group, "ioGroup", size=4)
        parts = _optional_parts(
            ("User", "Tool", "SpeedJ", "AccJ"), (),
            {"User": user, "Tool": tool, "SpeedJ": speedJ, "AccJ": accJ})
        return self.sendRecvMsg(_finish(string, parts))

    def Arc(self, x1, y1, z1, rx1, ry1, rz1, x2, y2, z2, rx2, ry2, rz2, *dynParams,
            user=None, tool=None, speedL=None, accL=None):
        """圆弧插补运动。

        第一组坐标为圆弧中间点，第二组为目标点。当前位置不能在两点确定的直线上。
        协议原型中这 12 个参数是平铺的，不带大括号。
        """
        string = "Arc({:s}".format(
            _floats((x1, y1, z1, rx1, ry1, rz1, x2, y2, z2, rx2, ry2, rz2), "pose"))
        parts = _optional_parts(
            ("User", "Tool", "SpeedL", "AccL"), dynParams,
            {"User": user, "Tool": tool, "SpeedL": speedL, "AccL": accL})
        return self.sendRecvMsg(_finish(string, parts))

    def Circle3(self, x1, y1, z1, rx1, ry1, rz1, x2, y2, z2, rx2, ry2, rz2, count,
                *dynParams, user=None, tool=None, speedL=None, accL=None):
        """整圆插补运动（控制器 3.5.5 及以上）。

        协议原型为 ``Circle3({P1},{P2},count,...)``，两个点位必须各用大括号包裹。
        count 为圈数，取值范围 1~999。
        """
        string = "Circle3({:s},{:s},{:d}".format(
            _table((x1, y1, z1, rx1, ry1, rz1), "P1"),
            _table((x2, y2, z2, rx2, ry2, rz2), "P2"),
            _as_int(count, "count"))
        parts = _optional_parts(
            ("User", "Tool", "SpeedL", "AccL"), dynParams,
            {"User": user, "Tool": tool, "SpeedL": speedL, "AccL": accL})
        return self.sendRecvMsg(_finish(string, parts))

    def ServoJ(self, j1, j2, j3, j4, j5, j6, t=None, lookahead_time=None, gain=None):
        """基于关节空间的动态跟随命令，建议以 30ms 以上的间隔循环调用。

        协议明确该指令“返回：无”，因此这里只下发不等待应答。

        t              可选，该点位的运行时间，单位 s，取值范围 [0.02,3600.0]，默认 0.1
        lookahead_time 可选，作用类似 PID 的 D 项，取值范围 [20.0,100.0]，默认 50
        gain           可选，作用类似 PID 的 P 项，取值范围 [200.0,1000.0]，默认 500

        三个可选参数仅控制器 3.5.5 及以上支持，不传时不会出现在指令里，
        以便兼容低版本控制器。
        """
        string = "ServoJ({:s}".format(_floats((j1, j2, j3, j4, j5, j6), "joint"))
        for name, value in (("t", t), ("lookahead_time", lookahead_time), ("gain", gain)):
            if value is not None:
                string += ",{:s}={:f}".format(name, _as_float(value, name))
        self.sendCmd(string + ")")

    def ServoP(self, x, y, z, rx, ry, rz):
        """基于笛卡尔空间的动态跟随命令，建议以 30ms 的间隔循环调用。

        协议明确该指令“返回：无”，因此这里只下发不等待应答。
        """
        self.sendCmd("ServoP({:s})".format(_floats((x, y, z, rx, ry, rz), "pose")))

    def MoveJog(self, axisID="", *dynParams, coordType=None, user=None, tool=None):
        """点动或停止点动机器人（控制器 3.5.2 及以上）。

        axisID    点动运动轴，区分大小写：J1+~J6+、J1-~J6-、X+/X-、Y+/Y-、Z+/Z-、
                  Rx+/Rx-、Ry+/Ry-、Rz+/Rz-。不携带或携带无效值表示停止点动。
        coordType 可选。0 表示用户坐标系，1 表示关节点动（默认），2 表示工具坐标系。
                  axisID 为笛卡尔坐标轴时只能取 0 或 2。
        user/tool 可选，已标定的坐标系索引，取值范围 [0,9]，默认 0
        """
        axis = str(axisID).strip()
        valid_axes = {
            'J1+', 'J1-', 'J2+', 'J2-', 'J3+', 'J3-', 'J4+', 'J4-', 'J5+', 'J5-', 'J6+', 'J6-',
            'X+', 'X-', 'Y+', 'Y-', 'Z+', 'Z-', 'Rx+', 'Rx-', 'Ry+', 'Ry-', 'Rz+', 'Rz-'
        }

        # 1. 未携带或携带无效轴名称时，按协议下发 MoveJog() 停止点动
        if not axis or axis not in valid_axes:
            return self.sendRecvMsg("MoveJog()")

        # 2. 区分关节点动与笛卡尔点动
        is_joint = axis.startswith('J')
        explicit = {}

        if is_joint:
            # 关节点动时 CoordType 默认为 1，可缺省；若显式指定则只允许为 1
            if coordType is not None and coordType != 1:
                raise DobotParamError(f"关节轴 {axis} 点动时 CoordType 只能为 1（关节点动），收到 {coordType}")
        else:
            # 笛卡尔点动时 CoordType 只能为 0（用户坐标系）或 2（工具坐标系）
            c_type = 2 if coordType is None else coordType
            if c_type not in (0, 2):
                raise DobotParamError(f"笛卡尔轴 {axis} 点动时 CoordType 只能为 0（用户坐标系）或 2（工具坐标系），收到 {coordType}")
            explicit["CoordType"] = c_type

        if user is not None:
            if not (0 <= user <= 9):
                raise DobotParamError(f"User 索引范围必须在 [0, 9]，收到 {user}")
            explicit["User"] = user

        if tool is not None:
            if not (0 <= tool <= 9):
                raise DobotParamError(f"Tool 索引范围必须在 [0, 9]，收到 {tool}")
            explicit["Tool"] = tool

        parts = _optional_parts(("CoordType", "User", "Tool"), dynParams, explicit)
        args = [axis] + parts
        cmd = "MoveJog(" + ",".join(args) + ")"
        self.log(f"MoveJog: {cmd}")
        return self.sendRecvMsg(cmd)

    def StartTrace(self, traceName):
        """轨迹拟合：用轨迹文件中的记录点位拟合运动路径（控制器 3.5.2 及以上）。

        调用前需先用 GetTraceStartPose 取首个点位并运动到该点。
        """
        return self.sendRecvMsg(
            "StartTrace({:s})".format(_as_name(traceName, "traceName")))

    def StartPath(self, traceName, const, cart):
        """轨迹复现：复现录制的运动轨迹（控制器 3.5.2 及以上）。

        const 1 表示匀速复现（移除轨迹中的停顿），0 表示按原速复现
        cart  1 表示按笛卡尔路径复现，0 表示按关节路径复现

        调用前需先用 GetPathStartPose 取首个关节点位并运动到该点。
        """
        string = "StartPath({:s},{:d},{:d})".format(
            _as_name(traceName, "traceName"),
            _as_int(const, "const"), _as_int(cart, "cart"))
        return self.sendRecvMsg(string)

    def Sync(self, timeout: float = QUEUE_REPLY_TIMEOUT):
        """阻塞程序执行队列指令，待队列最后的指令执行完后才返回。

        该指令的应答时间取决于队列里剩余运动的耗时，因此默认使用很长的超时。
        """
        return self.sendRecvMsg("Sync()", timeout=timeout)

    def RelMovJTool(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz,
                    tool, *dynParams, speedJ=None, accJ=None, user=None):
        """沿工具坐标系进行相对运动，末端运动方式为关节运动（控制器 3.5.2 及以上）。

        tool 已标定的工具坐标系索引，取值范围 [0,9]
        可选参数 SpeedJ、AccJ、User
        """
        string = "RelMovJTool({:s},{:d}".format(
            _floats((offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz),
                    "offset"),
            _as_int(tool, "tool"))
        parts = _optional_parts(("SpeedJ", "AccJ", "User"), dynParams,
                                {"SpeedJ": speedJ, "AccJ": accJ, "User": user},
                                allow_positional=True)
        return self.sendRecvMsg(_finish(string, parts))

    def RelMovLTool(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz,
                    tool, *dynParams, speedL=None, accL=None, user=None):
        """沿工具坐标系进行相对运动，末端运动方式为直线运动（控制器 3.5.2 及以上）。

        tool 已标定的工具坐标系索引，取值范围 [0,9]
        可选参数 SpeedL、AccL、User
        """
        string = "RelMovLTool({:s},{:d}".format(
            _floats((offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz),
                    "offset"),
            _as_int(tool, "tool"))
        parts = _optional_parts(("SpeedL", "AccL", "User"), dynParams,
                                {"SpeedL": speedL, "AccL": accL, "User": user},
                                allow_positional=True)
        return self.sendRecvMsg(_finish(string, parts))

    def RelMovJUser(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz,
                    user, *dynParams, speedJ=None, accJ=None, tool=None):
        """沿用户坐标系进行相对运动，末端运动方式为关节运动（控制器 3.5.2 及以上）。

        user 已标定的用户坐标系索引，取值范围 [0,9]
        可选参数 SpeedJ、AccJ、Tool
        """
        string = "RelMovJUser({:s},{:d}".format(
            _floats((offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz),
                    "offset"),
            _as_int(user, "user"))
        parts = _optional_parts(("SpeedJ", "AccJ", "Tool"), dynParams,
                                {"SpeedJ": speedJ, "AccJ": accJ, "Tool": tool},
                                allow_positional=True)
        return self.sendRecvMsg(_finish(string, parts))

    def RelMovLUser(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz,
                    user, *dynParams, speedL=None, accL=None, tool=None):
        """沿用户坐标系进行相对运动，末端运动方式为直线运动（控制器 3.5.2 及以上）。

        user 已标定的用户坐标系索引，取值范围 [0,9]
        可选参数 SpeedL、AccL、Tool
        """
        string = "RelMovLUser({:s},{:d}".format(
            _floats((offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz),
                    "offset"),
            _as_int(user, "user"))
        parts = _optional_parts(("SpeedL", "AccL", "Tool"), dynParams,
                                {"SpeedL": speedL, "AccL": accL, "Tool": tool},
                                allow_positional=True)
        return self.sendRecvMsg(_finish(string, parts))

    def RelJointMovJ(self, offset1, offset2, offset3, offset4, offset5, offset6,
                     *dynParams, speedJ=None, accJ=None):
        """沿关节坐标系进行相对运动，末端运动方式为关节运动（控制器 3.5.2 及以上）。

        offset1~offset6 各关节偏移量，单位度
        可选参数 SpeedJ、AccJ
        """
        string = "RelJointMovJ({:s}".format(
            _floats((offset1, offset2, offset3, offset4, offset5, offset6), "offset"))
        parts = _optional_parts(("SpeedJ", "AccJ"), dynParams,
                                {"SpeedJ": speedJ, "AccJ": accJ})
        return self.sendRecvMsg(_finish(string, parts))

    # ---- 以下指令未在控制柜 V3 六轴 TCP/IP 协议中定义 ----
    # 保留是为了兼容既有调用，但控制器可能返回 -10000（命令不存在）。
    # 请优先使用上面的协议指令：相对运动用 RelJointMovJ / RelMovLUser 等。

    def RelMovJ(self, offset1, offset2, offset3, offset4, offset5, offset6, *dynParams):
        """【非 V3 协议指令】关节偏移运动，建议改用 RelJointMovJ。"""
        string = "RelMovJ({:s}".format(
            _floats((offset1, offset2, offset3, offset4, offset5, offset6), "offset"))
        return self.sendRecvMsg(_finish(string, [_raw_optional(p) for p in dynParams]))

    def RelMovL(self, offsetX, offsetY, offsetZ, *dynParams):
        """【非 V3 协议指令】笛卡尔偏移运动，建议改用 RelMovLUser。"""
        string = "RelMovL({:s}".format(_floats((offsetX, offsetY, offsetZ), "offset"))
        return self.sendRecvMsg(_finish(string, [_raw_optional(p) for p in dynParams]))

    def ServoJS(self, j1, j2, j3, j4, j5, j6):
        """【非 V3 协议指令】基于关节空间的动态跟随，建议改用 ServoJ。"""
        return self.sendRecvMsg(
            "ServoJS({:s})".format(_floats((j1, j2, j3, j4, j5, j6), "joint")))

    def StartFCTrace(self, traceName):
        """【非 V3 协议指令】带力控的轨迹拟合，需力控工艺包支持。"""
        return self.sendRecvMsg(
            "StartFCTrace({:s})".format(_as_name(traceName, "traceName")))


# ---------------------------------------------------------------------------
# 实时反馈（30004 / 30005 / 30006 端口）
# ---------------------------------------------------------------------------


class DobotApiFeedBack(DobotApi):
    """实时反馈端口，每个数据包 1440 字节。

    30004 每 8ms、30005 每 200ms、30006 默认每 50ms 反馈一次。由于 TCP 是流式
    协议，一次 recv 可能拿到半个包，也可能拿到多个包，因此这里持续累积缓冲区，
    并用协议提供的 MessageSize 与 TestValue 两个字段校验包头对齐。
    """

    def __init__(self, ip, port, *args, verbose: bool = False,
                 connect_timeout: float = DEFAULT_CONNECT_TIMEOUT,
                 reply_timeout: float = DEFAULT_REPLY_TIMEOUT):
        if port not in FEEDBACK_PORTS:
            raise DobotParamError(
                f"实时反馈端口只能是 {'/'.join(str(p) for p in FEEDBACK_PORTS)}，"
                f"收到 {port}")
        super().__init__(ip, port, *args, verbose=verbose,
                         connect_timeout=connect_timeout, reply_timeout=reply_timeout)
        self._buffer = bytearray()
        self._latest: Optional[np.ndarray] = None

    # 反馈端口是只读的，禁止误用发送接口
    def send_data(self, string):
        raise DobotApiError(f"{self.port} 是只读的实时反馈端口，不能下发指令")

    def sendRecvMsg(self, string, timeout=None):
        raise DobotApiError(f"{self.port} 是只读的实时反馈端口，不能下发指令")

    def sendCmd(self, string):
        raise DobotApiError(f"{self.port} 是只读的实时反馈端口，不能下发指令")

    def _is_aligned(self, begin: int) -> bool:
        """校验 begin 处是否为一个包头对齐的完整数据包。"""
        if begin < 0 or begin + FEEDBACK_PACKAGE_SIZE > len(self._buffer):
            return False
        size = int.from_bytes(self._buffer[begin:begin + 2], "little")
        return size == FEEDBACK_PACKAGE_SIZE

    def _find_frame_start(self, search_from: int) -> int:
        """借助 TestValue 定位下一个包头，返回包起始下标，找不到返回 -1。"""
        index = self._buffer.find(_FEEDBACK_TEST_VALUE_BYTES, max(search_from, 0))
        while index != -1:
            begin = index - _FEEDBACK_TEST_VALUE_OFFSET
            if begin >= search_from and self._is_aligned(begin):
                return begin
            index = self._buffer.find(_FEEDBACK_TEST_VALUE_BYTES, index + 1)
        return -1

    def _take_latest_frame(self) -> Optional[bytes]:
        """取出缓冲区里最新的一个完整数据包，并丢弃它之前的陈旧数据。"""
        latest = -1
        search_from = 0
        while True:
            begin = self._find_frame_start(search_from)
            if begin < 0:
                break
            latest = begin
            search_from = begin + FEEDBACK_PACKAGE_SIZE

        if latest < 0:
            # 数据尚未凑够或全部无法对齐，保留尾部一小段继续等待后续字节
            if len(self._buffer) > FEEDBACK_PACKAGE_SIZE * 4:
                del self._buffer[:-FEEDBACK_PACKAGE_SIZE * 2]
            return None

        end = latest + FEEDBACK_PACKAGE_SIZE
        frame = bytes(self._buffer[latest:end])
        del self._buffer[:end]
        return frame

    def feedBackData(self, max_reads: int = 16):
        """返回最新一帧机械臂状态，类型为长度 1 的结构化数组。

        与旧实现的区别：
        - 保留 socket 超时，网络异常时抛 DobotTimeoutError 而不是永久阻塞；
        - 收到的数据会累积而不是相互覆盖，半包会被正确拼接；
        - 用 MessageSize 与 TestValue 校验包头对齐，错位时自动重新同步；
        - 缓冲区里存在多帧时返回最新的一帧，避免读到滞后的数据。
        """
        sock = self._require_socket()
        for _ in range(max_reads):
            frame = self._take_latest_frame()
            if frame is not None:
                self._latest = np.frombuffer(frame, dtype=MyType)
                return self._latest
            try:
                chunk = sock.recv(65536)
            except socket.timeout as exc:
                raise DobotTimeoutError(
                    f"{self.ip}:{self.port} 在 {self.reply_timeout}s 内没有反馈数据，"
                    "请检查网络环境") from exc
            except OSError as exc:
                logger.warning("Feedback socket error on %s:%d: %s", self.ip, self.port, exc)
                self._close_socket()
                if not self._is_closed and self.auto_reconnect:
                    if self.reconnect():
                        sock = self.socket_dobot
                        self._buffer.clear()
                        continue
                raise DobotConnectionError(
                    f"从 {self.ip}:{self.port} 接收反馈数据失败：{exc}") from exc
            if not chunk:
                logger.warning("Feedback connection to %s:%d closed by peer (EOF).", self.ip, self.port)
                self._close_socket()
                if not self._is_closed and self.auto_reconnect:
                    if self.reconnect():
                        sock = self.socket_dobot
                        self._buffer.clear()
                        continue
                raise DobotConnectionError(f"{self.ip}:{self.port} 连接已被对端关闭")
            self._buffer.extend(chunk)

        raise DobotProtocolError(
            f"连续 {max_reads} 次接收都无法凑出对齐的 {FEEDBACK_PACKAGE_SIZE} 字节数据包，"
            "请检查网络环境与控制器版本")

    @property
    def latest(self) -> Optional[np.ndarray]:
        """最近一次成功解析的反馈数据。"""
        return self._latest

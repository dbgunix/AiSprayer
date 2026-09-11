import math
import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, List, Union, Any, Dict

logger = logging.getLogger(__name__)

@dataclass
class RobotPose:
    """
    机器人直角坐标位姿（弧度制）
    默认坐标系为直角坐标系，坐标轴顺序为(x, y, z, a, b, c)
    平移单位: mm, 姿态角单位: rad。
    """
    x: float = 0.0; y: float = 0.0; z: float = 0.0
    a: float = 0.0; b: float = 0.0; c: float = 0.0

    @property
    def rx(self) -> float:
        return self.a

    @rx.setter
    def rx(self, val: float):
        self.a = float(val)

    @property
    def ry(self) -> float:
        return self.b

    @ry.setter
    def ry(self, val: float):
        self.b = float(val)

    @property
    def rz(self) -> float:
        return self.c

    @rz.setter
    def rz(self, val: float):
        self.c = float(val)

    @classmethod
    def from_list(cls, data: list) -> "RobotPose":
        """从长度 >=6 的列表创建位姿RobotPose, 不足部分填 0。"""
        if not data: return cls()
        return cls(*[float(x) for x in data[:6]])

    @classmethod
    def from_dict(cls, data: dict) -> "RobotPose":
        """从字典创建 RobotPose 对象。
        支持 key: x, y, z, 以及 rx/a, ry/b, rz/c。
        若 is_radians 为 False，则 rx, ry, rz 视为角度并自动转为弧度制。
        """
        if not data:
            return cls()
        x = float(data.get("x", 0.0))
        y = float(data.get("y", 0.0))
        z = float(data.get("z", 0.0))
        rx = float(data.get("rx", data.get("a", 0.0)))
        ry = float(data.get("ry", data.get("b", 0.0)))
        rz = float(data.get("rz", data.get("c", 0.0)))
        if not data.get("is_radians", False):
            rx = math.radians(rx)
            ry = math.radians(ry)
            rz = math.radians(rz)
        return cls(x=x, y=y, z=z, a=rx, b=ry, c=rz)

    def to_list(self) -> list:
        """转换为 [x,y,z,a,b,c] 列表。"""
        return [self.x, self.y, self.z, self.a, self.b, self.c]
    
    def __repr__(self):
        """返回可读性较好的字符串表示。"""
        return (f"RobotPose(x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f}, "
                f"rx={self.a:.3f}, ry={self.b:.3f}, rz={self.c:.3f})")
    
    def __eq__(self, other):
        """重载==，允许直接比较两个 RobotPose 对象是否相等。"""
        if not isinstance(other, RobotPose):
            return NotImplemented
        return (math.isclose(self.x, other.x) and math.isclose(self.y, other.y) and 
                math.isclose(self.z, other.z) and math.isclose(self.a, other.a) and 
                math.isclose(self.b, other.b) and math.isclose(self.c, other.c))

PoseLike = Union[RobotPose, List[float], Dict[str, Any]]

def _to_list(pose: PoseLike) -> list:
    """将 PoseLike 转换为 list 形式 [x, y, z, a, b, c]（弧度制）。"""
    if isinstance(pose, RobotPose):
        return pose.to_list()
    if isinstance(pose, dict):
        return RobotPose.from_dict(pose).to_list()
    return [float(x) for x in pose]


def is_spraying_on(value: Any, default: bool = True) -> bool:
    """
    将航点/航段的 spraying 字段规范为 bool (统一判定规则，避免各处重复实现)。
    兼容布尔值与轨迹生成器写入的字符串 ("on"/"off"/"true"/"false"/"1"/"0")，
    字段缺失 (None) 时返回 default (默认视为开启喷涂)。
    """
    if value is None:
        return default
    if isinstance(value, str):
        return value.strip().lower() not in ("off", "0", "false", "no", "")
    return bool(value)


class BaseRobotDriver(ABC):
    """
    机械臂驱动抽象基类，定义统一的对外控制接口。
    """
    
    @abstractmethod
    def startup(self, timeout: float = 10.0) -> bool:
        """
        启动机器人连接并使能伺服
        :param timeout: 超时时间(秒)
        :return: 是否启动成功
        """
        pass
        
    @abstractmethod
    def shutdown(self) -> None:
        """
        关闭机器人连接并下电/断开控制器
        """
        pass
        
    @abstractmethod
    def get_running_state(self) -> int:
        """
        获取机器人运行状态
        :return: 0=停止；1=暂停；2=运行中
        """
        pass
        
    @abstractmethod
    def is_robot_idle(self) -> bool:
        """
        运动程序是否空闲
        """
        pass

    @abstractmethod
    def get_current_pose(self) -> Optional[RobotPose]:
        """
        获取当前直角坐标位姿
        """
        pass

    @abstractmethod
    def get_current_joint(self) -> Optional[List[float]]:
        """
        获取当前关节角度列表 (单位: rad 或 按照驱动实现，通常与 move_joint 匹配)
        """
        pass

    @abstractmethod
    def is_reachable(self, pose: PoseLike, movetype: str = "MOVJ") -> bool:
        """
        判断目标位姿是否可达
        """
        pass

    @abstractmethod
    def move_j(
        self, 
        pose: PoseLike, 
        velocity: float = 40.0, 
        acc: float = 80.0, 
        dec: float = 80.0,
        tool_num: int = 0,
        wait: bool = True,
    ) -> int:
        """
        关节运动(MOVJ)，目标位姿以直角坐标(笛卡尔)描述，由控制器内部做逆解。
        :param pose: 目标位姿 RobotPose 或 [x(mm),y(mm),z(mm),A(rad),B(rad),C(rad)]
        :param velocity: 速度
        :param acc: 加速度
        :param dec: 减速度
        :param tool_num: 工具号
        :param wait: 是否等待运动完成
        :return: 返回码
        """
        pass

    @abstractmethod
    def move_joint(
        self, 
        joints: List[float], 
        velocity: float = 40.0, 
        acc: float = 80.0, 
        dec: float = 80.0,
        tool_num: int = 0,
        wait: bool = True,
    ) -> int:
        """
        关节坐标系运动(Joint Movement)。目标位置由各关节角度确定。
        :param joints: 目标关节角列表(通常长度为6)
        :param velocity: 速度
        :param acc: 加速度
        :param dec: 减速度
        :param tool_num: 工具号
        :param wait: 是否等待运动完成
        :return: 返回码
        """
        pass

    @abstractmethod
    def move_l(
        self, 
        pose: PoseLike, 
        velocity: float = 100.0, 
        acc: float = 80.0, 
        dec: float = 80.0,
        tool_num: int = 0,
        wait: bool = True,
    ) -> int:
        """
        直线运动(MOVL)，末端在笛卡尔空间走直线运动。
        :param pose: 目标位姿 RobotPose 或 [x(mm),y(mm),z(mm),A(rad),B(rad),C(rad)]
        :param velocity: 速度
        :param acc: 加速度
        :param dec: 减速度
        :param tool_num: 工具号
        :param wait: 是否等待运动完成
        :return: 返回码
        """
        pass

    @abstractmethod
    def move_j_queue(
        self, 
        poses: List[PoseLike], 
        velocity: float = 40.0, 
        acc: float = 80.0, 
        dec: float = 80.0,
        tool_num: int = 0,
        wait: bool = True,
        cp_ratio: int = 50
    ) -> int:
        """
        队列形式的关节运动(MOVJ)，用于多点连续平滑移动。
        :param poses: 目标位姿列表
        :param velocity: 速度
        :param acc: 加速度
        :param dec: 减速度
        :param tool_num: 工具号
        :param wait: 是否等待整个队列运动完成
        :param cp_ratio: 连续路径平滑过渡比例(1-100)，0表示关闭平滑
        :return: 返回码
        """
        pass

    @abstractmethod
    def move_l_queue(
        self, 
        poses: List[PoseLike], 
        velocity: float = 100.0, 
        acc: float = 80.0, 
        dec: float = 80.0,
        tool_num: int = 0,
        wait: bool = True,
        cp_ratio: int = 50
    ) -> int:
        """
        队列形式的直线运动(MOVL)，用于多点连续平滑移动。
        :param poses: 目标位姿列表
        :param velocity: 速度
        :param acc: 加速度
        :param dec: 减速度
        :param tool_num: 工具号
        :param wait: 是否等待整个队列运动完成
        :param cp_ratio: 连续路径平滑过渡比例(1-100)，0表示关闭平滑
        :return: 返回码
        """
        pass

    @abstractmethod
    def set_tool_number(self, tool_num: int) -> bool:
        """设置当前激活的工具编号"""
        pass

    @abstractmethod
    def set_global_speed(self, speed: int) -> bool:
        """设置全局速度百分比"""
        pass

    @abstractmethod
    def go_home(self, wait: bool = True, velocity: Optional[float] = None, acc: Optional[float] = None) -> int:
        """返回原点运动"""
        pass

    @abstractmethod
    def pause(self) -> bool:
        """Pause current motion"""
        pass

    @abstractmethod
    def resume(self) -> bool:
        """Resume paused motion"""
        pass

    @abstractmethod
    def estop(self) -> bool:
        """Emergency stop"""
        pass

    def clear_error(self) -> bool:
        """Clear error/alarm state on the robot controller"""
        return True

    def move_l_segments(
        self,
        segments: List[dict],
        velocity: float = 100.0,
        acc: float = 80.0,
        dec: float = 80.0,
        tool_num: Optional[int] = None,
        cp_ratio: int = 50,
        spray_do_index: int = 1,
    ) -> int:
        """
        分段执行连续笛卡尔直线轨迹 (MoveL)，并按各段的 spraying 状态在段边界切换喷涂 DO。

        DO 切换使用队列指令 (immediate=False)：与同批 MoveL 运动指令一起进入控制器算法队列
        按序执行，因此段间无需等待机械臂停止，整条轨迹保持 CP 平滑连续，仅在喷涂状态
        发生变化的边界下发一次 DO。调用方应已将相同 spraying 状态的相邻航点合并为一段。
        :param segments: 分段列表，每段格式为 {"spraying": bool|"on"/"off", "poses": List[PoseLike]}
        :param velocity: 目标线速度 (mm/s)
        :param acc: 加速度 (%)
        :param dec: 减速度 (%)
        :param tool_num: 工具坐标系编号 (None 时使用驱动层当前工具)
        :param cp_ratio: 连续路径平滑过渡比例 (0-100)
        :param spray_do_index: 喷涂开关 DO 端口编号
        :return: 0 成功，非 0 失败
        """
        # 只有最后一个含有效位姿的段需要 wait (等整条轨迹真正执行完)，其余段发送后立即排队下一批指令
        last_idx = max((i for i, s in enumerate(segments) if s.get("poses")), default=-1)

        do_status: Optional[int] = None  # None 表示尚未下发过 DO，首段强制下发一次以确保状态已知
        for seg_idx, seg in enumerate(segments):
            poses = seg.get("poses", [])
            if not poses:
                continue
            target_status = 1 if is_spraying_on(seg.get("spraying")) else 0
            if do_status != target_status:
                logger.info(f"move_l_segments: 切换喷涂 DO({spray_do_index}) -> {target_status} "
                            f"(segment {seg_idx + 1}/{len(segments)}, 队列指令)")
                if not self.set_do(spray_do_index, target_status, immediate=False):
                    # DO 下发失败时喷涂状态不可知，宁可中止轨迹也不让机械臂盲跑
                    logger.error(f"move_l_segments: 切换喷涂 DO({spray_do_index}) -> {target_status} 失败, 中止轨迹执行")
                    self.set_do(spray_do_index, 0, immediate=True)
                    return -1
                do_status = target_status

            res = self.move_l_queue(
                poses,
                velocity=velocity,
                acc=acc,
                dec=dec,
                tool_num=tool_num,
                wait=(seg_idx == last_idx),
                cp_ratio=cp_ratio,
            )
            if res != 0:
                logger.error(f"move_l_segments: Segment {seg_idx + 1} failed with error code {res}")
                # 轨迹中断时立即强制关喷 (立即指令，不受残留队列指令影响)
                self.set_do(spray_do_index, 0, immediate=True)
                return res

        # 所有分段执行完毕，若喷涂仍处于开启状态，必须主动安全关喷 (立即指令)
        if do_status == 1:
            logger.info(f"move_l_segments: 所有分段执行完毕，关闭喷涂 DO({spray_do_index}) (立即指令)")
            self.set_do(spray_do_index, 0, immediate=True)
        return 0

    def set_do(self, index: int, status: int, immediate: bool = False) -> bool:
        """
        设置机械臂数字输出 (DO) 端口状态。
        :param index: DO 端口编号 (1-based, 取值 1..16)
        :param status: 1: 有信号(高电平/开), 0: 无信号(低电平/关)
        :param immediate: True 为立即指令 (无视队列立即生效), False 为队列指令 (排入算法队列顺序执行)
        :return: 是否设置成功
        """
        logger.warning(f"set_do({index}, {status}) 当前驱动未实现, DO 控制被忽略")
        return False

    def get_feedback_diagnostics(self) -> dict:
        """获取机械臂实时诊断与动力学反馈数据（笛卡尔位姿、关节速度、负载重量、报警状态、DO 状态等）"""
        return {
            "tool_vector_actual": [0.0]*6,
            "qd_actual": [0.0]*6,
            "load": 0.0,
            "error_status": 0,
            "tcp_speed_actual": [0.0]*6,
            "digital_outputs": [0]*16,
            "digital_output_bits": 0,
        }


import math
import os
import xml.etree.ElementTree as ET
import yaml
import logging
from typing import Any, Dict, List, Optional, Tuple, Union

logger = logging.getLogger(__name__)

# 项目根目录 (SprayAnything/)
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))

# 旧版扁平配置键名与标准命名空间键名映射表 (保证平滑向后兼容)
_LEGACY_KEY_ALIASES: Dict[str, List[str]] = {
    "robot.ip": ["robot_ip"],
    "robot.port": ["robot_port"],
    "calib.board.cols": ["calib_board_cols"],
    "calib.board.rows": ["calib_board_rows"],
    "robot_ip": ["robot.ip"],
    "robot_port": ["robot.port"],
    "calib_board_cols": ["calib.board.cols"],
    "calib_board_rows": ["calib.board.rows"],
}

# 统一系统受管配置注册表 (Declarative System Configuration Registry)
CONFIG_REGISTRY: List[Dict[str, Any]] = [
    # ─── 1. 机械臂控制与工具端 (Robot Hardware & Tooling) ────────────────────
    {
        "key": "robot.ip",
        "category": "robot",
        "label": "Robot Controller IP",
        "type": "string",
        "yaml_path": "hardware.robot.ip",
        "default": "192.168.5.1",
        "description": "Network IPv4 address of the Dobot robot controller.",
        "legacy_key": "robot_ip",
    },
    {
        "key": "robot.port",
        "category": "robot",
        "label": "Robot Control Port",
        "type": "number",
        "yaml_path": "hardware.robot.port",
        "default": 29999,
        "min": 1,
        "max": 65535,
        "description": "TCP control port (Dobot default 29999).",
        "legacy_key": "robot_port",
    },
    {
        "key": "robot.global_speed_factor",
        "category": "robot",
        "label": "Global Speed Factor (%)",
        "type": "number",
        "yaml_path": "hardware.robot.global_speed_factor",
        "default": 50,
        "min": 1,
        "max": 100,
        "step": 5,
        "description": "Global velocity scaling factor applied on startup and motion (1-100%).",
    },
    {
        "key": "robot.spray_do_index",
        "category": "robot",
        "label": "Spraying DO Port Index",
        "type": "number",
        "yaml_path": "hardware.robot.spray_do_index",
        "default": 1,
        "min": 1,
        "max": 16,
        "step": 1,
        "description": "Digital output terminal index for spray gun trigger (1-16).",
    },
    {
        "key": "robot.robot_tcp_id",
        "category": "robot",
        "label": "Robot TCP Tool ID",
        "type": "select",
        "yaml_path": "hardware.robot.robot_tcp_id",
        "default": 1,
        "options": [
            {"value": 0, "label": "0 - Flange / Default Tool"},
            {"value": 1, "label": "1 - Gripper Tip (gripper_tip_link)"},
            {"value": 2, "label": "2 - Laser/Spray Head (laser_head_link)"},
        ],
        "description": "Controller tool coordinate frame index (0: Flange, 1: Gripper, 2: Laser/Spray).",
    },
    {
        "key": "robot.robot_tcp",
        "category": "robot",
        "label": "Robot TCP Node Name",
        "type": "select",
        "yaml_path": "hardware.robot.robot_tcp",
        "default": "gripper_tip_link",
        "options": [
            {"value": "laser_head_link", "label": "laser_head_link"},
            {"value": "gripper_tip_link", "label": "gripper_tip_link"},
        ],
        "description": "URDF TCP frame link name corresponding to active end-effector.",
    },
    {
        "key": "robot.robot_urdf",
        "category": "robot",
        "label": "Robot URDF Model Path",
        "type": "string",
        "yaml_path": "hardware.robot.robot_urdf",
        "default": "app/urdf/cr5_robot_with_my_tools.urdf",
        "description": "Relative or absolute path to robot kinematic URDF model.",
    },

    # ─── 2. 标定参数与标定板 (Calibration Target & Mount) ────────────────────
    {
        "key": "calib.mount",
        "category": "calib",
        "label": "Hand-Eye Mount Mode",
        "type": "select",
        "yaml_path": "calib.mount",
        "default": "eye-to-hand",
        "options": [
            {"value": "eye-to-hand", "label": "Eye-to-Hand (Camera Fixed, Board on Flange)"},
            {"value": "eye-in-hand", "label": "Eye-in-Hand (Camera on Flange, Board Fixed)"},
        ],
        "description": "Default kinematic mount structure for new calibration sessions.",
    },
    {
        "key": "calib.board.rows",
        "category": "calib",
        "label": "Checkerboard Rows",
        "type": "number",
        "yaml_path": "calib.board.rows",
        "default": 12,
        "min": 3,
        "max": 50,
        "step": 1,
        "description": "Number of checkerboard grid rows (external corner count).",
        "legacy_key": "calib_board_rows",
    },
    {
        "key": "calib.board.cols",
        "category": "calib",
        "label": "Checkerboard Columns",
        "type": "number",
        "yaml_path": "calib.board.cols",
        "default": 9,
        "min": 3,
        "max": 50,
        "step": 1,
        "description": "Number of checkerboard grid columns (external corner count).",
        "legacy_key": "calib_board_cols",
    },
    {
        "key": "calib.board.square_size_mm",
        "category": "calib",
        "label": "Checkerboard Square Size (mm)",
        "type": "number",
        "yaml_path": "calib.board.square_size_mm",
        "default": 15.0,
        "min": 1.0,
        "max": 200.0,
        "step": 0.5,
        "description": "Physical size of each grid square in millimeters.",
    },
    {
        "key": "calib.cleaning_threshold",
        "category": "calib",
        "label": "Data Cleaning Threshold",
        "type": "number",
        "yaml_path": "calib.cleaning_threshold",
        "default": 0.05,
        "min": 0.001,
        "max": 0.5,
        "step": 0.005,
        "description": "Outlier filtering threshold for vision vs robot flange displacement deviation.",
    },

    # ─── 3. 喷涂与规划工艺参数 (Spraying & Planning Process) ─────────────────
    {
        "key": "spraying.spray_dist_mm",
        "category": "spraying",
        "label": "Spray Standoff Distance (mm)",
        "type": "number",
        "yaml_path": "spraying.spray_dist_mm",
        "default": 150.0,
        "min": 20.0,
        "max": 1000.0,
        "step": 5.0,
        "description": "Target nozzle-to-workpiece normal standoff distance in millimeters.",
    },
    {
        "key": "spraying.spray_width_mm",
        "category": "spraying",
        "label": "Spray Fan Width (mm)",
        "type": "number",
        "yaml_path": "spraying.spray_width_mm",
        "default": 50.0,
        "min": 5.0,
        "max": 500.0,
        "step": 1.0,
        "description": "Effective fan pattern width of the spray nozzle in millimeters.",
    },
    {
        "key": "spraying.overlap_rate",
        "category": "spraying",
        "label": "Pass Overlap Ratio",
        "type": "number",
        "yaml_path": "spraying.overlap_rate",
        "default": 0.2,
        "min": 0.0,
        "max": 0.8,
        "step": 0.05,
        "description": "Overlap fraction between adjacent spray passes (0.0 to 0.8).",
    },
    {
        "key": "spraying.point_spacing_mm",
        "category": "spraying",
        "label": "Waypoint Spacing (mm)",
        "type": "number",
        "yaml_path": "spraying.point_spacing_mm",
        "default": 100.0,
        "min": 5.0,
        "max": 500.0,
        "step": 5.0,
        "description": "Discretization distance between consecutive trajectory waypoints along each pass.",
    },
    {
        "key": "spraying.velocity",
        "category": "spraying",
        "label": "Spraying Velocity (mm/s)",
        "type": "number",
        "yaml_path": "spraying.velocity",
        "default": 150.0,
        "min": 10.0,
        "max": 2000.0,
        "step": 10.0,
        "description": "Nominal linear execution speed of the TCP during spraying passes.",
    },
    {
        "key": "spraying.poi_anchor_source",
        "category": "spraying",
        "label": "POI Anchor Orientation Source",
        "type": "select",
        "yaml_path": "spraying.poi_anchor_source",
        "default": "config",
        "options": [
            {"value": "config", "label": "Config (Fixed Reference Euler RPY)"},
            {"value": "home", "label": "Home (Robot Home Pose TCP)"},
            {"value": "raw", "label": "Raw (Per-point Surface Normal)"},
        ],
        "description": "Center orientation baseline for POI tolerance envelope constraints.",
    },
    {
        "key": "spraying.poi_ref_rpy_deg",
        "category": "spraying",
        "label": "POI Reference RPY (deg)",
        "type": "vector3",
        "yaml_path": "spraying.poi_ref_rpy_deg",
        "default": [90.0, 0.0, 90.0],
        "description": "Reference Euler RPY [Rx, Ry, Rz] in degrees when anchor source is 'config'.",
    },
    {
        "key": "spraying.poi_tolerance_rpy_deg",
        "category": "spraying",
        "label": "POI Tolerance Envelope [Rx, Ry, Rz] (deg)",
        "type": "vector3",
        "yaml_path": "spraying.poi_tolerance_rpy_deg",
        "default": [30.0, 30.0, 180.0],
        "description": "Permissible angular deviation envelope [Rx, Ry, Rz] around anchor orientation.",
    },
    {
        "key": "spraying.tol_ladder",
        "category": "spraying",
        "label": "Tolerance Ladder Optimization Guard",
        "type": "boolean",
        "yaml_path": "spraying.tol_ladder",
        "default": True,
        "description": "Enforce monotonic ladder optimization passes to prevent high-tolerance J6 velocity spikes.",
    },
    {
        "key": "spraying.tol_ladder_stop_peak_ratio",
        "category": "spraying",
        "label": "Ladder Early Stop Peak Ratio",
        "type": "number",
        "yaml_path": "spraying.tol_ladder_stop_peak_ratio",
        "default": 0.3,
        "min": 0.05,
        "max": 1.0,
        "step": 0.05,
        "description": "Stop ladder tightening when joint velocity peak ratio drops below this threshold.",
    },

    # ─── 4. 视觉识别与交互式分割 (Vision & Interactive SAM) ─────────────────
    {
        "key": "interactive.detector.enabled",
        "category": "interactive",
        "label": "Enable Pre-Detection",
        "type": "boolean",
        "yaml_path": "interactive.detector.enabled",
        "default": True,
        "description": "Run Wissight object detector to initialize bounding box prompts before segmentation.",
    },
    {
        "key": "interactive.detector.sam_refine",
        "category": "interactive",
        "label": "Refine Mask with MobileSAM",
        "type": "boolean",
        "yaml_path": "interactive.detector.sam_refine",
        "default": False,
        "description": "Feed detected bounding box to MobileSAM for high-resolution contour refinement.",
    },
    {
        "key": "interactive.detector.classes",
        "category": "interactive",
        "label": "Detection Target Classes",
        "type": "tags",
        "yaml_path": "interactive.detector.classes",
        "default": ["trousers"],
        "description": "Target class names to filter from detector results (empty = allow all).",
    },
    {
        "key": "interactive.detector.conf",
        "category": "interactive",
        "label": "Detection Confidence Threshold",
        "type": "number",
        "yaml_path": "interactive.detector.conf",
        "default": 0.25,
        "min": 0.01,
        "max": 1.0,
        "step": 0.05,
        "description": "Minimum confidence score for Wissight garment detection.",
    },
    {
        "key": "interactive.detector.iou",
        "category": "interactive",
        "label": "NMS IoU Threshold",
        "type": "number",
        "yaml_path": "interactive.detector.iou",
        "default": 0.7,
        "min": 0.1,
        "max": 1.0,
        "step": 0.05,
        "description": "Non-Maximum Suppression (NMS) intersection-over-union threshold.",
    },
    {
        "key": "interactive.detector.max_boxes",
        "category": "interactive",
        "label": "Max Candidate Boxes",
        "type": "number",
        "yaml_path": "interactive.detector.max_boxes",
        "default": 5,
        "min": 1,
        "max": 50,
        "step": 1,
        "description": "Maximum number of candidate detection bounding boxes returned.",
    },
    {
        "key": "interactive.detector.backend",
        "category": "interactive",
        "label": "Detector Inference Backend",
        "type": "select",
        "yaml_path": "interactive.detector.backend",
        "default": "auto",
        "options": [
            {"value": "auto", "label": "Auto Detect (RKNN > CUDA > ONNX > PyTorch)"},
            {"value": "rknn", "label": "RKNN (Rockchip NPU)"},
            {"value": "onnx", "label": "ONNX Runtime"},
            {"value": "pt", "label": "PyTorch"},
        ],
        "description": "Inference acceleration backend for Wissight detector.",
    },
    {
        "key": "interactive.sam.backend",
        "category": "interactive",
        "label": "MobileSAM Inference Backend",
        "type": "select",
        "yaml_path": "interactive.sam.backend",
        "default": "auto",
        "options": [
            {"value": "auto", "label": "Auto Detect (RKNN > CUDA > ONNX > PyTorch)"},
            {"value": "rknn", "label": "RKNN (Rockchip NPU)"},
            {"value": "onnx", "label": "ONNX Runtime"},
            {"value": "pt", "label": "PyTorch"},
        ],
        "description": "Inference acceleration backend for MobileSAM interactive segmentation.",
    },
]


class SprayerConfig:
    """
    统一读取和解析 AiSprayer 系统配置 (aisprayer_config.yaml) 及相关引用的配置文件。
    单例模式：全局共享同一实例，支持三级梯级读取：
      [1. SQLite 数据库覆盖] -> [2. YAML 配置文件基线] -> [3. 代码默认值]
    """
    _instance = None

    def __new__(cls, config_path="configs/aisprayer_config.yaml", force_reload=False):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, config_path="configs/aisprayer_config.yaml", force_reload=False):
        if getattr(self, "_initialized", False) and not force_reload:
            return
        self.config_path = self._resolve_path(config_path)
        self._db_overrides: Dict[str, Any] = {}
        self.reload()
        self._initialized = True

    def reload(self):
        """重新从磁盘加载 YAML 配置文件与关联标定文件，并同步刷新数据库覆盖项"""
        self.config_data = self._load_yaml(self.config_path)
        
        # 自动加载关联的标定文件 (calibration_result.yaml)
        calib_rel_path = (
            self.config_data.get("spraying", {}).get("calib_path")
            or self.config_data.get("calib", {}).get("result_path")
        )
        self.calib_path = self._resolve_path(calib_rel_path) if calib_rel_path else None
        self.calib_data = self._load_yaml(self.calib_path) if self.calib_path else {}
        self.reload_db_overrides()

    def reload_db_overrides(self):
        """重新从 SQLite 数据库加载动态配置覆盖至内存缓存中 (避免高频轮询 DB I/O)"""
        try:
            from services.setting_service import SettingService
            self._db_overrides = SettingService().get_all_settings()
        except Exception as e:
            logger.warning(f"Failed to reload setting overrides from DB: {e}")
            self._db_overrides = {}

    def _get_yaml_nested(self, path: Optional[str], default: Any = None) -> Any:
        """根据点分路径从 self.config_data 中获取配置值，例如 'hardware.robot.ip'"""
        if not path or not self.config_data:
            return default
        curr = self.config_data
        for part in path.split("."):
            if isinstance(curr, dict) and part in curr:
                curr = curr[part]
            else:
                return default
        return curr

    def get_cascading(self, db_key: str, yaml_path: Optional[str] = None, default: Any = None) -> Any:
        """
        三级梯级读取机制：
        1. 优先从内存中的 SQLite 动态配置 (_db_overrides) 获取；
        2. 若不存在，检查兼容历史别名 (legacy aliases)；
        3. 若不存在，从 YAML 配置文件 (aisprayer_config.yaml) 中获取；
        4. 最终回退至代码默认值 (default)。
        """
        if db_key in self._db_overrides:
            return self._db_overrides[db_key]
        for alias in _LEGACY_KEY_ALIASES.get(db_key, []):
            if alias in self._db_overrides:
                return self._db_overrides[alias]
        if yaml_path:
            yaml_val = self._get_yaml_nested(yaml_path)
            if yaml_val is not None:
                return yaml_val
        return default

    def get_config_metadata(self) -> List[Dict[str, Any]]:
        """返回所有受管配置项的完整元数据定义与当前生效值、默认值及覆盖状态"""
        metadata_list = []
        for reg in CONFIG_REGISTRY:
            key = reg["key"]
            yaml_path = reg.get("yaml_path")
            default_val = reg.get("default")
            legacy_key = reg.get("legacy_key")
            
            yaml_val = self._get_yaml_nested(yaml_path, default_val) if yaml_path else default_val
            is_overridden = key in self._db_overrides or (legacy_key is not None and legacy_key in self._db_overrides)
            effective_val = self.get_cascading(key, yaml_path, default_val)
            
            item = {
                "key": key,
                "category": reg["category"],
                "label": reg["label"],
                "type": reg["type"],
                "value": effective_val,
                "yaml_default": yaml_val,
                "is_overridden": bool(is_overridden),
                "description": reg.get("description", ""),
            }
            if "min" in reg:
                item["min"] = reg["min"]
            if "max" in reg:
                item["max"] = reg["max"]
            if "step" in reg:
                item["step"] = reg["step"]
            if "options" in reg:
                item["options"] = reg["options"]
            if legacy_key:
                item["legacy_key"] = legacy_key
            metadata_list.append(item)
        return metadata_list

    def _resolve_path(self, path):
        if not path:
            return None
        if os.path.isabs(path):
            return path
        return os.path.join(PROJECT_ROOT, path)

    def _load_yaml(self, path):
        if not path or not os.path.exists(path):
            return {}
        try:
            with open(path, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
        except Exception as e:
            logger.warning(f"Failed to load yaml config from {path}: {e}")
            return {}

    @property
    def hand_eye_mount(self):
        """
        当前标定结果对应的相机安装方式: 'eye-to-hand' 或 'eye-in-hand'。

        历史结果文件没写这个字段 (或写的是旧的 calibration_mode), 一律按眼在手外
        处理 —— 那是本项目此前唯一支持的装法。
        """
        if not self.calib_data:
            return self.calib_mount
        meta = self.calib_data.get("metadata", {}) or {}
        mount = (self.calib_data.get("hand_eye_mount")
                 or meta.get("hand_eye_mount")
                 or meta.get("calibration_mode"))
        if mount:
            return "eye-in-hand" if mount == "eye-in-hand" else "eye-to-hand"
        return self.calib_mount

    @property
    def T_flange_camera(self):
        """眼在手上标定的相机安装外参 (4x4 列表, 平移 mm); 眼在手外时为 None。"""
        if not self.calib_data:
            return None
        return self.calib_data.get("T_flange_camera")

    def T_camera_to_base_at(self, base_flange_pose):
        """
        指定法兰位姿下相机到基座的变换 (4x4 列表, 平移 m)。

        眼在手外: 与法兰无关, 直接返回标定的常量外参。
        眼在手上: T_base_camera = T_base_flange(pose) · T_flange_camera, 每次拍摄都不同。

        :param base_flange_pose: [x, y, z, rx, ry, rz], 平移 mm, 姿态度 (Dobot 'xyz' 内禀序列)
        """
        if self.hand_eye_mount != "eye-in-hand":
            return self.T_camera_to_base
        if base_flange_pose is None or len(base_flange_pose) < 6:
            logger.warning("eye-in-hand calibration needs a base_flange_pose to resolve camera extrinsics")
            return None
        if not self.T_flange_camera:
            logger.warning("Calibration result is eye-in-hand but T_flange_camera is missing")
            return None

        import numpy as np
        from core.handeye import pose_to_matrix

        T_base_flange = pose_to_matrix(base_flange_pose)
        T = T_base_flange @ np.array(self.T_flange_camera, dtype=float)
        T[:3, 3] /= 1000.0  # mm -> m, 与 T_camera_to_base 一致
        return T.tolist()

    @property
    def T_camera_to_base(self):
        """手眼标定矩阵 (4x4 列表，平移部分被自动转换为米)。眼在手上时为 None, 改用 T_camera_to_base_at。"""
        if self.calib_data:
            key = 'T_base_camera' if 'T_base_camera' in self.calib_data else ('T_camera_to_base' if 'T_camera_to_base' in self.calib_data else None)
            if key:
                import copy
                T = copy.deepcopy(self.calib_data[key])
                # 标定文件中的平移部分是以毫米为单位保存的 (例如 847.1)
                # 但后续 3D 处理流水线 (点云/URDF/规划) 均使用米 (m)
                # 故在此处统一将平移部分缩放为米
                T[0][3] /= 1000.0
                T[1][3] /= 1000.0
                T[2][3] /= 1000.0
                return T
            if self.hand_eye_mount == "eye-in-hand" and not getattr(self, "_warned_eye_in_hand", False):
                # 眼在手上时相机在基座系的位姿不是常量, 返回一个错误的常量比返回 None 危险得多
                self._warned_eye_in_hand = True
                logger.warning(
                    "Active calibration is eye-in-hand: T_camera_to_base is not constant, "
                    "use T_camera_to_base_at(base_flange_pose)"
                )
        return None

    @property
    def model_path(self):
        """YOLO 分割模型路径 (自动解析为绝对路径)。"""
        path = self.config_data.get("spraying", {}).get("model_path")
        return self._resolve_path(path)

    @property
    def output_root(self):
        """生产运行数据存储根目录 (例如 data/runs) (自动解析为绝对路径)。"""
        path = self.config_data.get("spraying", {}).get("output_root", "data/runs")
        return self._resolve_path(path)

    @property
    def spray_width_mm(self) -> float:
        """喷涂幅宽 (mm, 默认 50.0)"""
        return float(self.get_cascading("spraying.spray_width_mm", "spraying.spray_width_mm", 50.0))

    @property
    def spray_width(self) -> float:
        """喷涂幅宽 (返回单位: 米)"""
        return self.spray_width_mm / 1000.0

    @property
    def spray_distance_mm(self) -> float:
        """默认喷涂靶距 / TCP standoff 距离 (mm, 默认 150.0)"""
        val = self.get_cascading("spraying.spray_dist_mm", "spraying.spray_dist_mm", None)
        if val is None:
            val = self._get_yaml_nested("spraying.spray_distance_mm", 150.0)
        return float(val)

    @property
    def spray_distance(self) -> float:
        """喷涂距离 (返回单位: 米)"""
        return self.spray_distance_mm / 1000.0

    @property
    def standoff_distance_mm(self) -> float:
        """TCP standoff 距离别名 (mm)"""
        return self.spray_distance_mm

    @property
    def overlap_rate(self) -> float:
        """喷幅重叠率 (0~1.0, 默认 0.2)"""
        return float(self.get_cascading("spraying.overlap_rate", "spraying.overlap_rate", 0.2))

    @property
    def row_spacing_mm(self) -> float:
        """自动规划行间距 (mm, 默认根据 spray_width_mm * (1 - overlap_rate) 计算)"""
        spraying_cfg = self.config_data.get("spraying", {})
        if "row_spacing_mm" in spraying_cfg and spraying_cfg["row_spacing_mm"]:
            return float(spraying_cfg["row_spacing_mm"])
        return self.spray_width_mm * (1.0 - self.overlap_rate)

    @property
    def point_spacing_mm(self) -> float:
        """自动规划沿行点间距 (mm, 默认 100.0)"""
        val = self.get_cascading("spraying.point_spacing_mm", "spraying.point_spacing_mm", None)
        if val is not None:
            return float(val)
        v_step = self._get_yaml_nested("spraying.v_step_mm")
        if v_step is not None:
            return float(v_step)
        return 100.0

    @property
    def spraying_velocity(self) -> float:
        """喷涂移动速度 (mm/s, 默认 150.0)"""
        return float(self.get_cascading("spraying.velocity", "spraying.velocity", 150.0))

    @property
    def slerp_step_mm(self) -> float:
        """轨迹验证与仿真插值步长 (mm, 默认 2.0)"""
        val = self.get_cascading("spraying.slerp_step_mm", "spraying.slerp_step_mm", 2.0)
        return float(val)

    @property
    def urdf_path(self) -> str:
        """机器人 URDF 模型文件路径 (绝对路径)"""
        path = self.get_cascading("robot.robot_urdf", "hardware.robot.robot_urdf", "app/urdf/cr5_robot_with_my_tools.urdf")
        return self._resolve_path(path)

    @property
    def robot_urdf(self) -> str:
        """机器人 URDF 模型文件路径别名 (绝对路径)"""
        return self.urdf_path

    @property
    def robot_ip(self) -> str:
        """机器人控制器 IP 地址"""
        return str(self.get_cascading("robot.ip", "hardware.robot.ip", "192.168.5.1"))

    @property
    def robot_port(self) -> int:
        """机器人控制端口 (Dobot 默认 29999)"""
        return int(self.get_cascading("robot.port", "hardware.robot.port", 29999))

    @property
    def robot_tcp_id(self) -> int:
        """
        机械臂末端工具坐标系 ID (0: 默认法兰/工具0, 1: gripper_tip_link, 2: laser_head_link)。
        """
        val = self.get_cascading("robot.robot_tcp_id", "hardware.robot.robot_tcp_id", None)
        if val is not None:
            try:
                return int(val)
            except (ValueError, TypeError):
                pass
        # 若未显式配置 robot_tcp_id，则根据 robot_tcp 名称自动推导
        tcp_name = str(self.robot_tcp).lower()
        if any(k in tcp_name for k in ["grip", "finger", "tip"]):
            return 1
        elif any(k in tcp_name for k in ["laser", "nozzle", "spray", "gun"]):
            return 2
        return 0

    @property
    def robot_tcp(self) -> str:
        """机器人末端工具 TCP 节点名称 (例如 laser_head_link, gripper_tip_link)"""
        val = self.get_cascading("robot.robot_tcp", "hardware.robot.robot_tcp", None)
        if val:
            return str(val).strip()
        # 若未显式指定名称，则根据 robot_tcp_id 映射
        tcp_id = self.robot_tcp_id
        if tcp_id == 1:
            return "gripper_tip_link"
        elif tcp_id == 2:
            return "laser_head_link"
        return "gripper_tip_link"

    @property
    def spray_do_index(self) -> int:
        """
        机械臂喷涂开关数字输出端口 (DO) 编号 (1-based, 取值范围 1-16, 默认 1)。
        """
        val = self.get_cascading("robot.spray_do_index", "hardware.robot.spray_do_index", 1)
        try:
            index = int(val)
            if 1 <= index <= 16:
                return index
        except (ValueError, TypeError):
            pass
        return 1

    @property
    def calib_mount(self) -> str:
        """新建标定会话默认相机安装方式: 'eye-to-hand' 或 'eye-in-hand'"""
        val = str(self.get_cascading("calib.mount", "calib.mount", "eye-to-hand")).strip().lower()
        return "eye-in-hand" if val == "eye-in-hand" else "eye-to-hand"

    @property
    def calib_board_cols(self) -> int:
        """标定板列数 (节点数)"""
        return int(self.get_cascading("calib.board.cols", "calib.board.cols", 9))

    @property
    def calib_board_rows(self) -> int:
        """标定板行数 (节点数)"""
        return int(self.get_cascading("calib.board.rows", "calib.board.rows", 12))

    @property
    def calib_board_square_size_mm(self) -> float:
        """标定板方格物理尺寸 (mm)"""
        return float(self.get_cascading("calib.board.square_size_mm", "calib.board.square_size_mm", 15.0))

    @property
    def calib_cleaning_threshold(self) -> float:
        """标定数据清洗偏差阈值 (比例)"""
        return float(self.get_cascading("calib.cleaning_threshold", "calib.cleaning_threshold", 0.05))

    @property
    def camera_model(self) -> str:
        return str(self._get_yaml_nested("hardware.camera.model", "orbbec"))

    @property
    def global_speed_factor(self) -> int:
        """示教/远程/运行全局速度百分比 (1-100%, 默认 50)"""
        val = self.get_cascading("robot.global_speed_factor", "hardware.robot.global_speed_factor", None)
        if val is None:
            val = self._get_yaml_nested("hardware.robot.global_speed_percent", 50)
        return int(val)

    @property
    def global_speed_percent(self) -> int:
        """全局速度百分比（兼容别名）"""
        return self.global_speed_factor

    @property
    def max_tcp_speed_mm_s(self) -> float:
        """机器人最大末端 TCP 线速度 (mm/s, 默认 2000.0)"""
        val = self._get_yaml_nested("hardware.robot.max_tcp_speed_mm_s", 2000.0)
        return float(val)

    @property
    def max_joint_speed_deg_s(self) -> List[float]:
        """机器人最大关节速度 (度/s, 6轴列表, 默认 [180, 180, 180, 180, 180, 180])"""
        speeds = self._get_yaml_nested("hardware.robot.max_joint_speed_deg_s", [180.0, 180.0, 180.0, 180.0, 180.0, 180.0])
        return [float(x) for x in speeds]

    @property
    def poi_tolerance_rpy_deg(self) -> List[float]:
        """POI 锚点姿态容差包络 [Rx, Ry, Rz] (度)"""
        val = self.get_cascading("spraying.poi_tolerance_rpy_deg", "spraying.poi_tolerance_rpy_deg", None)
        if val is None:
            val = self._get_yaml_nested("optimization.poi_tolerance_rpy_deg", [30.0, 30.0, 180.0])
        return [float(v) for v in val]

    @property
    def poi_anchor_source(self) -> str:
        """POI 锚点(容差包络中心)来源: 'config' | 'home' | 'raw'"""
        val = self.get_cascading("spraying.poi_anchor_source", "spraying.poi_anchor_source", None)
        if val is None:
            val = self._get_yaml_nested("optimization.poi_anchor_source", "config")
        src = str(val).strip().lower()
        return src if src in {"config", "home", "raw"} else "config"

    @property
    def poi_ref_rpy_deg(self) -> Optional[List[float]]:
        """POI 锚点参考姿态 [Rx, Ry, Rz] (度, Euler 'xyz'); 未配置则返回 None"""
        val = self.get_cascading("spraying.poi_ref_rpy_deg", "spraying.poi_ref_rpy_deg", None)
        if val is None:
            val = self._get_yaml_nested("optimization.poi_ref_rpy_deg", None)
        if not val or len(val) != 3:
            return None
        return [float(v) for v in val]

    @property
    def tol_ladder(self) -> bool:
        """容差阶梯择优 (Monotonicity Guard) 开关"""
        return bool(self.get_cascading("spraying.tol_ladder", "spraying.tol_ladder", True))

    @property
    def tol_ladder_stop_peak_ratio(self) -> float:
        """容差阶梯早停阈值比例"""
        return float(self.get_cascading("spraying.tol_ladder_stop_peak_ratio", "spraying.tol_ladder_stop_peak_ratio", 0.3))

    @property
    def grid_tol_x_deg(self) -> Tuple[float, float, float]:
        """轨迹优化器 X 轴搜索网格 (min, max, step) (度)"""
        val = self._get_yaml_nested("spraying.grid_tol_x_deg") or self._get_yaml_nested("optimization.grid_tol_x_deg", [-5.0, 5.0, 2.0])
        return tuple(float(v) for v in val)

    @property
    def grid_tol_y_deg(self) -> Tuple[float, float, float]:
        """轨迹优化器 Y 轴搜索网格 (min, max, step) (度)"""
        val = self._get_yaml_nested("spraying.grid_tol_y_deg") or self._get_yaml_nested("optimization.grid_tol_y_deg", [-5.0, 5.0, 2.0])
        return tuple(float(v) for v in val)

    @property
    def grid_tol_z_deg(self) -> Tuple[float, float, float]:
        """轨迹优化器 Z 轴搜索网格 (min, max, step) (度)"""
        val = self._get_yaml_nested("spraying.grid_tol_z_deg") or self._get_yaml_nested("optimization.grid_tol_z_deg", [-30.0, 30.0, 5.0])
        return tuple(float(v) for v in val)

    # ─── 交互式分割与自动检测 (interactive.*) ────────────────────────────────
    @property
    def sam_backend(self) -> str:
        """MobileSAM 推理后端：auto | rknn | onnx | pt。"""
        return str(self.get_cascading("interactive.sam.backend", "interactive.sam.backend", "auto") or "auto").strip()

    @property
    def detector_enabled(self) -> bool:
        """进入交互分割时是否先跑目标检测出框（false = 维持纯手动点选行为）。"""
        return bool(self.get_cascading("interactive.detector.enabled", "interactive.detector.enabled", True))

    @property
    def detector_backend(self) -> str:
        """Wissight 推理后端：auto | rknn | onnx | pt。"""
        return str(self.get_cascading("interactive.detector.backend", "interactive.detector.backend", "auto") or "auto").strip()

    @property
    def detector_classes(self) -> List[str]:
        """允许当成 SAM prompt 的类别名；空列表 = 不按类别过滤。"""
        classes = self.get_cascading("interactive.detector.classes", "interactive.detector.classes", ["trousers"])
        return [str(c).strip() for c in classes] if classes else []

    @property
    def detector_conf(self) -> float:
        """检测置信度阈值。"""
        return float(self.get_cascading("interactive.detector.conf", "interactive.detector.conf", 0.25))

    @property
    def detector_iou(self) -> float:
        """NMS IoU 阈值。"""
        return float(self.get_cascading("interactive.detector.iou", "interactive.detector.iou", 0.7))

    @property
    def detector_max_boxes(self) -> int:
        """接口最多回传几个候选框。"""
        return int(self.get_cascading("interactive.detector.max_boxes", "interactive.detector.max_boxes", 5))

    @property
    def detector_sam_refine(self) -> bool:
        """检到目标后是否再用 MobileSAM 精修。"""
        return bool(self.get_cascading("interactive.detector.sam_refine", "interactive.detector.sam_refine", False))


# ─── 全局单例对象 (模块导入时完成初始化与加载) ──────────────────────────────────
config = SprayerConfig()
sprayer_config = config


def get_config() -> SprayerConfig:
    """获取全局配置单例对象"""
    return config


def get_configured_robot_config(config_path: str = None) -> tuple[str, str]:
    """统一从全局配置获取 (urdf_abs_path, tcp_target_link)。"""
    cfg = config if config_path is None else SprayerConfig(config_path=config_path)
    return cfg.robot_urdf, cfg.robot_tcp


def load_tcp_from_urdf(urdf_path: str = None, target_tcp_name: str = None) -> dict:
    """从 URDF 解析挂在 Link6/法兰上的工具 TCP（毫米 / 度），给 Web 回显用。"""
    if urdf_path is None or target_tcp_name is None:
        cfg_urdf, cfg_tcp = get_configured_robot_config()
        if urdf_path is None:
            urdf_path = cfg_urdf
        if target_tcp_name is None:
            target_tcp_name = cfg_tcp

    tcp_info = {
        "has_tool": False,
        "tool_name": "flange",
        "xyz_mm": [0.0, 0.0, 0.0],
        "rpy_deg": [0.0, 0.0, 0.0],
        "urdf_source": os.path.basename(urdf_path) if urdf_path else None,
    }
    if not urdf_path or not os.path.exists(urdf_path):
        return tcp_info

    try:
        root = ET.parse(urdf_path).getroot()
        best_score = -1
        for joint in root.findall("joint"):
            parent = joint.find("parent")
            child = joint.find("child")
            if parent is None or parent.get("link") not in ["Link6", "link6", "flange"]:
                continue
            origin = joint.find("origin")
            if origin is None:
                continue
            child_name = child.get("link", "") if child is not None else ""
            xyz_m = [float(v) for v in origin.get("xyz", "0 0 0").split()]
            rpy_rad = [float(v) for v in origin.get("rpy", "0 0 0").split()]
            xyz_mm = [round(v * 1000.0, 2) for v in xyz_m]
            rpy_deg = [round(math.degrees(v), 2) for v in rpy_rad]
            score = 0
            if target_tcp_name and (
                child_name.lower() == target_tcp_name.lower()
                or target_tcp_name.lower() in child_name.lower()
            ):
                score = 1000
            elif any(k in child_name.lower() for k in ["laser", "nozzle", "tcp"]):
                score = 100
            elif "tip" in child_name.lower():
                score = 80
            elif "gun" in child_name.lower():
                score = 50
            elif "tool" in child_name.lower():
                score = 30
            if score > best_score:
                best_score = score
                tcp_info = {
                    "has_tool": True,
                    "tool_name": child_name,
                    "xyz_mm": xyz_mm,
                    "rpy_deg": rpy_deg,
                    "urdf_source": os.path.basename(urdf_path),
                }
    except Exception as e:
        logger.warning("Could not parse TCP from URDF %s: %s", urdf_path, e)
    return tcp_info


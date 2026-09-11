# AiSprayer 系统配置重构方案 (System Configuration Refactoring Design)

> **文档状态**：方案设计阶段（Architecture Design）  
> **文档位置**：`app/docs/system_config_refactoring_design.md`  
> **涉及核心文件**：
> - 静态基座配置：[`configs/aisprayer_config.yaml`](file:///Users/carl/robots/AiSprayer/configs/aisprayer_config.yaml)
> - 后端配置内核：[`app/src/core/config.py`](file:///Users/carl/robots/AiSprayer/app/src/core/config.py)
> - SQLite 数据模型与服务：[`app/src/db/models.py`](file:///Users/carl/robots/AiSprayer/app/src/db/models.py)、[`app/src/services/setting_service.py`](file:///Users/carl/robots/AiSprayer/app/src/services/setting_service.py)
> - 系统配置 API：[`app/src/apps/system/api.py`](file:///Users/carl/robots/AiSprayer/app/src/apps/system/api.py)
> - 前端配置管理视图：[`app/frontend/src/views/ConfigView.tsx`](file:///Users/carl/robots/AiSprayer/app/frontend/src/views/ConfigView.tsx)  
> *(注：按照需求，工位跟随 `follow` 模块的独立配置保持现状，暂不纳入本次重构范围)*

---

## 一、背景与现存问题分析 (Background & Problem Statement)

### 1. 当前现状与架构断层 (Architectural Disconnect)
目前 AiSprayer 存在 **两套独立且割裂** 的配置体系：

1. **静态 YAML 文件 (`configs/aisprayer_config.yaml`)**：
   - 涵盖了硬件（相机、机械臂）、标定棋盘格、喷涂工艺、POI 姿态优化及 MobileSAM/Wissight 视觉算法等几乎所有业务参数；
   - 后端核心服务主要通过 [`SprayerConfig`](file:///Users/carl/robots/AiSprayer/app/src/core/config.py) 单例以只读方式直接从磁盘加载该 YAML；
   - **痛点**：现场工艺调试、更换标定板、调整喷涂幅宽靶距或切换工具 TCP 时，现场操作员必须通过 SSH 或终端手动修改 YAML 文件，无法在 Web 界面便捷调参。

2. **前端界面与 SQLite 数据库 (`data/aisprayer.db`)**：
   - 界面上存在 `System Configuration`（[`ConfigView.tsx`](file:///Users/carl/robots/AiSprayer/app/frontend/src/views/ConfigView.tsx)）页面；
   - 后端通过 `SettingService`（[`sys_settings` 表](file:///Users/carl/robots/AiSprayer/app/src/db/models.py#L5)）读写 SQLite；
   - **痛点**：目前页面**仅仅硬编码了 4 个字段**（`robot_ip`, `robot_port`, `calib_board_cols`, `calib_board_rows`）。更严重的是，除 `robot_ip` 和 `robot_port` 在连接时被读取外，**其余配置均未接入核心计算管道**，页面修改后后端计算仍直接吃 YAML 中的初始值，修改无法真正落地生效。

### 2. 重构目标 (Refactoring Goals)
1. **三层级联配置引擎 (3-Tier Cascading Engine)**：构建“**运行时参数 > SQLite 动态覆盖 > YAML 生产基线 > 默认硬编码**”的统一优先级访问机制；
2. **动静分离 (Dynamic vs. Static Decoupling)**：严谨梳理全量配置项，基础设施保留在 YAML，工艺、硬件连接与算法业务参数沉淀为 SQLite 动态项；
3. **内存热缓存与变更通知 (In-Memory Hot-Reload & Broadcast)**：消除每次 10Hz 轮询对 SQLite 的高频 I/O 损耗，界面保存时主动热刷新后端服务内存，即改即生效；
4. **全英文工业级界面升级 (Strict English-Only Dynamic UI)**：扩展 `ConfigView.tsx`，按功能模块组织成整洁直观的配置卡片，支持重置为默认值与输入范围防御。

---

## 二、配置项动静分类与判定矩阵 (Configuration Classification Matrix)

> 判定标准：
> - **动态配置 (Dynamic -> SQLite)**：现场部署调整频繁、工艺工程师常用调参、换型换件（如更换标定板、切换末端工件、调喷涂靶距、目标服装品类筛选等）。
> - **静态基座配置 (Static -> YAML)**：系统网络监听端口、C++ 独立守护进程启动参数、底层驱动库依赖、操作系统路径协议规范等。

### 1. 硬件层配置 (`hardware`)

| YAML 键路径 | SQLite 存储键 | 类型 | 判定分类 | 推荐 UI 控件 | 详细说明与生效方式 |
| :--- | :--- | :---: | :---: | :---: | :--- |
| `hardware.robot.ip` | `robot.ip` | `str` | **动态** | `TextInput` | 机械臂控制器 IP。重新点击 Connect 时即时生效。 |
| `hardware.robot.port` | `robot.port` | `int` | **动态** | `NumberInput` | 控制端口（Dobot: 29999, Inexbot: 6001）。重新连接生效。 |
| `hardware.robot.type` | `robot.type` | `str` | **动态** | `Select` | 机器人品牌型号（`dobot` / `inexbot`）。重新连接生效。 |
| `hardware.robot.global_speed_factor`| `robot.global_speed_factor` | `int` | **动态** | `Slider (1-100)` | 机械臂全局安全速度比例（%）。保存时直接触发底层驱动 `set_global_speed`。 |
| `hardware.robot.robot_tcp_id` | `robot.tcp_id` | `int` | **动态** | `Select` | 末端工具坐标系 ID（`0`: 法兰, `1`: 夹爪 `gripper_tip_link`, `2`: 喷枪/激光 `laser_head_link`）。保存时热同步驱动与 3D 渲染。 |
| `hardware.robot.robot_tcp` | `robot.tcp_name` | `str` | **动态** | `Select` | 对应的 URDF 坐标系链接名称。与 `tcp_id` 联动选择。 |
| `hardware.robot.spray_do_index` | `robot.spray_do_index` | `int` | **动态** | `Select (1-16)` | 喷涂电磁阀对应的数字输出 (DO) 端子号（默认 1）。保存时立即热更新 `robot_service.spray_do_index`。 |
| `hardware.robot.robot_urdf` | `robot.urdf_file` | `str` | **动态** | `Select` | 机械臂 3D 模型文件（可选标准 CR5、带枪 CR5 或带夹爪 CR5）。更新后 3D Viewer 重新载入。 |
| `hardware.robot.max_tcp_speed_mm_s` | - | `float`| **静态** | - | 硬件线速度物理极限（2000 mm/s）。CR5 机器物理参数，保留在 YAML。 |
| `hardware.robot.max_joint_speed_deg_s`| - | `list` | **静态** | - | 硬件各轴最大角速度（180 deg/s）。安全边界，保留在 YAML。 |
| `hardware.camera.model` | - | `str` | **静态** | - | 相机硬件型号（`orbbec`）。关联 C++ 驱动加载，保留在 YAML。 |
| `hardware.camera.server.*` | - | `int/str`| **静态** | - | 微服务 REST 端口 (`18080`)、流媒体端口 (`8008`, `8554`, `1935`)。涉及外部网络路由，保留在 YAML。 |
| `hardware.camera.streaming.*` | - | `int` | **静态** | - | H.264 视频硬编码分辨率与码率。C++ 底层管线，保留在 YAML。 |

---

### 2. 手眼标定配置 (`calib`)

| YAML 键路径 | SQLite 存储键 | 类型 | 判定分类 | 推荐 UI 控件 | 详细说明与生效方式 |
| :--- | :--- | :---: | :---: | :---: | :--- |
| `calib.mount` | `calib.mount` | `str` | **动态** | `Segmented` | 默认手眼安装模式（`eye-to-hand` 眼在手外 / `eye-in-hand` 眼在手上）。新建标定会话时的默认模式。 |
| `calib.board.cols` | `calib.board_cols` | `int` | **动态** | `NumberInput` | 标定板宽度方格数（默认 9）。新建检测或标定时热生效。 |
| `calib.board.rows` | `calib.board_rows` | `int` | **动态** | `NumberInput` | 标定板高度方格数（默认 12）。新建检测或标定时热生效。 |
| `calib.board.square_size_mm` | `calib.board_square_size_mm` | `float` | **动态** | `NumberInput` | 棋盘格物理格子边长（mm，默认 15.0）。标定解算时直接影响米制真值，非常关键。 |
| `calib.cleaning_threshold` | `calib.cleaning_threshold` | `float` | **动态** | `NumberInput` | 视觉位移与机械臂位移校验偏差比例阈值（默认 0.05 即 ±5%）。数据清洗时使用。 |
| `calib.capture.output_dir` | - | `str` | **静态** | - | 标定样本落盘存储目录（`data/calib`）。文件系统结构，保留在 YAML。 |
| `calib.result_path` | - | `str` | **静态** | - | 标定外参矩阵持久化 YAML 路径。系统间数据契约，保留在 YAML。 |
| `calib.root_points_path` | - | `str` | **静态** | - | 安全点位数据库路径。保留在 YAML。 |

---

### 3. 喷涂与轨迹规划工艺参数 (`spraying`)

| YAML 键路径 | SQLite 存储键 | 类型 | 判定分类 | 推荐 UI 控件 | 详细说明与生效方式 |
| :--- | :--- | :---: | :---: | :---: | :--- |
| `spraying.spray_dist_mm` | `spraying.dist_mm` | `float` | **动态** | `NumberInput` | 喷枪与工件表面的基准靶距 / TCP standoff（mm，默认 150.0）。路径生成时直接使用。 |
| `spraying.spray_width_mm` | `spraying.width_mm` | `float` | **动态** | `NumberInput` | 喷枪有效喷幅直径/宽度（mm，默认 50.0）。与重叠率共同决定栅格规划行距。 |
| `spraying.overlap_rate` | `spraying.overlap_rate` | `float` | **动态** | `Slider (0-0.8)` | 相邻栅格行间重叠率（0~1.0，默认 0.2 即 20%）。动态调整覆盖密度。 |
| `spraying.point_spacing_mm` | `spraying.point_spacing_mm` | `float` | **动态** | `NumberInput` | 规划路径沿行各航点的密集度步长（mm，默认 100.0）。影响轨迹平滑度。 |
| `spraying.velocity` | `spraying.velocity` | `float` | **动态** | `NumberInput` | 喷涂过程工作移动线速度（mm/s，默认 150.0）。决定喷枪扫过涂层的厚度。 |
| `spraying.poi_anchor_source` | `spraying.poi_anchor_source` | `str` | **动态** | `Select` | 容差包络中心锚点源（`config` 固定欧拉角 / `home` 位姿 / `raw` 沿面逐点法向）。 |
| `spraying.poi_ref_rpy_deg` | `spraying.poi_ref_rpy_deg` | `list` | **动态** | `3x NumberInput` | 固定锚点参考姿态 `[Rx, Ry, Rz]`（度）。当锚点源为 `config` 时生效。 |
| `spraying.poi_tolerance_rpy_deg` | `spraying.poi_tolerance_rpy` | `list` | **动态** | `3x NumberInput` | 姿态自由度容差包络 `[±Rx, ±Ry, ±Rz]`（度，如 `[30, 30, 180]`）。 |
| `spraying.tol_ladder` | `spraying.tol_ladder` | `bool` | **动态** | `Switch` | 是否开启容差阶梯收紧（Monotonicity Guard，大容差自适应最优解）。 |
| `spraying.tol_ladder_stop_peak_ratio` | `spraying.tol_ladder_stop_ratio`| `float` | **动态** | `NumberInput` | 阶梯收紧早停比率阈值（默认 0.3 即峰值限速 ≤ 30% 提前收敛）。 |
| `spraying.output_root` | - | `str` | **静态** | - | 运行工件点云与生产历史保存路径（`data/runs`）。保留在 YAML。 |
| `spraying.slerp_step_mm` | - | `float`| **静态** | - | 运动学验证与四元数球面插值分段步长（默认 2.0mm）。核心算法常数，保留在 YAML。 |
| `spraying.grid_tol_*` | - | `list` | **静态** | - | 逆运动学搜索网格粒度。算法底层收敛参数，避免过度暴露，保留在 YAML。 |

---

### 4. 交互式语义分割与目标检测 (`interactive`)

| YAML 键路径 | SQLite 存储键 | 类型 | 判定分类 | 推荐 UI 控件 | 详细说明与生效方式 |
| :--- | :--- | :---: | :---: | :---: | :--- |
| `interactive.detector.enabled` | `interactive.detector_enabled` | `bool` | **动态** | `Switch` | 是否开启自动目标检测。关闭后交互式作业维持纯人工点击模式。 |
| `interactive.detector.sam_refine` | `interactive.sam_refine` | `bool` | **动态** | `Switch` | 检出目标后是否交由 MobileSAM 进行边缘高精度精修（关闭则直出 YOLO mask，大幅提速）。 |
| `interactive.detector.classes` | `interactive.detector_classes` | `list` | **动态** | `MultiSelect` | 目标检测有效类别筛选（如 `["trousers", "skirt", "short_sleeved_shirt"]`）。换型关键参数。 |
| `interactive.detector.conf` | `interactive.detector_conf` | `float` | **动态** | `Slider (0.05-0.9)` | 检测置信度过滤阈值（默认 0.25）。过滤背景杂质或提升召回率。 |
| `interactive.detector.iou` | `interactive.detector_iou` | `float` | **动态** | `Slider (0.1-0.9)` | NMS 重叠抑制阈值（默认 0.7）。 |
| `interactive.detector.max_boxes` | `interactive.max_boxes` | `int` | **动态** | `NumberInput` | 单张图最多回传的目标候选框数量（默认 5）。 |
| `interactive.detector.backend` | `interactive.detector_backend` | `str` | **动态** | `Select` | Wissight 推理后端（`auto`, `onnx`, `rknn`, `pt`）。本地调测常用。 |
| `interactive.sam.backend` | `interactive.sam_backend` | `str` | **动态** | `Select` | MobileSAM 推理后端（`auto`, `onnx`, `rknn`, `mps`, `cuda`, `cpu`）。 |

---

## 三、三层级联配置引擎架构设计 (Cascading Config Architecture)

```
                       [ Web UI: System Config Page ]
                                     │  (HTTP REST)
                                     ▼
                       [ POST /api/system/config ]
                                     │
                     ┌───────────────┴───────────────┐
                     ▼                               ▼
        [ Write SQLite: sys_settings ]    [ Invalidate Memory Cache ]
                     │                               │
                     └───────────────┬───────────────┘
                                     ▼
        ┌─────────────────────────────────────────────────────────┐
        │        SprayerConfig (统一配置接入层 / Core Engine)        │
        │                                                         │
        │  Priority 1: 内存快速热缓存 (Dirty Flag / Thread-Safe Cache)│
        │  Priority 2: SQLite sys_settings (用户动态覆盖值)         │
        │  Priority 3: aisprayer_config.yaml (出厂静态默认基座)       │
        │  Priority 4: Python Code Fallback Defaults (硬编码安全兜底) │
        └────────────────────────────┬────────────────────────────┘
                                     │
          ┌──────────────────────────┼──────────────────────────┐
          ▼                          ▼                          ▼
   [ RobotService ]         [ CalibrationService ]      [ Vision & Planning ]
   • 连接参数 (IP/Port)      • 棋盘格规格 (Rows/Cols)     • 喷涂靶距与幅宽 (Dist/Width)
   • 工具坐标 (TCP ID/Link)  • 格子边长 (SquareSize)     • 容差包络 (POI Tolerance)
   • 喷涂 DO 端口            • 安装模式 (Mount Mode)     • 目标检测类别与阈值 (Classes/Conf)
```

### 1. 核心级联访问机制 (Cascading Lookup Logic)
在 [`SprayerConfig`](file:///Users/carl/robots/AiSprayer/app/src/core/config.py) 中，所有 Property 改造为统一的级联查询宏或辅助函数：

```python
def get_cascading(self, key: str, yaml_path: tuple, default: Any = None) -> Any:
    """
    统一三级配置查询：
    1. 先查 SQLite 动态覆盖表 (sys_settings)；
    2. 若未覆盖，则查 aisprayer_config.yaml 的对应节点；
    3. 若 YAML 缺失，则使用代码兜底 default。
    """
    # 1. 读内存缓存中的 SQLite 覆盖值
    if key in self._db_overrides:
        return self._db_overrides[key]

    # 2. 读 YAML 节点
    curr = self.config_data
    for p in yaml_path:
        if isinstance(curr, dict) and p in curr:
            curr = curr[p]
        else:
            return default
    return curr if curr is not None else default
```

### 2. 避免高频 I/O 的内存缓存策略
- 机械臂反馈轮询（10Hz）和连续轨迹规划会频繁读取 `robot_tcp_id`、`spray_do_index`、`spray_distance_mm` 等配置；
- **禁止在属性访问器（Getter）中每次打开 SQLite 连接**；
- `SprayerConfig` 内部维护一个线程安全的 `_db_overrides: dict`；
- 初始化时一次性从 SQLite 批量载入；
- 当前端调用 `POST /api/system/config` 写入 SQLite 成功后，调用 `sprayer_config.reload_db_overrides()` 热重载内存缓存字典，完全实现 **微秒级无锁读取 + 即时热更新**。

---

## 四、数据库与 API 协议重构设计 (Database & API Design)

### 1. 数据库模型微调 (`app/src/db/models.py`)
现有 `SysSettings` 模型定义良好，建议仅做类型安全增强（将 `value` 字段从 `String(255)` 调整为 `Text`，以安全支持容差欧拉角数组 `[90, 0, 90]`、类别列表 `["trousers", "skirt"]` 等 JSON 序列化对象）：

```python
class SysSettings(Base):
    __tablename__ = "sys_settings"

    key = Column(String(64), primary_key=True, index=True)
    value = Column(Text, nullable=False)                    # JSON-serialized string
    category = Column(String(32), index=True, default="common") # robot | calib | spraying | interactive
    description = Column(String(255), default="")
    updated_at = Column(DateTime(timezone=True), onupdate=func.now(), default=func.now())
```

### 2. 系统配置 API 契约扩展 (`app/src/apps/system/api.py`)

#### `GET /api/system/config` (获取全量合并配置及元数据)
- **响应格式**：返回当前生效的完整配置树，并携带是否被数据库覆盖的标志与默认值，便于前端展示高亮与“重置”按钮。
```json
{
  "status": "success",
  "data": {
    "robot": {
      "ip": { "value": "192.168.5.1", "default": "192.168.5.1", "is_overridden": false },
      "port": { "value": 29999, "default": 29999, "is_overridden": false },
      "global_speed_factor": { "value": 50, "default": 50, "is_overridden": false },
      "tcp_id": { "value": 1, "default": 1, "is_overridden": false },
      "spray_do_index": { "value": 1, "default": 1, "is_overridden": false }
    },
    "calib": {
      "mount": { "value": "eye-to-hand", "default": "eye-to-hand", "is_overridden": false },
      "board_cols": { "value": 9, "default": 9, "is_overridden": false },
      "board_rows": { "value": 12, "default": 12, "is_overridden": false },
      "board_square_size_mm": { "value": 15.0, "default": 15.0, "is_overridden": false }
    },
    "spraying": {
      "dist_mm": { "value": 150.0, "default": 150.0, "is_overridden": false },
      "width_mm": { "value": 50.0, "default": 50.0, "is_overridden": false },
      "overlap_rate": { "value": 0.2, "default": 0.2, "is_overridden": false },
      "velocity": { "value": 150.0, "default": 150.0, "is_overridden": false }
    },
    "interactive": {
      "detector_enabled": { "value": true, "default": true, "is_overridden": false },
      "sam_refine": { "value": false, "default": false, "is_overridden": false },
      "detector_classes": { "value": ["trousers"], "default": ["trousers"], "is_overridden": false },
      "detector_conf": { "value": 0.25, "default": 0.25, "is_overridden": false }
    }
  }
}
```

#### `POST /api/system/config` (批量更新动态配置)
- **请求格式**：
```json
{
  "settings": {
    "robot.ip": "192.168.5.1",
    "spraying.dist_mm": 160.0,
    "interactive.detector_classes": ["trousers", "skirt"]
  }
}
```
- **处理逻辑**：
  1. 校验入参合法性（如 `spray_do_index` 必须在 1-16 内，`overlap_rate` 在 0-1 之间）；
  2. 存入 SQLite 并提交事务；
  3. 调用 `sprayer_config.reload_db_overrides()` 热重载内存缓存；
  4. 若包含 `robot.global_speed_factor` 且机械臂处于连接状态，立即向硬件控制器发送 `set_global_speed`；
  5. 若包含 `robot.spray_do_index`，立即热同步至 `robot_service.spray_do_index`。

#### `POST /api/system/config/reset` (重置配置项)
- 支持单项重置或整组重置回 YAML 原始基线，从 SQLite 中移除对应的覆盖记录。

---

## 五、前端页面重构方案 (`ConfigView.tsx`)

根据项目全英文规则（Strict English-Only UI），前端配置中心划分为 **4 大专业卡片**：

```
+-----------------------------------------------------------------------------------+
|  [Server Icon] System Configuration                      [Reset All] [Save Changes]|
|  Manage dynamic hardware connections, calibration targets, and process parameters |
+-----------------------------------------------------------------------------------+
|                                        |                                          |
|  [Card 1] Robot Hardware & Tooling     |  [Card 2] Calibration Target & Mount     |
|  ------------------------------------  |  ------------------------------------    |
|  • Robot IP: [ 192.168.5.1       ]     |  • Camera Mounting: (•) Eye-to-Hand      |
|  • Robot Port: [ 29999           ]     |                     ( ) Eye-in-Hand      |
|  • Global Speed Factor: [50%]---O      |  • Chessboard Cols: [ 9   ]              |
|  • Active Tool TCP: [Gripper TipLink v]|  • Chessboard Rows: [ 12  ]              |
|  • Spray DO Index: [DO 1           v]  |  • Square Size (mm): [ 15.0 ]            |
|                                        |  • Cleaning Dev Limit: [ 5.0% ]          |
+----------------------------------------+------------------------------------------+
|                                        |                                          |
|  [Card 3] Spraying & Path Process      |  [Card 4] Vision & Interactive SAM       |
|  ------------------------------------  |  ------------------------------------    |
|  • Standoff Distance (mm): [ 150.0 ]   |  • Auto Clothing Detector: [ Toggle ON ] |
|  • Spray Pattern Width (mm): [ 50.0 ]  |  • SAM Boundary Refinement: [Toggle OFF] |
|  • Overlap Ratio: [20%]-------O        |  • Target Classes: [x] Trousers [x] Skirt|
|  • Process Velocity (mm/s): [ 150.0 ]  |  • Confidence Threshold: [0.25]----O     |
|  • Point Spacing (mm): [ 100.0 ]       |  • Max Candidate Boxes: [ 5 ]            |
|  • Tolerance Ladder (Guard): [Toggle]  |  • Inference Backend: [ Auto (Fastest) v]|
+-----------------------------------------------------------------------------------+
```

### 1. 交互与微动效细节
- **状态高亮标记 (Modified Badges)**：凡是用户修改过且被 SQLite 覆盖的值，在输入框旁标记轻量蓝色徽章 `Customized`，右侧带一键还原至 YAML 默认值的还原图标；
- **范围防呆校验 (Range Validation)**：
  - 靶距限制：`50.0 mm` ~ `500.0 mm`；
  - 重叠率限制：`0%` ~ `80%`；
  - 速度限制：`10 mm/s` ~ `500 mm/s`；
  - DO 端口：严格限定下拉 `DO 1` ~ `DO 16`；
- **反馈体验**：点击 `Save Changes`，保存中显示加载动画，成功后顶部弹出英文全局通知并高亮绿色。

---

## 六、实施路线图与保障机制 (Implementation Roadmap)

1. **Phase 1：数据持久层与配置内核升级**
   - 升级 `SysSettings` 模型（扩容 `value` 为 Text 并增加 `category`）；
   - 改造 `SettingService`，支持分命名空间存取 JSON 类型；
   - 改造 `SprayerConfig`，构建三级级联读取与内存热缓存机制。

2. **Phase 2：系统 API 层与业务服务热更新联动**
   - 升级 `app/src/apps/system/api.py` 的 GET/POST 路由；
   - 接入参数验证（Pydantic Schema 防御）；
   - 在 `robot_service.py`、`detector.py` 等服务中增加配置热重载监听接口。

3. **Phase 3：前端 `ConfigView.tsx` 页面重构**
   - 实现 4 大模块化配置卡片与表单状态管理；
   - 落实 100% 纯英文标签与错误提示；
   - 接入配置变更后的保存与还原交互。

4. **Phase 4：工程验证与回归**
   - 验证在 Web 界面修改 DO 端口、靶距或棋盘格大小后，后续标定会话与轨迹规划即刻采用新参数；
   - 验证服务重启后 SQLite 历史配置依然被正确保留并覆盖 YAML 默认值。

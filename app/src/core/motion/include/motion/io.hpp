#pragma once

#include "motion/report.hpp"
#include "motion/types.hpp"

#include <string>

namespace motion {

// 与 configs/aisprayer_config.yaml 的 spraying / hardware.robot 对齐
struct SprayingConfig {
  std::string urdf_path;
  std::string tool_name = "gripper_tip_link";
  std::string anchor_source = "config";  // config | home | raw
  Eigen::Vector3d ref_rpy_deg{90.0, 0.0, 90.0};
  Eigen::Vector3d tol_deg{10.0, 10.0, 30.0};
  AxisGrid grid_x{-5.0, 5.0, 2.0};
  AxisGrid grid_y{-5.0, 5.0, 2.0};
  AxisGrid grid_z{-30.0, 30.0, 5.0};
  double speed_mm_s = 150.0;
  double step_mm = 2.0;
  // 容差阶梯择优（对应 OptimizeOptions 同名字段）：放到配置里是为了部署侧能不重编
  // 就关掉阶梯或调整护栏；CLI 显式传参优先级更高。
  bool tol_ladder = true;
  std::vector<double> tol_ladder_scales{0.5, 1.0 / 3.0, 0.25};
  double tol_ladder_stop_peak_ratio = 0.3;
  double tol_ladder_max_pointing_deg = 0.0;
  // ⑤ 边内关节速度约束（对应 OptimizeOptions 同名字段）：部署侧可不重编调整。
  bool opt_enforce_vel_limit = true;
  double opt_vel_soft_ratio = 0.9;
  double opt_vel_cost_weight = 40.0;
  double opt_vel_hard_ratio = 1.0;
  // ⑤-A 腕部奇异自适应降速（verifier 与 optimizer 同名同值，保证选边与终校口径一致）。
  bool singularity_scaling = false;
  double singularity_ref_deg = 25.0;
  double singularity_min_scale = 0.2;
};

bool LoadSprayingConfig(const std::string& yaml_path, SprayingConfig& out, std::string* err);

// 写回 *.poi.path.yaml 顶层的 poi_config 块，供 web 回显实际生效的锚点约束。
struct PoiConfig {
  std::string mode = "absolute_anchor_tolerance";
  std::string anchor_source = "config";
  Eigen::Vector3d ref_rpy_deg{90.0, 0.0, 90.0};
  Eigen::Vector3d tolerance_rpy_deg{10.0, 10.0, 30.0};
  bool has_ref_rpy = true;  // anchor_source=raw 时 Python 侧写 null
  // 容差阶梯择优：tolerance_rpy_deg 保持原语义（= 用户/配置请求的包络，前端会回显它），
  // 另用一个新字段记录最终采纳的包络（总是≤请求档），避免前端把两者混淆。
  Eigen::Vector3d adopted_tolerance_rpy_deg{10.0, 10.0, 30.0};
  bool tolerance_ladder_applied = false;
};

bool LoadPathYaml(const std::string& path, PathDocument& out, std::string* err);
// verify / poi 均可为空；非空时按重构前 Python 版 _clean_report_data 的 schema 落盘。
bool SavePathYaml(const std::string& path, const PathDocument& doc, const VerifyReport* verify,
                  const PoiConfig* poi, std::string* err);

std::string JsonReportVerify(const VerifyReport& report, double elapsed_ms, bool success,
                             const std::string& message);
std::string JsonReportOptimize(const OptimizeResult& result, const VerifyReport* all,
                               bool success, const std::string& message);
// 把任意文本安全地放进 JSON 字符串值（转义引号/反斜杠/控制字符）。
std::string JsonEscapeString(const std::string& s);

}  // namespace motion

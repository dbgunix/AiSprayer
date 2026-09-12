#pragma once

#include "motion/conventions.hpp"
#include "motion/robot_model.hpp"
#include "motion/types.hpp"

#include <array>
#include <string>
#include <vector>

namespace motion {

struct Issue {
  std::string type;
  std::string severity;  // ERROR | WARNING
  int segment_index = 0;
  int step_index = 0;
  std::string detail;
  Eigen::Vector3d location_xyz_mm{0, 0, 0};
};

struct PathVerifyReport {
  int path_id = 0;
  std::string name;
  std::string status = "PASS";  // PASS | WARNING | FAILED
  int total_interpolated = 0;
  double speed_mm_s = 120.0;
  double step_size_mm = 1.5;
  double recommended_safe_speed_mm_s = 120.0;
  std::array<double, 6> max_joint_velocities_deg_s{{180, 180, 180, 180, 180, 180}};
  std::array<double, 6> peak_joint_speeds_deg_s{{0, 0, 0, 0, 0, 0}};
  std::vector<Issue> issues;
  std::vector<JointVec> trajectory_q;
  std::vector<std::array<double, 6>> trajectory_tcp;  // x,y,z mm + rx,ry,rz deg
  // ⑤-A 奇异自适应降速剖面：逐输入航点的建议笛卡尔线速度 (mm/s)。
  // 仅在开启奇异缩放时填充；非奇异航点 = nominal speed，近奇异航点按 |sin(J5)| 压低。
  // 执行侧按此逐段设 TCPSpeed（成对括住队列以保 CP 连续）。
  std::vector<double> waypoint_speed_mm_s;
};

struct VerifySummary {
  std::string status = "PASS";
  int total_paths = 0;
  int total_waypoints = 0;
  int total_steps = 0;
  int total_issues = 0;
  int singularity_count = 0;
  int overspeed_count = 0;
  int unreachable_count = 0;
};

struct VerifyReport {
  VerifySummary summary;
  double nominal_speed_mm_s = 120.0;
  double slerp_step_mm = 1.5;
  std::array<double, 6> max_joint_velocities_deg_s{{180, 180, 180, 180, 180, 180}};
  ToolOffset urdf_tcp;
  std::vector<PathVerifyReport> path_reports;
};

struct VerifyOptions {
  double step_mm = 1.5;
  double speed_mm_s = 120.0;
  // ⑤-A 腕部奇异自适应降速：按 |sin(J5)| 在近奇异段压低线速度后再折算关节角速度判超速。
  // 默认关（=旧行为，不影响现有回归）；开启后校验器与 ⑤ 采用同一缩放模型，口径一致。
  bool singularity_scaling = false;
  double singularity_ref_deg = 25.0;   // |J5| ≥ 此值(deg) 不减速（可操作度充分）
  double singularity_min_scale = 0.2;  // 减速下限：最多降到该比例的名义线速度
};

// 容差阶梯中的一档：一次完整 DP + 密集校验的结果摘要。
// 只用于日志/报表/择优，不参与轨迹数据本身。
struct LadderRung {
  Eigen::Vector3d tol_deg{0.0, 0.0, 0.0};  // 该档实际使用的锚点包络
  // PASS | WARNING | FAILED | ERROR(该档抛异常) | UNVERIFIED(关闭了密集校验)
  std::string status = "ERROR";
  double peak_deg_s = 0.0;   // max_j 峰值关节角速度 (deg/s)
  double peak_ratio = 0.0;   // max_j peak_j / limit_j；无密集校验时为 0（不参与早停）
  double max_pointing_deg = 0.0;  // 优化后枪尖法向相对原始法向的最大偏量 (deg)
  double objective = 0.0;    // DP 总代价 J
  double elapsed_ms = 0.0;
  std::string error;         // status==ERROR 时的异常信息
};

struct OptimizeOptions {
  AxisGrid grid_x{-5, 5, 2};
  AxisGrid grid_y{-5, 5, 2};
  AxisGrid grid_z{-30, 30, 5};  // 与 aisprayer_config.yaml spraying.grid_tol_z_deg 一致
  int beam_width = 32;
  int max_candidates_per_branch = 16;
  int movel_checks_min = 10;
  int movel_checks_max = 100;
  double movel_spacing_mm = 5.0;
  // 候选姿态的第一质量指标是相对名义工具 Z 轴的夹角。自旋不改变圆喷嘴的
  // 指向，但仍作为同等法向候选之间的轻微稳定性偏好。
  // raw 模式会先单独验证零偏差链；只有该链不可行时，才与关节平滑代价共同
  // 选择包络内的修复姿态。
  double pointing_cost_weight = 1.0;
  double spin_cost_weight = 0.01;
  JointVec joint_weights = (JointVec() << 1.0, 1.2, 1.0, 0.8, 0.8, 0.5).finished();
  // 密集 MoveL 复核开关；采样步长/速度由传入的 ChainVerifier 自带 VerifyOptions 决定。
  bool dense_verify = true;

  // ── 边内关节速度约束（⑤ 根治真·喷涂超速）─────────────────────────────────
  // 旧 DP 代价 J = Σ weights·Δq² 只看「相邻航点关节位移大小」，**完全不含 dt**；
  // 校验器却按 MoveL 线速度算 dt 判 °/s。于是 DP 眼里「Δq 很小」的一条边，落到固定
  // 线速度执行时照样能顶出超速（实测 raw 下 J4 持续 ~200–250°/s）。这里让 DP 在选边时
  // 就按段时长 dt_seg = 段长/线速度 折算真实角速度，超速边加罚/硬禁，从搜索阶段规避。
  double exec_speed_mm_s = 150.0;  // 与校验器同一线速度 (mm/s)，保证 DP 与终校口径一致
  bool enforce_vel_limit = true;   // false = 退化为旧行为（纯 Δq²，无速度约束）
  double vel_soft_ratio = 0.9;     // 关节角速度 ≤ 该比例 × 限速 的边不加罚（留 10% 余量）
  double vel_cost_weight = 40.0;   // 超软阈惩罚权重：cost += Σ (ratio - soft)² × 此值
  double vel_hard_ratio = 1.0;     // 可收紧硬阈；旧配置的 0 或 >1 不得放宽物理限速

  // ⑤-A 腕部奇异自适应降速：必须与传入 ChainVerifier 的 VerifyOptions 取同名同值，
  // 否则 DP 判不可行的边与终校按降速后判可行的边不一致。缩放模型见 SingularitySpeedScale。
  bool singularity_scaling = false;
  double singularity_ref_deg = 25.0;
  double singularity_min_scale = 0.2;

  // ── 容差阶梯择优（Monotonicity Guard）──────────────────────────────────────
  // 大容差包络在几何上包含小容差包络，理论上最优解不应变差；但 DP 的目标
  // J = Σ Δq² + 姿态偏置 **不含峰值角速度**，且候选集与 beam 剪枝都随容差变化，
  // 实测「放大容差」会让峰值明显变差（见 docs/optimizer_monotonicity_improvement_proposal.md
  // §5–§6：同一工件 [10,10,50] → 43.6°/s，[30,30,180] → 133.8°/s）。
  // 这里用「多档包络各跑一次 + 按与容差无关的标尺择优」把包含关系变成构造性保证：
  // 每一档的解都落在用户请求的包络内，密集校验又与容差无关，所以返回的解
  // 不会比阶梯里任何一档差。代价是最多多跑几档（单档 ~1.5 s），由早停阈值收敛。
  bool tol_ladder = true;
  // 相对请求包络的收紧比例（逐分量乘）；请求档(1.0)总是第一档，不在此列出。
  std::vector<double> tol_ladder_scales{0.5, 1.0 / 3.0, 0.25};
  // 早停：某档 PASS 且峰值 ≤ 该比例 × 关节限速时不再继续收紧（0 表示跑完全部档位）。
  double tol_ladder_stop_peak_ratio = 0.3;
  // 指向偏量护栏：收紧包络会牺牲法向跟随（实测峰值 133.8→44 伴随指向偏量 18.7°→45.9°）。
  // >0 时，某档的最大指向偏量超过请求档 + 该值就弃用该档；0 = 不限制（默认，只看运动学质量）。
  double tol_ladder_max_pointing_deg = 0.0;

  std::string Validate() const;
};

struct OptimizeResult {
  PathItem path;
  bool modified = false;
  std::vector<JointVec> joints_rad;
  PathVerifyReport verify;
  double elapsed_ms = 0.0;
  double objective = 0.0;  // DP 回溯终点的累计代价 J（诊断与择优用）
  // 采纳解实际使用的包络：阶梯择优后可能比请求的更紧，报表/落盘必须回显它。
  Eigen::Vector3d adopted_tol_deg{0.0, 0.0, 0.0};
  std::vector<LadderRung> ladder;  // 各档摘要；未启用阶梯时为空
};

}  // namespace motion

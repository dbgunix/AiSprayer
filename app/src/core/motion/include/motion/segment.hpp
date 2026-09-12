#pragma once

#include "motion/kinematics.hpp"
#include "motion/robot_model.hpp"
#include "motion/types.hpp"

#include <optional>
#include <vector>

namespace motion {

struct DenseStep {
  Transform T_gun;
  Transform T_flange_ctrl;
  double dt_sec = 0.0;
  int segment_index = 0;
  bool is_jump = false;
};

class Interpolator {
 public:
  Interpolator(const ToolOffset& tool, double step_mm = 1.5, double speed_mm_s = 120.0);

  std::vector<DenseStep> Interpolate(const std::vector<Waypoint>& waypoints) const;

 private:
  const ToolOffset& tool_;
  double step_mm_;
  double speed_mm_s_;
};

// 一段笛卡尔直线（MoveL）是否可连续跟踪。不是运动学：它只是「插值 + BestIk」。
struct MoveLQuery {
  Eigen::Vector3d p_start_m{0, 0, 0};
  Eigen::Vector3d p_end_m{0, 0, 0};
  Eigen::Vector4d quat1_xyzw{0, 0, 0, 1};
  Eigen::Vector4d quat2_xyzw{0, 0, 0, 1};
  JointVec q_start = JointVec::Zero();
  std::vector<double> alphas;
  JointVec q_branch_end = JointVec::Zero();
  bool check_end_branch = true;
  double max_jump_rad = Rad(kBranchJumpDeg);
  double match_rad = Rad(kEndBranchMatchDeg);
  JointVec weights = JointVec::Ones();
};

struct MoveLWalk {
  JointVec q_end = JointVec::Zero();
  double cost = 0.0;
};

class SegmentChecker {
 public:
  explicit SegmentChecker(const Cr5Kinematics& kin);
  SegmentChecker(const Cr5Kinematics& kin, const ToolOffset& tool);

  std::optional<MoveLWalk> Walk(const MoveLQuery& q) const;

  // C ABI：参数布局与旧 cr5_kinematics::walk_movel 相同。
  int WalkRaw(const double* p_start, const double* p_end, const double* quat1,
              const double* quat2, const double* q_start, const double* alphas, int n_alphas,
              const double* q_branch_end, int check_end_branch, double max_jump_rad,
              double match_rad, const double* weights, double deg2_from_rad2, double* q_end_out,
              double* cost_out) const;

 private:
  const Cr5Kinematics& kin_;
  const ToolOffset* tool_ = nullptr;  // null keeps the C ABI flange-pose contract
};

}  // namespace motion

#pragma once

#include "motion/conventions.hpp"
#include "motion/robot_model.hpp"
#include "motion/types.hpp"

#include <optional>

namespace motion {

struct SingularityFlags {
  bool wrist = false;
  bool elbow = false;
  bool shoulder = false;
  double wrist_angle_deg = 0.0;
  double elbow_angle_deg = 0.0;
  double shoulder_q1_separation_deg = 0.0;
  bool is_singular() const { return wrist || elbow || shoulder; }
};

// CR5 解析解：DH 闭式 + URDF q2/q4 偏置 + 控制器帧。全部实现在这一个类里。
class Cr5Kinematics {
 public:
  explicit Cr5Kinematics(RobotLimits limits = {});

  Transform Fk(const JointVec& q_urdf) const;
  int Ik(const Transform& T_urdf, JointVec* out_sols) const;
  std::optional<JointVec> BestIk(const Transform& T_urdf, const JointVec& q_seed,
                                 const JointVec* weights = nullptr) const;
  const RobotLimits& limits() const { return limits_; }

  void FkController(const JointVec& q_urdf, Eigen::Vector3d& xyz_mm,
                    Eigen::Vector3d& rpy_deg) const;
  int IkController(const Eigen::Vector3d& xyz_mm, const Eigen::Vector3d& rpy_deg,
                   JointVec* out_sols) const;

  bool IsJointValid(const JointVec& q) const;
  // Closest equivalent angles inside the physical limits (rad), axis by axis.
  std::optional<JointVec> NearestInLimits(const JointVec& q, const JointVec& reference) const;
  SingularityFlags CheckSingularity(const JointVec& q, const Transform& T_urdf) const;
  int IkBatch(const Transform* T, int n, JointVec* out, int* n_sols) const;

  // 行主序 4×4 热路径（C ABI / 段检查）。表达式与旧 cr5_* 逐位相同。
  static void DhFk(const double* q_dh, double* T16);
  static int DhIk(const double* T16, double* q_sols, double q6_des = 0.0);
  static void FkRaw(const double* q_urdf, double* T16);
  static int IkRaw(const double* T_urdf, double* q_sols);
  int BestIkRaw(const double* T_urdf, const double* q_seed, const double* weights,
                double* q_out) const;

 private:
  RobotLimits limits_;
};

}  // namespace motion

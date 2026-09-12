#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>

namespace motion {

using JointVec = Eigen::Matrix<double, 6, 1>;
using Transform = Eigen::Isometry3d;

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kMmPerM = 1000.0;
inline constexpr double kSingSin = 0.052335956242943835;       // sin(3°)
inline constexpr double kShoulderHalfRad = 0.05235987755982989; // 3°
inline constexpr double kBranchJumpDeg = 45.0;
inline constexpr double kEndBranchMatchDeg = 5.0;
inline constexpr double kJointTol = 1e-4;
inline constexpr double kDhD4 = 0.141;
inline constexpr double kDhD6 = 0.105;

inline double Deg(double rad) { return rad * 180.0 / kPi; }
inline double Rad(double deg) { return deg * kPi / 180.0; }
inline double WrapPi(double x) {
  x = std::fmod(x + kPi, 2.0 * kPi);
  if (x < 0.0) x += 2.0 * kPi;
  return x - kPi;
}
inline double Wrap180(double deg) {
  deg = std::fmod(deg + 180.0, 360.0);
  if (deg < 0.0) deg += 360.0;
  return deg - 180.0;
}

inline JointVec WrapPi(const JointVec& q) {
  JointVec o;
  for (int i = 0; i < 6; ++i) o[i] = WrapPi(q[i]);
  return o;
}

// Dobot 控制器报文 [rx,ry,rz](deg): R = Rz(rz)·Ry(ry)·Rx(rx) ≡ scipy 'xyz'
inline Eigen::Matrix3d RotFromCtrlRpyDeg(const Eigen::Vector3d& rpy_deg) {
  const double ax = Rad(rpy_deg[0]), ay = Rad(rpy_deg[1]), az = Rad(rpy_deg[2]);
  return Eigen::AngleAxisd(az, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
         Eigen::AngleAxisd(ay, Eigen::Vector3d::UnitY()).toRotationMatrix() *
         Eigen::AngleAxisd(ax, Eigen::Vector3d::UnitX()).toRotationMatrix();
}

// 与 path_opt._euler_xyz_deg_from_R / scipy as_euler('xyz') 一致
inline Eigen::Vector3d CtrlRpyDegFromRot(const Eigen::Matrix3d& R) {
  const double sy = std::min(1.0, std::max(-1.0, -R(2, 0)));
  const double y = std::asin(sy);
  double x, z;
  if (std::abs(sy) < 0.999999) {
    x = std::atan2(R(2, 1), R(2, 2));
    z = std::atan2(R(1, 0), R(0, 0));
  } else {
    x = std::atan2(std::copysign(1.0, sy) * R(0, 1), R(1, 1));
    z = 0.0;
  }
  return Eigen::Vector3d(Deg(x), Deg(y), Deg(z));
}

inline Transform PoseFromCtrlMmDeg(const Eigen::Vector3d& xyz_mm,
                                   const Eigen::Vector3d& rpy_deg) {
  Transform T = Transform::Identity();
  T.linear() = RotFromCtrlRpyDeg(rpy_deg);
  T.translation() = xyz_mm / kMmPerM;
  return T;
}

inline void PoseToCtrlMmDeg(const Transform& T, Eigen::Vector3d& xyz_mm,
                            Eigen::Vector3d& rpy_deg) {
  xyz_mm = T.translation() * kMmPerM;
  rpy_deg = CtrlRpyDegFromRot(T.linear());
}

// T_urdf = T_base_inv · T_ctrl · T_tool_inv（与 CR5Kinematics.controller_matrix_to_urdf 逐元素一致）
inline Transform CtrlToUrdf(const Transform& Tc) {
  const Eigen::Matrix3d R = Tc.linear();
  const Eigen::Vector3d p = Tc.translation();
  Transform Tu = Transform::Identity();
  Tu.linear() << -R(0, 2), R(0, 0), R(0, 1),
                 -R(1, 2), R(1, 0), R(1, 1),
                  R(2, 2), -R(2, 0), -R(2, 1);
  Tu.translation() << -p[0], -p[1], p[2];
  return Tu;
}

inline void TransformToRowMajor(const Transform& T, double* out16) {
  const Eigen::Matrix4d M = T.matrix();
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c) out16[r * 4 + c] = M(r, c);
}

inline Transform TransformFromRowMajor(const double* T16) {
  Eigen::Matrix4d M;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c) M(r, c) = T16[r * 4 + c];
  return Transform(M);
}

inline double GeodesicDeg(const Eigen::Matrix3d& Ra, const Eigen::Matrix3d& Rb) {
  const double c = 0.5 * ((Ra.transpose() * Rb).trace() - 1.0);
  return Deg(std::acos(std::min(1.0, std::max(-1.0, c))));
}

// 两个姿态的「指向」夹角：只比工具 Z 轴（喷嘴指向），忽略绕该轴的自旋。
// 与 GeodesicDeg 的区别：绕枪轴自旋 80° 时 GeodesicDeg≈80 而 PointingDeg≈0，
// 后者才是影响漆膜厚度的工艺量（圆喷嘴绕轴旋转对称）。
inline double PointingDeg(const Eigen::Matrix3d& Ra, const Eigen::Matrix3d& Rb) {
  const double c = std::min(1.0, std::max(-1.0, Ra.col(2).dot(Rb.col(2))));
  return Deg(std::acos(c));
}

// scipy as_quat / walk_movel: [x, y, z, w]，w>=0
inline Eigen::Vector4d QuatXyzw(const Eigen::Matrix3d& R) {
  Eigen::Quaterniond q(R);
  q.normalize();
  if (q.w() < 0.0) q.coeffs() *= -1.0;
  return Eigen::Vector4d(q.x(), q.y(), q.z(), q.w());
}

inline int BranchKey(const JointVec& q) {
  JointVec qw = WrapPi(q);
  const double q2 = std::fmod(qw[2], 2.0 * kPi);
  const double q4 = std::fmod(qw[4], 2.0 * kPi);
  const double q02 = q2 < 0.0 ? q2 + 2.0 * kPi : q2;
  const double q04 = q4 < 0.0 ? q4 + 2.0 * kPi : q4;
  const int shoulder = qw[0] >= 0.0 ? 0 : 1;
  const int elbow = q02 <= kPi ? 0 : 1;
  const int wrist = q04 <= kPi ? 0 : 1;
  return (shoulder << 2) | (elbow << 1) | wrist;
}

inline double ShoulderHalfRad(const Transform& T_urdf) {
  const Eigen::Matrix4d M = T_urdf.matrix();
  const double t02 = -M(0, 0), t03 = -M(0, 3);
  const double t12 = -M(1, 0), t13 = -M(1, 3);
  const double a = kDhD6 * t12 - t13;
  const double b = kDhD6 * t02 - t03;
  const double r = a * a + b * b;
  if (r <= 1e-16) return 0.0;
  const double ratio = kDhD4 / std::sqrt(r);
  if (ratio >= 1.0) return 0.0;
  if (ratio <= -1.0) return kPi;
  return std::acos(ratio);
}

struct AxisGrid {
  double min_deg = 0.0;
  double max_deg = 0.0;
  double step_deg = 0.0;
};

// 与 numpy.arange(min, max + 0.5*step, step) 一致：用 min + i*step 而非累加，
// 避免浮点累积误差导致边界点丢失或与 Python 侧网格不一致。
inline std::vector<double> ExpandAxisGrid(const AxisGrid& g) {
  if (g.step_deg <= 0.0 || g.max_deg < g.min_deg) return {0.0};
  const double span = g.max_deg + 0.5 * g.step_deg - g.min_deg;
  const int n = static_cast<int>(std::ceil(span / g.step_deg));
  std::vector<double> out;
  out.reserve(static_cast<size_t>(std::max(1, n)));
  for (int i = 0; i < n; ++i) out.push_back(g.min_deg + i * g.step_deg);
  if (out.empty()) out.push_back(g.min_deg);
  return out;
}

// 腕部奇异速度回退（Nakamura 奇异性鲁棒逆解的离散简化）：以 |sin(J5)| 作腕部可操作度代理。
// J5→0 时 J4/J6 轴趋于共线（腕部奇异），小笛卡尔旋转被放大成大关节角速度；这里越接近
// 奇异越压低笛卡尔线速度，使折算关节角速度不超限（保持枪尖贴法向，仅该段节拍变长）。
// 返回 ∈ [floor_ratio, 1]：|J5|≥ref_rad 时不减速(=1)，趋 0 时降到 floor_ratio。量纲：rad。
inline double SingularitySpeedScale(double q5_rad, double ref_rad, double floor_ratio) {
  const double r = std::sin(ref_rad);
  if (r <= 0.0) return 1.0;
  double k = std::abs(std::sin(q5_rad)) / r;
  if (k > 1.0) k = 1.0;
  if (k < floor_ratio) k = floor_ratio;
  return k;
}

}  // namespace motion

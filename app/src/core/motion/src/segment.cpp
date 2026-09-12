#include "motion/segment.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace motion {
namespace {

// 与旧 cr5_path_opt.cpp quat_slerp_to_R 逐表达式相同。quat = [x, y, z, w]。
void QuatSlerpToR(const double* q1, const double* q2, double alpha, double* R) {
  double dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
  double q2m[4] = {q2[0], q2[1], q2[2], q2[3]};
  if (dot < 0.0) {
    q2m[0] = -q2m[0];
    q2m[1] = -q2m[1];
    q2m[2] = -q2m[2];
    q2m[3] = -q2m[3];
    dot = -dot;
  }
  double q[4];
  if (dot > 0.9995) {
    const double om = 1.0 - alpha;
    q[0] = om * q1[0] + alpha * q2m[0];
    q[1] = om * q1[1] + alpha * q2m[1];
    q[2] = om * q1[2] + alpha * q2m[2];
    q[3] = om * q1[3] + alpha * q2m[3];
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] /= n;
    q[1] /= n;
    q[2] /= n;
    q[3] /= n;
  } else {
    const double theta = std::acos(dot > 1.0 ? 1.0 : (dot < -1.0 ? -1.0 : dot));
    const double s = std::sin(theta);
    const double w1 = std::sin((1.0 - alpha) * theta) / s;
    const double w2 = std::sin(alpha * theta) / s;
    q[0] = w1 * q1[0] + w2 * q2m[0];
    q[1] = w1 * q1[1] + w2 * q2m[1];
    q[2] = w1 * q1[2] + w2 * q2m[2];
    q[3] = w1 * q1[3] + w2 * q2m[3];
  }
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  R[0] = 1.0 - 2.0 * (y * y + z * z);
  R[1] = 2.0 * (x * y - z * w);
  R[2] = 2.0 * (x * z + y * w);
  R[3] = 2.0 * (x * y + z * w);
  R[4] = 1.0 - 2.0 * (x * x + z * z);
  R[5] = 2.0 * (y * z - x * w);
  R[6] = 2.0 * (x * z - y * w);
  R[7] = 2.0 * (y * z + x * w);
  R[8] = 1.0 - 2.0 * (x * x + y * y);
}

void CtrlToUrdfRow(const double* Tc, double* Tu) {
  Tu[0] = -Tc[2];
  Tu[1] = Tc[0];
  Tu[2] = Tc[1];
  Tu[3] = -Tc[3];
  Tu[4] = -Tc[6];
  Tu[5] = Tc[4];
  Tu[6] = Tc[5];
  Tu[7] = -Tc[7];
  Tu[8] = Tc[10];
  Tu[9] = -Tc[8];
  Tu[10] = -Tc[9];
  Tu[11] = Tc[11];
  Tu[12] = 0.0;
  Tu[13] = 0.0;
  Tu[14] = 0.0;
  Tu[15] = 1.0;
}

}  // namespace

Interpolator::Interpolator(const ToolOffset& tool, double step_mm, double speed_mm_s)
    : tool_(tool), step_mm_(step_mm), speed_mm_s_(speed_mm_s) {}

std::vector<DenseStep> Interpolator::Interpolate(const std::vector<Waypoint>& waypoints) const {
  std::vector<DenseStep> out;
  if (waypoints.empty()) return out;

  auto flange = [&](const Transform& T_gun) -> Transform { return T_gun * tool_.T_tcp_inv; };

  if (waypoints.size() < 2) {
    DenseStep s;
    s.T_gun = waypoints[0].tcp_pose;
    s.T_flange_ctrl = flange(s.T_gun);
    s.dt_sec = 0.0;
    s.segment_index = 0;
    s.is_jump = false;
    out.push_back(s);
    return out;
  }

  const double step_m = step_mm_ / kMmPerM;
  const double speed_m_s = speed_mm_s_ / kMmPerM;

  DenseStep first;
  first.T_gun = waypoints.front().tcp_pose;
  first.T_flange_ctrl = flange(first.T_gun);
  out.push_back(first);

  for (size_t seg = 0; seg + 1 < waypoints.size(); ++seg) {
    const Transform T0 = waypoints[seg].tcp_pose;
    const Transform T1 = waypoints[seg + 1].tcp_pose;
    const Eigen::Vector3d p0 = T0.translation();
    const Eigen::Vector3d p1 = T1.translation();
    const double dist_m = (p1 - p0).norm();
    const int num_steps = std::max(1, static_cast<int>(std::ceil(dist_m / step_m)));
    const double seg_duration = std::max(0.001, dist_m / speed_m_s);
    const double dt = seg_duration / static_cast<double>(num_steps);
    const bool is_jump = waypoints[seg + 1].is_jump || !waypoints[seg + 1].spraying;

    Eigen::Quaterniond q0(T0.linear());
    Eigen::Quaterniond q1(T1.linear());
    q0.normalize();
    q1.normalize();

    for (int step = 1; step <= num_steps; ++step) {
      const double t = static_cast<double>(step) / static_cast<double>(num_steps);
      DenseStep s;
      s.T_gun = Transform::Identity();
      s.T_gun.linear() = q0.slerp(t, q1).toRotationMatrix();
      s.T_gun.translation() = (1.0 - t) * p0 + t * p1;
      s.T_flange_ctrl = flange(s.T_gun);
      s.dt_sec = dt;
      s.segment_index = static_cast<int>(seg);
      s.is_jump = is_jump;
      out.push_back(s);
    }
  }

  return out;
}

SegmentChecker::SegmentChecker(const Cr5Kinematics& kin) : kin_(kin) {}

SegmentChecker::SegmentChecker(const Cr5Kinematics& kin, const ToolOffset& tool)
    : kin_(kin), tool_(&tool) {}

int SegmentChecker::WalkRaw(const double* p_start, const double* p_end, const double* quat1,
                            const double* quat2, const double* q_start, const double* alphas,
                            int n_alphas, const double* q_branch_end, int check_end_branch,
                            double max_jump_rad, double match_rad, const double* weights,
                            double deg2_from_rad2, double* q_end_out, double* cost_out,
                            const MoveLQuery* timing, JointVec* peak_vel_deg_s) const {
  double prev[6];
  std::memcpy(prev, q_start, 6 * sizeof(double));
  double acc = 0.0;
  double previous_alpha = 0.0;
  if (peak_vel_deg_s) peak_vel_deg_s->setZero();
  double T_ctrl[16];
  T_ctrl[12] = 0.0;
  T_ctrl[13] = 0.0;
  T_ctrl[14] = 0.0;
  T_ctrl[15] = 1.0;

  for (int k = 0; k < n_alphas; ++k) {
    const double a = alphas[k];
    const double om = 1.0 - a;
    T_ctrl[3] = om * p_start[0] + a * p_end[0];
    T_ctrl[7] = om * p_start[1] + a * p_end[1];
    T_ctrl[11] = om * p_start[2] + a * p_end[2];

    double R[9];
    QuatSlerpToR(quat1, quat2, a, R);
    T_ctrl[0] = R[0];
    T_ctrl[1] = R[1];
    T_ctrl[2] = R[2];
    T_ctrl[4] = R[3];
    T_ctrl[5] = R[4];
    T_ctrl[6] = R[5];
    T_ctrl[8] = R[6];
    T_ctrl[9] = R[7];
    T_ctrl[10] = R[8];

    // The optimizer supplies TCP MoveL poses, matching the execution and
    // verifier contract.  Convert each interpolated TCP pose to its flange
    // pose before IK.  The legacy C ABI has no ToolOffset and continues to
    // supply flange poses, so it leaves tool_ null.
    if (tool_) {
      const Transform T_flange = TransformFromRowMajor(T_ctrl) * tool_->T_tcp_inv;
      TransformToRowMajor(T_flange, T_ctrl);
    }

    double T_urdf[16];
    CtrlToUrdfRow(T_ctrl, T_urdf);

    double nxt[6];
    // The C++ optimizer follows the verifier's unweighted nearest-IK rule;
    // weights affect path cost only. The legacy C ABI retains its IK weights.
    if (!kin_.BestIkRaw(T_urdf, prev, timing ? nullptr : weights, nxt)) return 0;
    if (!kin_.IsJointValid(Eigen::Map<const JointVec>(nxt))) return 0;
    if (std::fabs(std::sin(nxt[4])) < kSingSin || std::fabs(std::sin(nxt[2])) < kSingSin) {
      return 0;
    }
    if (ShoulderHalfRad(TransformFromRowMajor(T_urdf)) < kShoulderHalfRad) return 0;

    double max_abs = 0.0;
    for (int j = 0; j < 6; ++j) {
      const double dq = nxt[j] - prev[j];
      const double ad = std::fabs(dq);
      if (ad > max_abs) max_abs = ad;
      acc += weights[j] * dq * dq;
    }
    if (max_abs > max_jump_rad) return 0;
    if (timing && peak_vel_deg_s && timing->duration_sec > 0.0) {
      const double dt_sec = (a - previous_alpha) * timing->duration_sec;
      if (dt_sec <= 0.0) return 0;
      const double scale = timing->singularity_scaling
          ? SingularitySpeedScale(nxt[4], Rad(timing->singularity_ref_deg),
                                  timing->singularity_min_scale)
          : 1.0;
      for (int j = 0; j < 6; ++j) {
        (*peak_vel_deg_s)[j] = std::max((*peak_vel_deg_s)[j],
            std::abs(Deg(nxt[j] - prev[j])) * scale / dt_sec);
      }
    }
    previous_alpha = a;
    std::memcpy(prev, nxt, 6 * sizeof(double));
  }

  if (check_end_branch) {
    const auto q_target = kin_.NearestInLimits(Eigen::Map<const JointVec>(q_branch_end),
                                               Eigen::Map<const JointVec>(prev));
    if (!q_target) return 0;
    for (int j = 0; j < 6; ++j) {
      if (std::fabs(prev[j] - (*q_target)[j]) > match_rad) return 0;
    }
  }

  *cost_out = acc * deg2_from_rad2;
  std::memcpy(q_end_out, prev, 6 * sizeof(double));
  return 1;
}

std::optional<MoveLWalk> SegmentChecker::Walk(const MoveLQuery& q) const {
  if (q.alphas.empty()) return std::nullopt;
  double q_end[6];
  double cost = 0.0;
  JointVec peak_vel_deg_s;
  const double deg2 = (180.0 / kPi) * (180.0 / kPi);
  const int ok = WalkRaw(q.p_start_m.data(), q.p_end_m.data(), q.quat1_xyzw.data(),
                         q.quat2_xyzw.data(), q.q_start.data(), q.alphas.data(),
                         static_cast<int>(q.alphas.size()), q.q_branch_end.data(),
                         q.check_end_branch ? 1 : 0, q.max_jump_rad, q.match_rad, q.weights.data(),
                         deg2, q_end, &cost, &q, &peak_vel_deg_s);
  if (!ok) return std::nullopt;
  MoveLWalk w;
  w.q_end = Eigen::Map<const JointVec>(q_end);
  w.cost = cost;
  w.peak_vel_deg_s = peak_vel_deg_s;
  return w;
}

}  // namespace motion

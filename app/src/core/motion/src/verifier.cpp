#include "motion/verifier.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace motion {
namespace {

Eigen::Vector3d LocXyz(const Transform& T_gun) {
  const Eigen::Vector3d p = T_gun.translation() * kMmPerM;
  return Eigen::Vector3d(std::round(p[0] * 100.0) / 100.0,
                         std::round(p[1] * 100.0) / 100.0,
                         std::round(p[2] * 100.0) / 100.0);
}

std::string Fmt1(double v) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(1) << v;
  return os.str();
}

std::string JoinDeg(const JointVec& q) {
  std::ostringstream os;
  os << "[";
  for (int i = 0; i < 6; ++i) {
    if (i) os << ", ";
    os << std::fixed << std::setprecision(2) << Deg(q[i]);
  }
  os << "]";
  return os.str();
}

void PushIssue(std::vector<Issue>& issues, const char* type, const char* sev,
               int seg, int step, const std::string& detail, const Eigen::Vector3d& loc) {
  Issue iss;
  iss.type = type;
  iss.severity = sev;
  iss.segment_index = seg;
  iss.step_index = step;
  iss.detail = detail;
  iss.location_xyz_mm = loc;
  issues.push_back(std::move(iss));
}

// 与 path_opt._DEFAULT_SEED_Q / 生产 POI 校验一致（Home）。
// Python KinematicChainVerifier 无 init_q 时用 ready 姿态，结果会换支；
// 本仓库黄金数据（2026-09-03_225937）是 Home 种子跑出来的。
JointVec DefaultSeed() {
  return (JointVec() << 0.0, 0.0, -kPi / 2.0, -kPi / 2.0, -kPi / 2.0, 0.0).finished();
}

}  // namespace

ChainVerifier::ChainVerifier(const Cr5Kinematics& kin, const ToolOffset& tool, VerifyOptions opt)
    : kin_(kin), tool_(tool), opt_(opt), interp_(tool, opt.step_mm, opt.speed_mm_s) {}

Transform ChainVerifier::ToUrdfFlange(const Transform& T_gun) const {
  return CtrlToUrdf(T_gun * tool_.T_tcp_inv);
}

SingularityFlags ChainVerifier::Diagnose(const JointVec& q, const Transform& T_gun, int step_idx,
                                         int seg_idx, std::vector<Issue>& issues, bool prev[3],
                                         bool emit_always) const {
  const Eigen::Vector3d loc = LocXyz(T_gun);
  const Transform T_urdf = ToUrdfFlange(T_gun);
  const SingularityFlags risk = kin_.CheckSingularity(q, T_urdf);

  if (!kin_.IsJointValid(q)) {
    PushIssue(issues, "JOINT_LIMIT", "ERROR", seg_idx, step_idx,
              "Joint angles out of URDF/CR5 limits: " + JoinDeg(q) + " deg", loc);
  }

  const bool active[3] = {risk.shoulder, risk.elbow, risk.wrist};
  const char* types[3] = {"SHOULDER_SINGULARITY", "ELBOW_SINGULARITY", "WRIST_SINGULARITY"};
  const std::string details[3] = {
      "Near shoulder singularity: two q1 branches separated by " +
          Fmt1(risk.shoulder_q1_separation_deg) +
          "° (wrist near J1-axis cylinder).",
      "Near elbow singularity: Joint 3 angle=" + Fmt1(risk.elbow_angle_deg) +
          "° (a2/a3 collinear, sin(q3)~0).",
      "Near wrist singularity: Joint 5 angle=" + Fmt1(risk.wrist_angle_deg) +
          "° (J4/J6 collinear, sin(q5)~0).",
  };
  for (int i = 0; i < 3; ++i) {
    const bool entered = active[i] && (emit_always || !prev[i]);
    if (entered) {
      PushIssue(issues, types[i], "WARNING", seg_idx, step_idx, details[i], loc);
    }
    prev[i] = active[i];
  }
  return risk;
}

PathVerifyReport ChainVerifier::Verify(const PathItem& path,
                                       std::optional<JointVec> init_q) const {
  PathVerifyReport rep;
  rep.path_id = path.path_id;
  rep.name = path.name;
  rep.speed_mm_s = opt_.speed_mm_s;
  rep.step_size_mm = opt_.step_mm;
  for (int i = 0; i < 6; ++i) {
    rep.max_joint_velocities_deg_s[i] = kin_.limits().max_vel_deg_s[i];
  }

  if (path.points.empty()) {
    rep.status = "FAILED";
    Issue iss;
    iss.severity = "ERROR";
    iss.type = "EMPTY_PATH";
    iss.detail = "Path contains 0 waypoints";
    rep.issues.push_back(iss);
    return rep;
  }

  const auto dense = interp_.Interpolate(path.points);
  rep.total_interpolated = static_cast<int>(dense.size());
  // ⑤-A 逐段(航点 i→i+1)最小速度缩放，用于生成执行侧建议速度剖面。
  std::vector<double> seg_min_scale;
  if (opt_.singularity_scaling && path.points.size() > 1)
    seg_min_scale.assign(path.points.size() - 1, 1.0);
  bool prev[3] = {false, false, false};

  const JointVec q_ref = init_q.has_value() ? *init_q : DefaultSeed();
  std::optional<JointVec> curr;
  const Transform T_urdf_start = ToUrdfFlange(dense[0].T_gun);

  // 自动选取无奇异风险（腕部/肘部正常）且距种子最近的健康分支，杜绝盲选入奇异死区
  JointVec sols[8];
  const int n_sols = kin_.Ik(T_urdf_start, sols);
  double best_dist_safe = 1e300;
  double best_dist_any = 1e300;
  std::optional<JointVec> best_safe;
  std::optional<JointVec> best_any;
  for (int i = 0; i < n_sols; ++i) {
    const auto candidate = kin_.NearestInLimits(sols[i], q_ref);
    if (!candidate) continue;
    const JointVec& q_cand = *candidate;

    const double dist = (q_cand - q_ref).squaredNorm();
    if (dist < best_dist_any) {
      best_dist_any = dist;
      best_any = q_cand;
    }
    const SingularityFlags risk = kin_.CheckSingularity(q_cand, T_urdf_start);
    if (!risk.is_singular() && dist < best_dist_safe) {
      best_dist_safe = dist;
      best_safe = q_cand;
    }
  }
  curr = best_safe.has_value() ? best_safe : best_any;
  if (!curr) {
    const Eigen::Vector3d loc = LocXyz(dense[0].T_gun);
    Issue iss;
    iss.severity = "ERROR";
    iss.type = "UNREACHABLE_START";
    iss.detail = "Start waypoint has no valid IK solutions within joint limits.";
    iss.location_xyz_mm = loc;
    iss.step_index = 0;
    iss.segment_index = 0;
    rep.issues.push_back(iss);
    rep.status = "FAILED";
    return rep;
  }

  Diagnose(*curr, dense[0].T_gun, 0, 0, rep.issues, prev, true);
  rep.trajectory_q.push_back(*curr);

  std::vector<Eigen::Matrix<double, 6, 1>> vels;
  for (size_t step = 1; step < dense.size(); ++step) {
    const DenseStep& pt = dense[step];
    const Eigen::Vector3d loc = LocXyz(pt.T_gun);
    auto next = kin_.BestIk(ToUrdfFlange(pt.T_gun), *curr);
    if (!next && pt.is_jump) {
      next = kin_.BestIk(ToUrdfFlange(pt.T_gun), q_ref);
    }
    if (!next) {
      PushIssue(rep.issues, "UNREACHABLE_STEP", "ERROR", pt.segment_index,
                static_cast<int>(step),
                "Step " + std::to_string(step) + " (segment " +
                    std::to_string(pt.segment_index) +
                    ") has no valid IK within joint limits.",
                loc);
      break;
    }

    const JointVec dq = *next - *curr;
    double max_abs_deg = 0.0;
    for (int j = 0; j < 6; ++j) max_abs_deg = std::max(max_abs_deg, std::abs(Deg(dq[j])));
    if (!pt.is_jump && max_abs_deg > kBranchJumpDeg) {
      std::ostringstream d;
      d << std::fixed << std::setprecision(1)
        << "Branch jump detected (max Δq=" << max_abs_deg
        << "°). Near singularity or joint limit.";
      PushIssue(rep.issues, "KINEMATIC_DISCONTINUITY", "ERROR", pt.segment_index,
                static_cast<int>(step), d.str(), loc);
    }

    const SingularityFlags risk = Diagnose(*next, pt.T_gun, static_cast<int>(step),
                                           pt.segment_index, rep.issues, prev, false);

    // ⑤-A 奇异自适应降速：按本步关节构型的 |sin(J5)| 折算线速度缩放系数(<1=减速)，
    // 使近奇异段的关节角速度按执行侧真实降速后的值判定，而非名义线速度下的虚高值。
    double speed_scale = 1.0;
    if (opt_.singularity_scaling && !pt.is_jump) {
      speed_scale = SingularitySpeedScale((*next)[4], Rad(opt_.singularity_ref_deg),
                                          opt_.singularity_min_scale);
      const int sg = pt.segment_index;
      if (sg >= 0 && sg < static_cast<int>(seg_min_scale.size()))
        seg_min_scale[sg] = std::min(seg_min_scale[sg], speed_scale);
    }

    if (!pt.is_jump && pt.dt_sec > 1e-9) {
      Eigen::Matrix<double, 6, 1> vel_deg;
      bool over = false;
      std::ostringstream bad;
      bool first_bad = true;
      for (int j = 0; j < 6; ++j) {
        vel_deg[j] = std::abs(Deg(dq[j] / pt.dt_sec)) * speed_scale;
        if (vel_deg[j] > kin_.limits().max_vel_deg_s[j]) {
          over = true;
          if (!first_bad) bad << ", ";
          first_bad = false;
          bad << "J" << (j + 1) << ":" << Fmt1(vel_deg[j]) << "°/s";
        }
      }
      vels.push_back(vel_deg);
      if (over) {
        const char* sev = risk.is_singular() ? "ERROR" : "WARNING";
        std::ostringstream d;
        d << "Joint overspeed detected: " << bad.str() << " (max: [";
        for (int j = 0; j < 6; ++j) {
          if (j) d << ", ";
          d << kin_.limits().max_vel_deg_s[j];
        }
        d << "]";
        if (risk.is_singular()) d << "; near singularity";
        d << ")";
        PushIssue(rep.issues, "JOINT_OVERSPEED", sev, pt.segment_index,
                  static_cast<int>(step), d.str(), loc);
      }
    }
    curr = next;
    rep.trajectory_q.push_back(*curr);
  }

  // ⑤-A 落盘逐航点建议速度：非奇异航点=名义线速度，近奇异航点按其所在段最小缩放压低。
  if (opt_.singularity_scaling && path.points.size() > 1) {
    const size_t n_wp = path.points.size();
    rep.waypoint_speed_mm_s.assign(n_wp, opt_.speed_mm_s);
    for (size_t i = 0; i + 1 < n_wp; ++i) {
      rep.waypoint_speed_mm_s[i] =
          std::round(opt_.speed_mm_s * seg_min_scale[i] * 10.0) / 10.0;
    }
    rep.waypoint_speed_mm_s[n_wp - 1] = rep.waypoint_speed_mm_s[n_wp - 2];
  }

  for (size_t i = 0; i < rep.trajectory_q.size() && i < dense.size(); ++i) {
    Eigen::Vector3d xyz, rpy;
    PoseToCtrlMmDeg(dense[i].T_gun, xyz, rpy);
    std::array<double, 6> tcp{};
    tcp[0] = std::round(xyz[0] * 100.0) / 100.0;
    tcp[1] = std::round(xyz[1] * 100.0) / 100.0;
    tcp[2] = std::round(xyz[2] * 100.0) / 100.0;
    tcp[3] = std::round(rpy[0] * 100.0) / 100.0;
    tcp[4] = std::round(rpy[1] * 100.0) / 100.0;
    tcp[5] = std::round(rpy[2] * 100.0) / 100.0;
    rep.trajectory_tcp.push_back(tcp);
  }

  bool has_err = false, has_warn = false;
  for (const auto& iss : rep.issues) {
    if (iss.severity == "ERROR") has_err = true;
    if (iss.severity == "WARNING") has_warn = true;
  }
  rep.status = has_err ? "FAILED" : (has_warn ? "WARNING" : "PASS");

  Eigen::Matrix<double, 6, 1> peak = Eigen::Matrix<double, 6, 1>::Zero();
  if (!vels.empty()) {
    for (const auto& v : vels)
      for (int j = 0; j < 6; ++j) peak[j] = std::max(peak[j], v[j]);
    double max_ratio = 0.0;
    for (int j = 0; j < 6; ++j) {
      max_ratio = std::max(max_ratio, peak[j] / kin_.limits().max_vel_deg_s[j]);
    }
    if (max_ratio > 1.0) {
      rep.recommended_safe_speed_mm_s = std::round(opt_.speed_mm_s / max_ratio * 0.9 * 10.0) / 10.0;
    } else {
      rep.recommended_safe_speed_mm_s = opt_.speed_mm_s;
    }
  } else {
    rep.recommended_safe_speed_mm_s = opt_.speed_mm_s;
  }
  for (int j = 0; j < 6; ++j) {
    rep.peak_joint_speeds_deg_s[j] = std::round(peak[j] * 10.0) / 10.0;
  }
  return rep;
}

VerifyReport ChainVerifier::VerifyAll(const std::vector<PathItem>& paths,
                                      std::optional<JointVec> init_q) const {
  VerifyReport all;
  all.nominal_speed_mm_s = opt_.speed_mm_s;
  all.slerp_step_mm = opt_.step_mm;
  all.urdf_tcp = tool_;
  for (int i = 0; i < 6; ++i) {
    all.max_joint_velocities_deg_s[i] = kin_.limits().max_vel_deg_s[i];
  }

  std::optional<JointVec> last_q = init_q;
  std::string overall = "PASS";
  for (const auto& path : paths) {
    auto rep = Verify(path, last_q);
    all.summary.total_waypoints += static_cast<int>(path.points.size());
    all.summary.total_steps += rep.total_interpolated;
    all.summary.total_issues += static_cast<int>(rep.issues.size());
    if (rep.status == "FAILED") overall = "FAILED";
    else if (rep.status == "WARNING" && overall != "FAILED") overall = "WARNING";
    for (const auto& iss : rep.issues) {
      if (iss.type.find("SINGULARITY") != std::string::npos) ++all.summary.singularity_count;
      else if (iss.type.find("OVERSPEED") != std::string::npos) ++all.summary.overspeed_count;
      else if (iss.type.find("UNREACHABLE") != std::string::npos ||
               iss.type == "KINEMATIC_DISCONTINUITY" || iss.type == "JOINT_LIMIT") {
        ++all.summary.unreachable_count;
      }
    }
    if (!rep.trajectory_q.empty()) last_q = rep.trajectory_q.back();
    all.path_reports.push_back(std::move(rep));
  }
  all.summary.status = overall;
  all.summary.total_paths = static_cast<int>(paths.size());
  return all;
}

}  // namespace motion

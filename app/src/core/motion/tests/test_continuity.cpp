#include "motion/segment.hpp"

#include <cmath>
#include <iostream>

using namespace motion;

int main() {
  int failed = 0;
  auto check = [&](bool ok, const char* name) {
    if (!ok) { std::cerr << "FAIL " << name << '\n'; ++failed; }
  };
  Cr5Kinematics kin;
  JointVec reference = (JointVec() << Rad(350), Rad(-74), Rad(-53),
                        Rad(-179), Rad(-7), Rad(18)).finished();
  JointVec solution = reference;
  solution[0] = Rad(-9);
  solution[3] = Rad(179);
  const auto nearest = kin.NearestInLimits(solution, reference);
  check(nearest.has_value(), "valid equivalent exists");
  if (nearest) {
    check(std::abs(Deg((*nearest)[0] - reference[0]) - 1.0) < 1e-8,
          "wide J1 keeps nearest valid turn even when J4 cannot wrap");
    check(std::abs(Deg((*nearest)[3] - reference[3]) - 358.0) < 1e-8,
          "bounded J4 must travel 358 degrees");
  }

  // Same IK family crosses the bounded J4 seam. The old checker wrapped the
  // real 358-degree change into 2 degrees and accepted this MoveL.
  reference[0] = Rad(65);
  JointVec end = reference;
  end[3] = Rad(-181);
  Eigen::Vector3d xyz0, rpy0, xyz1, rpy1;
  kin.FkController(reference, xyz0, rpy0);
  kin.FkController(end, xyz1, rpy1);
  MoveLQuery query;
  query.p_start_m = xyz0 / kMmPerM;
  query.p_end_m = xyz1 / kMmPerM;
  query.quat1_xyzw = Eigen::Quaterniond(RotFromCtrlRpyDeg(rpy0)).coeffs();
  query.quat2_xyzw = Eigen::Quaterniond(RotFromCtrlRpyDeg(rpy1)).coeffs();
  query.q_start = reference;
  query.q_branch_end = end;
  query.alphas = {1.0};
  check(!SegmentChecker(kin).Walk(query), "reject bounded-joint seam crossing");

  // Each sample's dt/segment/jump describes the motion arriving at that
  // sample, including the final endpoint and a transition out of a jump.
  ToolOffset tool;
  std::vector<Waypoint> points(4);
  for (auto& point : points) { point.tcp_pose = Transform::Identity(); point.spraying = true; }
  points[1].tcp_pose.translation().x() = 0.004;
  points[1].is_jump = true;
  points[2].tcp_pose.translation().x() = 0.005;
  points[3].tcp_pose.translation().x() = 0.008;
  const auto dense = Interpolator(tool, 2.0, 100.0).Interpolate(points);
  check(dense.size() == 6, "sample count");
  if (dense.size() == 6) {
    check(dense[2].segment_index == 0 && dense[2].is_jump &&
          std::abs(dense[2].dt_sec - 0.02) < 1e-12, "jump endpoint uses incoming interval");
    check(dense[3].segment_index == 1 && !dense[3].is_jump &&
          std::abs(dense[3].dt_sec - 0.01) < 1e-12, "short spray segment timing");
    check(dense.back().segment_index == 2 && !dense.back().is_jump &&
          std::abs(dense.back().dt_sec - 0.015) < 1e-12, "final interval is not fixed 50ms");
  }

  // Analytical constant-rate wrist rotation: a stationary flange rotated
  // about J6 by 4 degrees in 0.02 s has a 200 deg/s peak, not an average
  // derived from only the translation distance.
  reference = (JointVec() << 0, 0, -kPi/2, -kPi/2, -kPi/2, 0).finished();
  end = reference; end[5] = Rad(4);
  kin.FkController(reference, xyz0, rpy0);
  kin.FkController(end, xyz1, rpy1);
  query.p_start_m = xyz0 / kMmPerM;
  query.p_end_m = xyz1 / kMmPerM;
  query.quat1_xyzw = Eigen::Quaterniond(RotFromCtrlRpyDeg(rpy0)).coeffs();
  query.quat2_xyzw = Eigen::Quaterniond(RotFromCtrlRpyDeg(rpy1)).coeffs();
  query.q_start = reference;
  query.q_branch_end = end;
  query.alphas = {0.5, 1.0};
  query.duration_sec = 0.02;
  const auto walk = SegmentChecker(kin).Walk(query);
  check(walk.has_value(), "regular wrist rotation is reachable");
  if (walk) check(std::abs(walk->peak_vel_deg_s[5] - 200.0) < 1e-6, "per-step velocity in deg/s");
  return failed ? 1 : 0;
}

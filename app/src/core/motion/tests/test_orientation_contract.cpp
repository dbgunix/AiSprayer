#include "motion/io.hpp"
#include "motion/kinematics.hpp"
#include "motion/optimizer.hpp"
#include "motion/robot_model.hpp"
#include "motion/verifier.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#ifndef REPO_ROOT
#define REPO_ROOT "."
#endif

using namespace motion;

namespace {

int g_fail = 0;

#define CHECK(cond)                                                                    \
  do {                                                                                 \
    if (!(cond)) {                                                                     \
      std::cerr << "FAIL " << __FILE__ << ":" << __LINE__ << " " << #cond << "\n";     \
      ++g_fail;                                                                        \
    }                                                                                  \
  } while (0)

double MaxPointing(const PathItem& raw, const PathItem& optimized) {
  double max_deg = 0.0;
  for (size_t i = 0; i < raw.points.size(); ++i) {
    max_deg = std::max(max_deg, PointingDeg(raw.points[i].tcp_pose.linear(),
                                             optimized.points[i].tcp_pose.linear()));
  }
  return max_deg;
}

double MaxRawEnvelopeOverflow(const PathItem& raw, const PathItem& optimized,
                              const Eigen::Vector3d& tol_deg) {
  double overflow = 0.0;
  for (size_t i = 0; i < raw.points.size(); ++i) {
    Eigen::Vector3d rel = CtrlRpyDegFromRot(raw.points[i].tcp_pose.linear().transpose() *
                                            optimized.points[i].tcp_pose.linear());
    for (int axis = 0; axis < 3; ++axis) {
      overflow = std::max(overflow, std::abs(Wrap180(rel[axis])) - std::abs(tol_deg[axis]));
    }
  }
  return overflow;
}

OptimizeOptions Options() {
  OptimizeOptions opt;
  opt.grid_x = {-15.0, 15.0, 2.0};
  opt.grid_y = {-15.0, 15.0, 2.0};
  opt.grid_z = {-30.0, 30.0, 5.0};
  opt.exec_speed_mm_s = 150.0;
  opt.singularity_scaling = true;
  opt.singularity_ref_deg = 25.0;
  opt.singularity_min_scale = 0.2;
  return opt;
}

}  // namespace

int main() {
  const std::string root = REPO_ROOT;
  RobotModel model;
  std::string error;
  CHECK(LoadRobotModelFromUrdf(root + "/app/urdf/cr5_robot_with_my_tools.urdf",
                               "gripper_tip_link", model, &error));
  if (g_fail) return 1;

  Cr5Kinematics kin(model.limits);
  VerifyOptions vopt;
  vopt.step_mm = 2.0;
  vopt.speed_mm_s = 150.0;
  vopt.singularity_scaling = true;
  vopt.singularity_ref_deg = 25.0;
  vopt.singularity_min_scale = 0.2;
  ChainVerifier verifier(kin, model.tool, vopt);

  PathDocument path_092632;
  PathDocument path_104336;
  CHECK(LoadPathYaml(root + "/data/template_group/2026-09-12_092632/scan.auto.path.yaml",
                     path_092632, &error));
  CHECK(LoadPathYaml(root + "/data/template_group/2026-09-12_104336/scan.auto.path.yaml",
                     path_104336, &error));
  CHECK(path_092632.paths.size() == 1 && path_104336.paths.size() == 1);
  if (g_fail) return 1;

  AnchorSpec raw_spec;
  raw_spec.source = "raw";
  raw_spec.tol_deg = {10.0, 10.0, 180.0};

  // 092632 has a continuous IK branch at its untouched orientations.  The
  // optimizer must recognise it instead of manufacturing tilt.
  const PathItem& raw_092632 = path_092632.paths.front();
  const Anchor raw_anchor_092632 = ResolveAnchor(raw_spec, kin, raw_092632);
  const OptimizeResult kept =
      ViterbiOptimizer(kin, model.tool, Options(), &verifier).Optimize(raw_092632, raw_anchor_092632);
  CHECK(kept.verify.status == "PASS");
  CHECK(MaxPointing(raw_092632, kept.path) < 0.02);  // YAML stores RPY to 0.01 degree.
  CHECK(MaxRawEnvelopeOverflow(raw_092632, kept.path, raw_spec.tol_deg) < 1e-6);

  // 104336 needs an in-envelope attitude adjustment near waypoint 51.  The
  // repaired chain must still pass and stay inside the requested raw envelope.
  const PathItem& raw_104336 = path_104336.paths.front();
  const Anchor raw_anchor_104336 = ResolveAnchor(raw_spec, kin, raw_104336);
  const OptimizeResult repaired =
      ViterbiOptimizer(kin, model.tool, Options(), &verifier).Optimize(raw_104336, raw_anchor_104336);
  CHECK(repaired.verify.status == "PASS");
  CHECK(MaxPointing(raw_104336, repaired.path) <= 10.1);
  CHECK(MaxRawEnvelopeOverflow(raw_104336, repaired.path, raw_spec.tol_deg) < 1e-6);

  // A zero envelope is an exact-pose request.  It must reject the same raw
  // waypoint rather than silently applying the recovery tilt.
  AnchorSpec locked_spec = raw_spec;
  locked_spec.tol_deg = {0.0, 0.0, 0.0};
  bool rejected = false;
  try {
    const Anchor locked = ResolveAnchor(locked_spec, kin, raw_104336);
    (void)ViterbiOptimizer(kin, model.tool, Options(), &verifier).Optimize(raw_104336, locked);
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  CHECK(rejected);

  if (g_fail) {
    std::cerr << g_fail << " orientation-contract checks failed\n";
    return 1;
  }
  std::cout << "test_orientation_contract OK\n";
  return 0;
}

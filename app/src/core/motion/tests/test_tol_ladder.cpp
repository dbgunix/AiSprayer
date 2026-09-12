// 容差阶梯择优（Monotonicity Guard）验收测试。
//
// 背景：包络 [30,30,180] 在几何上包含 [10,10,50]，理论上最优解不该变差；但 DP 的目标
// J = Σ Δq² + 姿态偏置不含峰值角速度，且候选集/beam 剪枝都随容差变化，实测同一工件
// [10,10,50] → 43.6°/s 而 [30,30,180] → 133.8°/s（详见 docs/
// optimizer_monotonicity_improvement_proposal.md §5–§6）。
//
// 本测试锁住修复后的性质：
//   P1 阶梯里每一档包络都 ⊆ 请求包络（逐分量），且采纳解的每个航点姿态都落在采纳包络内
//      —— 这是"择优合法"的前提；
//   P2 在同一校验等级内，采纳档优先最小化喷嘴对原始法向的偏差；
//   P3 指向偏量护栏开启时，不得用喷嘴偏离法向去换峰值速度。
#include "motion/io.hpp"
#include "motion/kinematics.hpp"
#include "motion/optimizer.hpp"
#include "motion/robot_model.hpp"
#include "motion/verifier.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#ifndef REPO_ROOT
#define REPO_ROOT "."
#endif

using namespace motion;

static int g_fail = 0;
#define CHECK(cond)                                                                    \
  do {                                                                                 \
    if (!(cond)) {                                                                     \
      std::cerr << "FAIL " << __FILE__ << ":" << __LINE__ << " " << #cond << "\n";     \
      ++g_fail;                                                                        \
    }                                                                                  \
  } while (0)

namespace {

double PeakMax(const PathVerifyReport& r) {
  return *std::max_element(r.peak_joint_speeds_deg_s.begin(), r.peak_joint_speeds_deg_s.end());
}

// 采纳解是否真的待在它声称的包络里：逐航点算相对锚点的欧拉偏角并逐分量比较。
double MaxEnvelopeOverflow(const PathItem& optimized, const Eigen::Matrix3d& R_anchor,
                           const Eigen::Vector3d& tol_deg) {
  double worst = 0.0;
  for (const auto& wp : optimized.points) {
    Eigen::Vector3d rel = CtrlRpyDegFromRot(R_anchor.transpose() * wp.tcp_pose.linear());
    for (int i = 0; i < 3; ++i) {
      rel[i] = Wrap180(rel[i]);
      worst = std::max(worst, std::abs(rel[i]) - std::abs(tol_deg[i]));
    }
  }
  return worst;
}

}  // namespace

int main() {
  const std::string root = REPO_ROOT;
  const std::string urdf = root + "/app/urdf/cr5_robot_with_my_tools.urdf";
  const std::string auto_yaml = root + "/data/template_group/2026-09-06_200125/scan.auto.path.yaml";
  if (!std::ifstream(auto_yaml).good()) {
    std::cout << "test_tol_ladder SKIP: 缺少测试数据 " << auto_yaml << "\n";
    return 0;
  }

  RobotModel model;
  std::string err;
  if (!LoadRobotModelFromUrdf(urdf, "gripper_tip_link", model, &err)) {
    std::cerr << err << "\n";
    return 1;
  }
  PathDocument doc;
  if (!LoadPathYaml(auto_yaml, doc, &err) || doc.paths.empty()) {
    std::cerr << "load failed: " << err << "\n";
    return 1;
  }
  const PathItem& path = doc.paths[0];
  std::cout << "waypoints=" << path.points.size() << "\n";

  Cr5Kinematics kin(model.limits);
  VerifyOptions vopt;
  vopt.step_mm = 2.0;      // 与 spraying.slerp_step_mm 一致
  vopt.speed_mm_s = 150.0;  // 与 spraying.velocity 一致
  ChainVerifier verifier(kin, model.tool, vopt);

  OptimizeOptions base;
  base.grid_x = {-5, 5, 2};
  base.grid_y = {-5, 5, 2};
  base.grid_z = {-30, 30, 5};
  base.dense_verify = true;
  CHECK(base.Validate().empty());

  AnchorSpec spec;
  spec.source = "config";
  spec.ref_rpy_deg = {90.0, 0.0, 90.0};
  spec.tol_deg = {30.0, 30.0, 180.0};
  // 锚点姿态与容差无关，解一次就够（后面只用它的 R）。
  const Anchor anchor = ResolveAnchor(spec, kin, path);
  CHECK(anchor.has_global());
  const Eigen::Vector3d kRequestedTol = spec.tol_deg;
  const Eigen::Vector3d kRefTol{10.0, 10.0, 50.0};

  auto run = [&](const Eigen::Vector3d& tol, bool ladder) -> OptimizeResult {
    OptimizeOptions o = base;
    o.tol_ladder = ladder;
    Anchor a = anchor;
    a.tol_deg = tol;
    return ViterbiOptimizer(kin, model.tool, o, &verifier).Optimize(path, a);
  };

  // 参考档：用户手调的小容差（单档，旧行为）。
  const OptimizeResult ref = run(kRefTol, false);
  // 请求档：大容差，关阶梯 = 修复前的行为。
  const OptimizeResult loose_off = run(kRequestedTol, false);
  // 请求档：大容差，开阶梯 = 修复后的行为。
  const OptimizeResult loose_on = run(kRequestedTol, true);

  const double ref_peak = PeakMax(ref.verify);
  const double off_peak = PeakMax(loose_off.verify);
  const double on_peak = PeakMax(loose_on.verify);
  std::cout << "小容差 [10,10,50]        : status=" << ref.verify.status << " peak=" << ref_peak
            << "°/s J=" << ref.objective << "\n"
            << "大容差 [30,30,180] 关阶梯: status=" << loose_off.verify.status << " peak=" << off_peak
            << "°/s J=" << loose_off.objective << "\n"
            << "大容差 [30,30,180] 开阶梯: status=" << loose_on.verify.status << " peak=" << on_peak
            << "°/s J=" << loose_on.objective << " 采纳包络=[" << loose_on.adopted_tol_deg.transpose()
            << "] 档数=" << loose_on.ladder.size() << "\n";

  CHECK(ref.verify.status == "PASS");
  CHECK(loose_off.verify.status == "PASS");
  CHECK(loose_on.verify.status == "PASS");

  // P1a 阶梯确实跑了多档；法向更差的紧档可以被拒绝，所以不要求它一定被采纳。
  CHECK(loose_on.ladder.size() >= 2);
  // P1b 每一档都 ⊆ 请求包络（逐分量）；第一档就是请求档本身。
  CHECK((loose_on.ladder.front().tol_deg - kRequestedTol).norm() < 1e-12);
  for (const auto& rung : loose_on.ladder) {
    for (int i = 0; i < 3; ++i) CHECK(rung.tol_deg[i] <= kRequestedTol[i] + 1e-9);
  }
  // P1c 采纳解的每个航点姿态都真的落在采纳包络内（溢出量应为 0）。
  const double overflow = MaxEnvelopeOverflow(loose_on.path, *anchor.R, loose_on.adopted_tol_deg);
  std::cout << "采纳包络溢出量 max(|rel|-tol) = " << overflow << "°\n";
  CHECK(overflow < 1e-6);

  // P2 满足相同校验等级时，法向保真优先于峰值速度。这里不能再要求“更紧包络
  // 一定更快”：那正是旧实现用牺牲喷涂方向换来的错误目标。
  double best_pointing = std::numeric_limits<double>::infinity();
  for (const auto& rung : loose_on.ladder) {
    if (rung.status == loose_on.verify.status) {
      best_pointing = std::min(best_pointing, rung.max_pointing_deg);
    }
  }
  CHECK(std::isfinite(best_pointing));
  double adopted_pointing = 0.0;
  for (const auto& rung : loose_on.ladder) {
    if ((rung.tol_deg - loose_on.adopted_tol_deg).norm() < 1e-9) {
      adopted_pointing = rung.max_pointing_deg;
      break;
    }
  }
  CHECK(adopted_pointing <= best_pointing + 1e-9);

  // P5 指向偏量护栏：收紧包络会把喷嘴拉离表面法向（实测 18.7° → 45.9°）。
  // 开了护栏后这些档应被弃用，采纳档退回请求档（宁可峰值高也不要牺牲涂层质量）。
  OptimizeOptions guarded = base;
  guarded.tol_ladder = true;
  guarded.tol_ladder_max_pointing_deg = 1.0;
  CHECK(guarded.Validate().empty());
  {
    Anchor a = anchor;
    a.tol_deg = kRequestedTol;
    const OptimizeResult r = ViterbiOptimizer(kin, model.tool, guarded, &verifier).Optimize(path, a);
    const double req_pointing = r.ladder.empty() ? 0.0 : r.ladder.front().max_pointing_deg;
    std::cout << "护栏开启(≤1°): 采纳包络=[" << r.adopted_tol_deg.transpose() << "] 峰值="
              << PeakMax(r.verify) << "°/s 请求档指向偏量=" << req_pointing << "°\n";
    CHECK(r.verify.status == "PASS");
    CHECK((r.adopted_tol_deg - kRequestedTol).norm() < 1e-9);  // 未被护栏放行的档全部弃用
  }

  if (g_fail) {
    std::cerr << g_fail << " tolerance-ladder checks failed\n";
    return 1;
  }
  std::cout << "test_tol_ladder OK\n";
  return 0;
}

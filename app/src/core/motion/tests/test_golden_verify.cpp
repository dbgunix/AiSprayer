#include "motion/io.hpp"
#include "motion/kinematics.hpp"
#include "motion/robot_model.hpp"
#include "motion/verifier.hpp"

#include <cmath>
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

int main() {
  const std::string urdf = std::string(REPO_ROOT) + "/app/urdf/cr5_robot_with_my_tools.urdf";
  const std::string yaml =
      std::string(REPO_ROOT) + "/data/template_group/2026-09-03_225937/scan.auto.path.yaml";

  RobotModel model;
  std::string err;
  if (!LoadRobotModelFromUrdf(urdf, "gripper_tip_link", model, &err)) {
    std::cerr << err << "\n";
    return 1;
  }
  CHECK(model.tool.has_tool);
  CHECK(model.tool.tool_name == "gripper_tip_link");
  CHECK(std::abs(model.tool.xyz_mm[0] + 3.36) < 1e-9);
  CHECK(std::abs(model.tool.xyz_mm[1] + 22.22) < 1e-9);
  CHECK(std::abs(model.tool.xyz_mm[2] - 251.67) < 1e-9);
  CHECK(std::abs(model.tool.rpy_deg[0] - 2.47) < 1e-9);
  CHECK(std::abs(model.tool.rpy_deg[1] - 2.29) < 1e-9);
  CHECK(std::abs(model.tool.rpy_deg[2] - 175.11) < 1e-9);
  CHECK(std::abs(model.limits.max_vel_deg_s[0] - 179.91) < 1e-9);

  PathDocument doc;
  if (!LoadPathYaml(yaml, doc, &err)) {
    std::cerr << err << "\n";
    return 1;
  }
  CHECK(doc.paths.size() == 1);
  CHECK(doc.paths[0].points.size() == 81);

  Cr5Kinematics kin(model.limits);
  VerifyOptions opt;
  opt.step_mm = 1.5;
  opt.speed_mm_s = 120.0;
  ChainVerifier v(kin, model.tool, opt);
  const VerifyReport report = v.VerifyAll(doc.paths);

  std::cout << "status=" << report.summary.status
            << " steps=" << report.summary.total_steps
            << " issues=" << report.summary.total_issues << "\n";
  if (!report.path_reports.empty()) {
    std::cout << "peak=[";
    for (int i = 0; i < 6; ++i) {
      if (i) std::cout << ",";
      std::cout << report.path_reports[0].peak_joint_speeds_deg_s[i];
    }
    std::cout << "]\n";
    if (!report.path_reports[0].trajectory_q.empty()) {
      const auto& q0 = report.path_reports[0].trajectory_q.front();
      std::cout << "first_q=[" << q0.transpose() << "]\n";
    }
  }

  CHECK(report.summary.status == "PASS");
  CHECK(report.summary.total_paths == 1);
  CHECK(report.summary.total_waypoints == 81);
  // 下面的黄金常量锁定「校验链路（ChainVerifier + IK + MoveL 插值）在当前工件上的位级行为」。
  // 注意：输入 scan.auto.path.yaml 是 gitignore 的流水线再生数据，一旦被重新生成，
  // 这些常量就会失配（本测试失败）——此时应确认新工件 status=PASS/issues=0 后重新采集下方数值。
  CHECK(report.summary.total_steps == 5384);
  CHECK(report.summary.total_issues == 0);
  CHECK(!report.path_reports.empty());

  // Correct incoming-interval timing removes artificial peaks at segment boundaries.
  const double expect_peak[6] = {19.4, 27.7, 26.4, 145.7, 31.6, 122.5};
  for (int i = 0; i < 6; ++i) {
    const double got = report.path_reports[0].peak_joint_speeds_deg_s[i];
    if (std::abs(got - expect_peak[i]) > 0.15) {
      std::cerr << "peak J" << (i + 1) << " got " << got << " expect " << expect_peak[i] << "\n";
      ++g_fail;
    }
  }

  const double expect_q0[6] = {1.02779, -0.248253, -1.42254, -1.3748, -0.620557, -1.5595};
  if (!report.path_reports[0].trajectory_q.empty()) {
    const JointVec& q0 = report.path_reports[0].trajectory_q.front();
    for (int i = 0; i < 6; ++i) {
      if (std::abs(q0[i] - expect_q0[i]) > 2e-3) {
        std::cerr << "first_q J" << (i + 1) << " got " << q0[i] << " expect " << expect_q0[i]
                  << "\n";
        ++g_fail;
      }
    }
  } else {
    ++g_fail;
  }

  if (g_fail) {
    std::cerr << g_fail << " golden checks failed\n";
    return 1;
  }
  std::cout << "test_golden_verify OK\n";
  return 0;
}

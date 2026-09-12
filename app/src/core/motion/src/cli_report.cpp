#include "cli_report.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <ostream>
#include <sstream>
#include <vector>

namespace motion {
namespace {

int CodepointWidth(uint32_t cp) {
  if (cp < 0x80) return 1;
  if ((cp >= 0x1100 && cp <= 0x115F) || (cp >= 0x2E80 && cp <= 0xA4CF) ||
      (cp >= 0xAC00 && cp <= 0xD7A3) || (cp >= 0xF900 && cp <= 0xFAFF) ||
      (cp >= 0xFE10 && cp <= 0xFE6F) || (cp >= 0xFF00 && cp <= 0xFF60) ||
      (cp >= 0xFFE0 && cp <= 0xFFE6) || (cp >= 0x3000 && cp <= 0x303F) ||
      (cp >= 0x3040 && cp <= 0x30FF) || (cp >= 0x4E00 && cp <= 0x9FFF) ||
      (cp >= 0x3400 && cp <= 0x4DBF) || (cp >= 0x2500 && cp <= 0x259F) ||
      (cp >= 0x1F300 && cp <= 0x1FAFF)) {
    return 2;
  }
  return 1;
}

int DispLen(const std::string& s) {
  int w = 0;
  for (size_t i = 0; i < s.size();) {
    const unsigned char c = static_cast<unsigned char>(s[i]);
    uint32_t cp = 0;
    int n = 1;
    if (c < 0x80) {
      cp = c;
    } else if ((c & 0xE0) == 0xC0 && i + 1 < s.size()) {
      cp = (c & 0x1F) << 6 | (static_cast<unsigned char>(s[i + 1]) & 0x3F);
      n = 2;
    } else if ((c & 0xF0) == 0xE0 && i + 2 < s.size()) {
      cp = (c & 0x0F) << 12 | (static_cast<unsigned char>(s[i + 1]) & 0x3F) << 6 |
           (static_cast<unsigned char>(s[i + 2]) & 0x3F);
      n = 3;
    } else if ((c & 0xF8) == 0xF0 && i + 3 < s.size()) {
      cp = (c & 0x07) << 18 | (static_cast<unsigned char>(s[i + 1]) & 0x3F) << 12 |
           (static_cast<unsigned char>(s[i + 2]) & 0x3F) << 6 |
           (static_cast<unsigned char>(s[i + 3]) & 0x3F);
      n = 4;
    } else {
      cp = c;
    }
    w += CodepointWidth(cp);
    i += static_cast<size_t>(n);
  }
  return w;
}

std::string Pad(const std::string& s, int width, const char* align) {
  const int pad = std::max(0, width - DispLen(s));
  if (align[0] == 'r') return std::string(static_cast<size_t>(pad), ' ') + s;
  if (align[0] == 'c') {
    const int l = pad / 2;
    return std::string(static_cast<size_t>(l), ' ') + s +
           std::string(static_cast<size_t>(pad - l), ' ');
  }
  return s + std::string(static_cast<size_t>(pad), ' ');
}

double Nz(double v) { return std::abs(v) < 5e-3 ? 0.0 : v; }

std::string Fmt(const char* fmt, double a) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), fmt, a);
  return buf;
}
std::string Fmt(const char* fmt, double a, double b, double c) {
  char buf[96];
  std::snprintf(buf, sizeof(buf), fmt, a, b, c);
  return buf;
}
std::string Fmt(const char* fmt, double a, double b, double c, double d, double e, double f) {
  char buf[160];
  std::snprintf(buf, sizeof(buf), fmt, a, b, c, d, e, f);
  return buf;
}

std::string FormatTable(const std::vector<std::string>& headers, const std::vector<int>& widths,
                        const std::vector<std::vector<std::string>>& rows,
                        const std::string& title) {
  std::ostringstream os;
  int tot = 4;
  for (size_t i = 0; i < widths.size(); ++i) tot += widths[i] + (i ? 3 : 0);
  os << std::string(static_cast<size_t>(tot), '=') << "\n";
  if (!title.empty()) {
    os << "📊 " << title << "\n";
    os << std::string(static_cast<size_t>(tot), '-') << "\n";
  }
  os << "| ";
  for (size_t i = 0; i < headers.size(); ++i) {
    if (i) os << " | ";
    os << Pad(headers[i], widths[i], i == 0 ? "c" : "l");
  }
  os << " |\n+-";
  for (size_t i = 0; i < widths.size(); ++i) {
    if (i) os << "-+-";
    os << std::string(static_cast<size_t>(widths[i]), '-');
  }
  os << "-+\n";
  for (const auto& r : rows) {
    os << "| ";
    for (size_t i = 0; i < headers.size(); ++i) {
      if (i) os << " | ";
      const std::string cell = i < r.size() ? r[i] : "";
      os << Pad(cell, widths[i], i == 0 ? "c" : "l");
    }
    os << " |\n";
  }
  os << std::string(static_cast<size_t>(tot), '=');
  return os.str();
}

Eigen::Vector3d RelRpy(const Eigen::Matrix3d& R_anchor, const Eigen::Matrix3d& R) {
  Eigen::Vector3d e = CtrlRpyDegFromRot(R_anchor.transpose() * R);
  e[0] = Wrap180(e[0]);
  e[1] = Wrap180(e[1]);
  e[2] = Wrap180(e[2]);
  return e;
}

std::vector<JointVec> TrackRawJoints(const Cr5Kinematics& kin, const PathItem& raw,
                                     const JointVec& home) {
  std::vector<JointVec> out;
  JointVec q = home;
  for (const auto& wp : raw.points) {
    auto sol = kin.BestIk(CtrlToUrdf(wp.tcp_pose), q);
    if (sol) {
      q = *sol;
      out.push_back(q);
    } else {
      out.push_back(JointVec::Constant(std::numeric_limits<double>::quiet_NaN()));
    }
  }
  return out;
}

std::string JointDegStr(const JointVec& q) {
  if (!std::isfinite(q[0])) return "IK 无解";
  return Fmt("[%5.1f, %5.1f, %5.1f, %5.1f, %5.1f, %5.1f]", Deg(q[0]), Deg(q[1]), Deg(q[2]),
             Deg(q[3]), Deg(q[4]), Deg(q[5]));
}

}  // namespace

void PrintOptimizePreamble(std::ostream& os, const std::string& input, const PathItem& path,
                           const Cr5Kinematics& kin, const AnchorSpec& spec, const Anchor& anchor,
                           const OptimizeOptions& opt, double speed_mm_s, double step_mm) {
  os << std::string(136, '=') << "\n";
  os << "📂 1. 输入数据来源: " << input << "\n";
  os << "   轨迹名称: " << path.name << " (ID: " << path.path_id
     << "), 包含航点数: " << path.points.size() << "\n";

  os << "\n⚡ 2. 运动学引擎状态:\n";
  os << "   当前解算后端: CPP\n";
  os << "   C++ 加速动态库: 已内建于 motion_cli（不再走 ctypes / libur_kin）\n";
  os << "   密插值校验: 步长 " << step_mm << " mm, 假定线速度 " << speed_mm_s << " mm/s\n";

  Eigen::Vector3d xyz, rpy;
  kin.FkController(spec.home_joints_rad, xyz, rpy);
  os << "\n📍 3. 锚点配置 (Anchor Pose via Forward Kinematics):\n";
  os << "   Home 关节角度: [";
  for (int i = 0; i < 6; ++i) {
    if (i) os << ", ";
    os << Deg(spec.home_joints_rad[i]);
  }
  os << "] deg\n";
  os << "   解算锚点位置: XYZ = [" << Fmt("%.2f", xyz[0]) << ", " << Fmt("%.2f", xyz[1]) << ", "
     << Fmt("%.2f", xyz[2]) << "] mm\n";
  if (anchor.has_global()) {
    const Eigen::Vector3d ar = CtrlRpyDegFromRot(*anchor.R);
    os << "   解算锚点姿态: RPY = [" << Fmt("%.2f", Nz(ar[0])) << ", " << Fmt("%.2f", Nz(ar[1]))
       << ", " << Fmt("%.2f", Nz(ar[2])) << "] deg (Euler 'xyz')\n";
  } else {
    os << "   解算锚点姿态: 逐点自适应名义法向姿态 (Adaptive Per-waypoint Surface Normal Envelope)\n";
  }

  os << "\n⚙️  4. 优化参数设置:\n";
  auto g = [](const AxisGrid& a) {
    return Fmt("(%.1f, %.1f, %.1f)", a.min_deg, a.max_deg, a.step_deg);
  };
  os << "   搜索网格: Tol_X=" << g(opt.grid_x) << ", Tol_Y=" << g(opt.grid_y)
     << ", Tol_Z=" << g(opt.grid_z) << "\n";
  os << "   锚点硬包络: (Tol_Rx=±" << Fmt("%.1f", anchor.tol_deg[0]) << "°, Tol_Ry=±"
     << Fmt("%.1f", anchor.tol_deg[1]) << "°, Tol_Rz=±" << Fmt("%.1f", anchor.tol_deg[2])
     << "°)\n";
  if (opt.tol_ladder && !opt.tol_ladder_scales.empty()) {
    std::ostringstream sc;
    for (size_t i = 0; i < opt.tol_ladder_scales.size(); ++i) {
      if (i) sc << ", ";
      sc << Fmt("%.3g", opt.tol_ladder_scales[i]);
    }
    os << "   容差阶梯择优: 开启（额外收紧档比例 " << sc.str() << "，早停阈值 峰值/限速 ≤ "
       << Fmt("%.0f", opt.tol_ladder_stop_peak_ratio * 100.0) << "%）\n";
  }
  os << "   Beam Width: " << opt.beam_width << ", 8支单桶容量: " << opt.max_candidates_per_branch
     << ", MoveL 抽检: [" << opt.movel_checks_min << ", " << opt.movel_checks_max << "] 点 (间距 "
     << opt.movel_spacing_mm << " mm)\n";

  os << "\n🔄 5. 正在执行 Viterbi DP 全局连续性优化 (Waypoints 数量: " << path.points.size()
     << ")...\n";
  os.flush();
}

void PrintOptimizeReport(std::ostream& os, const Cr5Kinematics& kin, const PathItem& raw,
                         const OptimizeResult& result, const Anchor& anchor,
                         const Eigen::Vector3d& tol_deg, const JointVec& home_rad,
                         const PathVerifyReport* verify, const std::string& output_path) {
  os << "   优化完成! 耗时: " << Fmt("%.2f", result.elapsed_ms)
     << " ms | 姿态已修改: " << (result.modified ? "True" : "False") << "\n";

  // 容差阶梯择优：把每一档包络的实测结果列出来并标明采纳档。
  // 单档（阶梯关闭）时 ladder 为空，报表格式与以前一致。
  if (!result.ladder.empty()) {
    const auto& req = result.ladder.front().tol_deg;
    const std::vector<std::string> hd = {"档位", "包络 ±(Rx,Ry,Rz)°", "校验",
                                        "峰值°/s", "峰值/限速", "指向偏量°", "DP代价 J", "耗时 ms"};
    const std::vector<int> wd = {8, 22, 11, 10, 10, 11, 12, 9};
    std::vector<std::vector<std::string>> rows;
    for (size_t i = 0; i < result.ladder.size(); ++i) {
      const auto& r = result.ladder[i];
      const bool adopted = (r.tol_deg - result.adopted_tol_deg).cwiseAbs().maxCoeff() < 1e-9;
      const bool err = r.status == "ERROR";
      rows.push_back({(adopted ? "★" : "") + std::to_string(i + 1),
                      Fmt("(%.1f, %.1f, %.1f)", r.tol_deg[0], r.tol_deg[1], r.tol_deg[2]),
                      r.status, err ? "-" : Fmt("%.1f", r.peak_deg_s),
                      err ? "-" : Fmt("%.1f%%", r.peak_ratio * 100.0),
                      err ? "-" : Fmt("%.2f", r.max_pointing_deg),
                      err ? r.error : Fmt("%.1f", r.objective), Fmt("%.0f", r.elapsed_ms)});
    }
    os << FormatTable(hd, wd, rows, "5.1 容差阶梯择优 (Tolerance Ladder)") << "\n";
    os << "   请求包络: ±" << Fmt("(%.1f, %.1f, %.1f)", req[0], req[1], req[2]) << "° → 采纳包络: ±"
       << Fmt("(%.1f, %.1f, %.1f)", result.adopted_tol_deg[0], result.adopted_tol_deg[1],
              result.adopted_tol_deg[2])
       << "°\n";
    os << "   说明: 每一档包络都包含于请求包络，而密集校验与容差无关，因此采纳解不会比\n"
       << "         任何一档差（择优标尺: 校验状态 → 指向护栏 → 指向偏量 → 峰值/限速 → J）。\n"
       << "         在通过校验的档位中优先保持原始法向，其次比较峰值速度，\n"
       << "         涂层质量优先时用 --tol-ladder-max-pointing-deg 限制或 --no-tol-ladder 关闭。\n";
  }

  const bool per_point = !anchor.has_global();
  const Eigen::Matrix3d R_fix = per_point ? Eigen::Matrix3d::Identity() : *anchor.R;
  const auto raw_q = TrackRawJoints(kin, raw, home_rad);
  const auto& opt_pts = result.path.points;
  const size_t n = raw.points.size();

  const std::vector<std::string> h1 = {"序号",           "位置 (X,Y,Z) mm", "原始姿态 RPY°",
                                       "相对锚点偏角", "指向偏角",       "原始关节 J1~J6 (deg)"};
  const std::vector<int> w1 = {6, 21, 21, 17, 10, 42};
  std::vector<std::vector<std::string>> r1;
  for (size_t i = 0; i < n; ++i) {
    Eigen::Vector3d xyz, rpy;
    PoseToCtrlMmDeg(raw.points[i].tcp_pose, xyz, rpy);
    const Eigen::Matrix3d R_raw = raw.points[i].tcp_pose.linear();
    const Eigen::Matrix3d R_a = per_point ? R_raw : R_fix;
    const Eigen::Vector3d rel = RelRpy(R_a, R_raw);
    const double pt = PointingDeg(R_raw, R_a);
    r1.push_back({
        "#" + std::to_string(i + 1),
        Fmt("%5.1f, %5.1f, %5.1f", xyz[0], xyz[1], xyz[2]),
        Fmt("%6.2f, %6.2f, %6.2f", rpy[0], rpy[1], rpy[2]),
        Fmt("[%+4.0f, %+4.0f, %+5.0f]", rel[0], rel[1], rel[2]),
        Fmt("%6.2f°", pt),
        i < raw_q.size() ? JointDegStr(raw_q[i]) : "IK 无解",
    });
  }
  os << "\n"
     << FormatTable(h1, w1, r1, "6.1 优化前 (原始 Raw) 航点姿态、相对锚点偏角与关节角度") << "\n";

  const std::vector<std::string> h2 = {"序号",           "位置 (X,Y,Z) mm", "优化后姿态 RPY°",
                                       "相对锚点偏角", "指向偏角",       "优化关节 J1~J6 (deg)"};
  std::vector<std::vector<std::string>> r2;
  for (size_t i = 0; i < n; ++i) {
    Eigen::Vector3d xyz, raw_rpy, opt_rpy;
    PoseToCtrlMmDeg(raw.points[i].tcp_pose, xyz, raw_rpy);
    PoseToCtrlMmDeg(opt_pts[i].tcp_pose, xyz, opt_rpy);
    PoseToCtrlMmDeg(raw.points[i].tcp_pose, xyz, raw_rpy);  // 位置列与 Python 一样取原始航点 xyz
    const Eigen::Matrix3d R_raw = raw.points[i].tcp_pose.linear();
    const Eigen::Matrix3d R_opt = opt_pts[i].tcp_pose.linear();
    const Eigen::Matrix3d R_a = per_point ? R_raw : R_fix;
    const Eigen::Vector3d rel = RelRpy(R_a, R_opt);
    const double pt = PointingDeg(R_opt, R_a);
    JointVec q = JointVec::Zero();
    if (i < result.joints_rad.size()) q = result.joints_rad[i];
    r2.push_back({
        "#" + std::to_string(i + 1),
        Fmt("%5.1f, %5.1f, %5.1f", xyz[0], xyz[1], xyz[2]),
        Fmt("%6.2f, %6.2f, %6.2f", opt_rpy[0], opt_rpy[1], opt_rpy[2]),
        Fmt("[%+4.0f, %+4.0f, %+5.0f]", rel[0], rel[1], rel[2]),
        Fmt("%6.2f°", pt),
        JointDegStr(q),
    });
  }
  os << "\n"
     << FormatTable(h2, w1, r2, "6.2 优化后 (优化 Opt) 航点姿态、相对锚点偏角与关节角度") << "\n";

  const std::vector<std::string> h3 = {"序号", "原始姿态 RPY°", "优化后姿态 RPY°", "3D指向偏量",
                                       "相对锚点偏角变化 (Raw -> Opt)", "关节偏量 Δq_max"};
  const std::vector<int> w3 = {6, 21, 21, 12, 33, 16};
  std::vector<std::vector<std::string>> r3;
  for (size_t i = 0; i < n; ++i) {
    Eigen::Vector3d xyz, raw_rpy, opt_rpy;
    PoseToCtrlMmDeg(raw.points[i].tcp_pose, xyz, raw_rpy);
    PoseToCtrlMmDeg(opt_pts[i].tcp_pose, xyz, opt_rpy);
    const Eigen::Matrix3d R_raw = raw.points[i].tcp_pose.linear();
    const Eigen::Matrix3d R_opt = opt_pts[i].tcp_pose.linear();
    const Eigen::Matrix3d R_a = per_point ? R_raw : R_fix;
    const Eigen::Vector3d rel_r = RelRpy(R_a, R_raw);
    const Eigen::Vector3d rel_o = RelRpy(R_a, R_opt);
    const double pd = PointingDeg(R_raw, R_opt);
    std::string dq = "N/A";
    if (i < raw_q.size() && i < result.joints_rad.size() && std::isfinite(raw_q[i][0])) {
      double mx = 0.0;
      for (int j = 0; j < 6; ++j) {
        mx = std::max(mx, std::abs(Wrap180(Deg(result.joints_rad[i][j]) - Deg(raw_q[i][j]))));
      }
      dq = Fmt("%5.2f°", mx);
    }
    r3.push_back({
        "#" + std::to_string(i + 1),
        Fmt("%5.1f, %5.1f, %5.1f", raw_rpy[0], raw_rpy[1], raw_rpy[2]),
        Fmt("%5.1f, %5.1f, %5.1f", opt_rpy[0], opt_rpy[1], opt_rpy[2]),
        Fmt("%6.2f°", pd),
        Fmt("[%+4.0f,%+4.0f,%+4.0f] -> [%+4.0f,%+4.0f,%+4.0f]", rel_r[0], rel_r[1], rel_r[2],
            rel_o[0], rel_o[1], rel_o[2]),
        dq,
    });
  }
  os << "\n" << FormatTable(h3, w3, r3, "6.3 优化前后综合指标对比总表 (Raw vs Opt)") << "\n";

  os << "\n💡 注解说明:\n";
  if (per_point) {
    os << "   1. [相对锚点偏角]: 表示当前姿态相对逐点名义法向锚点 (Adaptive Per-waypoint Normal)"
          "的旋转量，严格受控在容差包络 (Rx:±"
       << Fmt("%.1f", tol_deg[0]) << "°, Ry:±" << Fmt("%.1f", tol_deg[1]) << "°, Rz:±"
       << Fmt("%.1f", tol_deg[2]) << "°) 之内。\n";
    os << "   2. [相对锚点指向角]: 表示喷枪中心法向与逐点名义法向锚点喷枪法向在 3D 空间中的夹角。\n";
  } else {
    const Eigen::Vector3d ar = CtrlRpyDegFromRot(*anchor.R);
    os << "   1. [相对锚点偏角]: 表示当前姿态相对锚点 [" << Fmt("%.0f", Nz(ar[0])) << ", "
       << Fmt("%.0f", Nz(ar[1])) << ", " << Fmt("%.0f", Nz(ar[2]))
       << "]的旋转量，严格受控在容差包络 (Rx:±" << Fmt("%.1f", tol_deg[0]) << "°, Ry:±"
       << Fmt("%.1f", tol_deg[1]) << "°, Rz:±" << Fmt("%.1f", tol_deg[2]) << "°) 之内。\n";
    os << "   2. [相对锚点指向角]: 表示喷枪中心法向与锚点 [" << Fmt("%.0f", Nz(ar[0])) << ", "
       << Fmt("%.0f", Nz(ar[1])) << ", " << Fmt("%.0f", Nz(ar[2]))
       << "]喷枪法向在 3D 空间中的夹角。\n";
  }
  os << "   3. [枪尖指向偏量(3D)]: 表示优化前后喷枪法向的实际偏转角度（排除了 Euler 欧拉角在 "
        "Ry≈-90° 时的万向节死锁双重表示现象）。\n";

  if (verify) {
    os << "\n🔍 7. 全路径运动学链校验结果 (Kinematic Chain Verification):\n";
    os << "   校验状态: " << verify->status << " (总插值步数: " << verify->total_interpolated
       << ", 发现问题数: " << verify->issues.size() << ")\n";
    os << "   各轴峰值速度: [";
    for (int j = 0; j < 6; ++j) {
      if (j) os << ", ";
      os << Fmt("%.1f", verify->peak_joint_speeds_deg_s[static_cast<size_t>(j)]);
    }
    os << "] deg/s\n";
    if (!verify->issues.empty()) {
      os << "   ⚠️ 校验问题详情:\n";
      const size_t lim = std::min<size_t>(10, verify->issues.size());
      for (size_t i = 0; i < lim; ++i) {
        os << "      - [" << verify->issues[i].severity << "] " << verify->issues[i].type << ": "
           << verify->issues[i].detail << "\n";
      }
    } else {
      os << "   🎉 校验完美通过 (0 奇异, 0 超速, 0 不可达, 关节连续平滑)!\n";
    }
  }

  if (!output_path.empty()) {
    os << "\n💾 8. 优化后轨迹已成功保存至: " << output_path << "\n";
  }
  os << std::string(136, '=') << "\n";
}

void PrintVerifyReport(std::ostream& os, const VerifyReport& report, double elapsed_ms) {
  os << std::string(136, '=') << "\n";
  os << "🔍 全路径运动学链校验结果 (Kinematic Chain Verification):\n";
  os << "   校验状态: " << report.summary.status << " (总插值步数: " << report.summary.total_steps
     << ", 发现问题数: " << report.summary.total_issues << ", 耗时 " << Fmt("%.2f", elapsed_ms)
     << " ms)\n";
  if (!report.path_reports.empty()) {
    const auto& pr = report.path_reports[0];
    os << "   各轴峰值速度: [";
    for (int j = 0; j < 6; ++j) {
      if (j) os << ", ";
      os << Fmt("%.1f", pr.peak_joint_speeds_deg_s[static_cast<size_t>(j)]);
    }
    os << "] deg/s\n";
    os << "   推荐安全线速度: " << pr.recommended_safe_speed_mm_s << " mm/s\n";
    if (!pr.issues.empty()) {
      os << "   ⚠️ 校验问题详情:\n";
      const size_t lim = std::min<size_t>(10, pr.issues.size());
      for (size_t i = 0; i < lim; ++i) {
        os << "      - [" << pr.issues[i].severity << "] " << pr.issues[i].type << ": "
           << pr.issues[i].detail << "\n";
      }
    } else {
      os << "   🎉 校验完美通过 (0 奇异, 0 超速, 0 不可达, 关节连续平滑)!\n";
    }
  }
  os << std::string(136, '=') << "\n";
}

}  // namespace motion

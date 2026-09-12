#include "cli_report.hpp"
#include "motion/io.hpp"
#include "motion/kinematics.hpp"
#include "motion/optimizer.hpp"
#include "motion/robot_model.hpp"
#include "motion/verifier.hpp"

#include <CLI/CLI.hpp>

#include <chrono>
#include <iostream>
#include <optional>
#include <sstream>
#include <vector>

namespace {

std::vector<double> SplitCsv(const std::string& s) {
  std::vector<double> out;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    if (!tok.empty()) out.push_back(std::stod(tok));
  }
  return out;
}

motion::AxisGrid ParseGrid(const std::string& s, const motion::AxisGrid& def) {
  const auto v = SplitCsv(s);
  if (v.size() != 3) return def;
  return {v[0], v[1], v[2]};
}

int EmitError(int code, const std::string& action, const std::string& msg) {
  // msg 常常是 yaml-cpp / 优化器抛出的异常文本，可能带引号或换行；不转义会让
  // Python 侧 json.loads 失败，从而把一个可读的错误变成“stdout 不是合法 JSON”。
  std::cout << "{\"success\":false,\"action\":\"" << action << "\",\"message\":\""
            << motion::JsonEscapeString(msg) << "\"}\n";
  return code;
}

// poi yaml 的 source_file 记的是同目录内的文件名，不是调用时传入的绝对路径。
std::string BaseName(const std::string& p) {
  const auto n = p.find_last_of("/\\");
  return n == std::string::npos ? p : p.substr(n + 1);
}

}  // namespace

int main(int argc, char** argv) {
  std::cerr << std::unitbuf;
  CLI::App app{"AiSprayer motion CLI (verify / optimize / fk / ik)"};
  app.require_subcommand(1);

  std::string input, output, urdf, tool = "gripper_tip_link";
  std::string config_path;
  std::string seed_deg = "0,0,-90,-90,-90,0";
  double speed = 120.0, step = 1.5;
  int path_id = -1;

  app.add_option("--config", config_path,
                 "aisprayer_config.yaml（读取 spraying.poi_* / grid_tol_* / robot_urdf）");

  auto* verify = app.add_subcommand("verify", "Dense MoveL kinematic verification (input: *.path.yaml)");
  verify->add_option("--input", input, "待验证路径 yaml")->required();
  verify->add_option("--output", output, "写入 verification 后的 yaml（可与 --input 相同）");
  verify->add_option("--urdf", urdf, "robot URDF（可被 --config 填充）");
  verify->add_option("--tool-tcp", tool, "TCP link name");
  verify->add_option("--speed", speed, "Cartesian speed mm/s");
  verify->add_option("--step", step, "interpolation step mm");
  verify->add_option("--path-id", path_id, "optional path_id filter");
  verify->add_option("--seed", seed_deg, "init joints deg (default Home)");

  std::string anchor_source = "config";
  std::string ref_rpy = "90,0,90";
  std::string anchor_tol = "10,10,30";
  std::string grid_x = "-5,5,2", grid_y = "-5,5,2", grid_z = "-30,30,5";
  std::string home_joints = "0,0,-90,-90,-90,0";
  int beam = 32, max_per_branch = 16;
  bool no_dense = false;
  // 容差阶梯择优（默认开）：请求包络 + 按比例收紧的若干档各跑一次，取最优。
  bool no_tol_ladder = false;
  std::string ladder_scales = "0.5,0.333333,0.25";
  double ladder_stop_ratio = 0.3;
  double ladder_max_pointing = 0.0;
  // ⑤ 边内关节速度约束（默认开）：CLI 显式传参 > 配置文件 > 默认值。
  bool no_opt_vel_limit = false;
  double opt_vel_soft_ratio = 0.9, opt_vel_cost_weight = 40.0, opt_vel_hard_ratio = 1.15;
  // ⑤-A 腕部奇异自适应降速（默认取配置，CLI 显式传参优先）。
  bool singularity_on = false;
  double sing_ref_deg = 25.0, sing_min_scale = 0.2;
  std::string state_type = "auto_poi";

  auto* optimize = app.add_subcommand("optimize", "Viterbi optimize scan.auto.path.yaml → poi");
  optimize->add_option("--input", input, "待优化路径 yaml（应为 scan.auto.path.yaml）")->required();
  optimize->add_option("--output", output, "output yaml");
  optimize->add_option("--urdf", urdf, "robot URDF（可被 --config 填充）");
  optimize->add_option("--tool-tcp", tool);
  optimize->add_option("--anchor-source", anchor_source, "config | home | raw");
  optimize->add_option("--ref-rpy", ref_rpy, "锚点中心 Rx,Ry,Rz deg（config/live）");
  optimize->add_option("--anchor-tol", anchor_tol, "锚点包络 ±Rx,±Ry,±Rz deg");
  optimize->add_option("--grid-x", grid_x, "工具系 X 搜索网格 min,max,step");
  optimize->add_option("--grid-y", grid_y, "工具系 Y 搜索网格 min,max,step");
  optimize->add_option("--grid-z", grid_z, "工具系 Z 搜索网格 min,max,step");
  optimize->add_option("--home-joints", home_joints);
  optimize->add_option("--beam-width", beam);
  optimize->add_option("--max-candidates-per-branch", max_per_branch);
  optimize->add_option("--speed", speed);
  optimize->add_option("--step", step);
  optimize->add_option("--state-type", state_type, "写出的 type/state_type：auto_poi | poi");
  optimize->add_flag("--no-dense-verify", no_dense);
  optimize->add_flag("--no-tol-ladder", no_tol_ladder,
                     "关闭容差阶梯择优（只跑请求包络一档，即旧行为）");
  optimize->add_option("--tol-ladder-scales", ladder_scales,
                       "阶梯收紧比例（逗号分隔，均需在 (0,1)）");
  optimize->add_option("--tol-ladder-stop-ratio", ladder_stop_ratio,
                       "早停阈值：某档 PASS 且 峰值/限速 ≤ 此值时不再继续收紧（0=跑完全部）");
  optimize->add_option("--tol-ladder-max-pointing-deg", ladder_max_pointing,
                       "指向偏量护栏：某档最大指向偏量 > 请求档 + 此值就弃用该档（0=不限制）");
  optimize->add_flag("--no-opt-vel-limit", no_opt_vel_limit,
                     "关闭边内关节速度约束（退化为纯 Δq² 代价，即旧行为）");
  optimize->add_option("--opt-vel-soft-ratio", opt_vel_soft_ratio,
                       "软罚阈值：角速度 > 此比例×限速 的边加二次惩罚（留余量）");
  optimize->add_option("--opt-vel-cost-weight", opt_vel_cost_weight,
                       "超软阈惩罚权重：cost += Σ(ratio-soft)²×此值");
  optimize->add_option("--opt-vel-hard-ratio", opt_vel_hard_ratio,
                       "硬禁阈值：角速度 > 此比例×限速 的边直接判不可行（0=只加罚不硬禁）");
  optimize->add_flag("--singularity-scaling", singularity_on,
                     "开启腕部奇异自适应降速（|J5|→0 段按可操作度压低线速度）");
  optimize->add_option("--singularity-ref-deg", sing_ref_deg,
                       "|J5| ≥ 此角度不减速（deg，默认 25）");
  optimize->add_option("--singularity-min-scale", sing_min_scale,
                       "减速下限比例（0~1，最多降到该倍名义线速度，默认 0.2）");

  std::string joints = "0,0,-90,-90,-90,0";
  std::string pose;
  auto* fk = app.add_subcommand("fk", "Forward kinematics (controller mm/deg)");
  fk->add_option("--joints", joints, "6 joints in degrees")->required();

  auto* ik = app.add_subcommand("ik", "Inverse kinematics (controller mm/deg)");
  ik->add_option("--pose", pose, "x,y,z,rx,ry,rz")->required();
  ik->add_option("--seed", joints, "seed joints deg");
  ik->add_option("--urdf", urdf);
  ik->add_option("--tool-tcp", tool);

  CLI11_PARSE(app, argc, argv);

  // 生效值：先取 CLI 默认 → 有 --config 时用配置覆盖未显式给出的项 → CLI 显式值最高优先。
  motion::SprayingConfig eff;
  eff.ref_rpy_deg = {90.0, 0.0, 90.0};
  eff.tol_deg = {10.0, 10.0, 30.0};

  auto resolve_config = [&]() -> int {
    CLI::App* sub = verify->parsed() ? verify : (optimize->parsed() ? optimize : nullptr);
    auto given = [&](const char* name) {
      return sub != nullptr && sub->get_option_no_throw(name) != nullptr &&
             sub->get_option(name)->count() > 0;
    };
    if (!config_path.empty()) {
      std::string err;
      if (!motion::LoadSprayingConfig(config_path, eff, &err)) return EmitError(3, "cli", err);
      if (urdf.empty()) urdf = eff.urdf_path;
      if (!given("--tool-tcp") && !eff.tool_name.empty()) tool = eff.tool_name;
      if (!given("--speed")) speed = eff.speed_mm_s;
      if (!given("--step")) step = eff.step_mm;
      if (!given("--anchor-source")) anchor_source = eff.anchor_source;
    }
    if (given("--ref-rpy")) {
      const auto v = SplitCsv(ref_rpy);
      if (v.size() != 3) return EmitError(2, "cli", "--ref-rpy needs 3 values");
      eff.ref_rpy_deg = {v[0], v[1], v[2]};
    }
    if (given("--anchor-tol")) {
      const auto v = SplitCsv(anchor_tol);
      if (v.size() != 3) return EmitError(2, "cli", "--anchor-tol needs 3 values");
      eff.tol_deg = {v[0], v[1], v[2]};
    }
    if (given("--grid-x")) eff.grid_x = ParseGrid(grid_x, eff.grid_x);
    if (given("--grid-y")) eff.grid_y = ParseGrid(grid_y, eff.grid_y);
    if (given("--grid-z")) eff.grid_z = ParseGrid(grid_z, eff.grid_z);
    if (optimize->parsed()) {
      std::cerr << "[motion_cli] anchor=" << anchor_source << " ref_rpy=["
                << eff.ref_rpy_deg.transpose() << "] tol=[" << eff.tol_deg.transpose()
                << "] grid_z=[" << eff.grid_z.min_deg << "," << eff.grid_z.max_deg << ","
                << eff.grid_z.step_deg << "] urdf=" << urdf << "\n";
    }
    return 0;
  };

  try {
    if (int rc = resolve_config()) return rc;
    if ((verify->parsed() || optimize->parsed()) && urdf.empty()) {
      return EmitError(2, "cli", "missing --urdf (or --config with hardware.robot.robot_urdf)");
    }
    if (verify->parsed()) {
      motion::RobotModel model;
      std::string err;
      if (!motion::LoadRobotModelFromUrdf(urdf, tool, model, &err)) {
        return EmitError(3, "verify", err);
      }
      motion::PathDocument doc;
      if (!motion::LoadPathYaml(input, doc, &err)) return EmitError(3, "verify", err);
      if (path_id >= 0) {
        std::vector<motion::PathItem> keep;
        for (auto& p : doc.paths)
          if (p.path_id == path_id) keep.push_back(std::move(p));
        doc.paths = std::move(keep);
      }
      motion::Cr5Kinematics kin(model.limits);
      motion::VerifyOptions opt;
      opt.step_mm = step;
      opt.speed_mm_s = speed;
      // verify 子命令与配置同口径的奇异降速（无专设 CLI 开关，直接取 --config 生效值）。
      opt.singularity_scaling = eff.singularity_scaling;
      opt.singularity_ref_deg = eff.singularity_ref_deg;
      opt.singularity_min_scale = eff.singularity_min_scale;
      motion::ChainVerifier v(kin, model.tool, opt);
      std::optional<motion::JointVec> seed;
      if (verify->count("--seed") > 0) {
        const auto seed_vals = SplitCsv(seed_deg);
        if (seed_vals.size() == 6) {
          motion::JointVec q;
          for (int i = 0; i < 6; ++i) q[i] = motion::Rad(seed_vals[i]);
          seed = q;
        }
      }
      const auto t0 = std::chrono::steady_clock::now();
      const motion::VerifyReport report = v.VerifyAll(doc.paths, seed);
      const auto t1 = std::chrono::steady_clock::now();
      const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
      motion::PrintVerifyReport(std::cerr, report, ms);
      if (!output.empty()) {
        doc.execution_speed_mm_s = speed;
        if (!motion::SavePathYaml(output, doc, &report, nullptr, &err)) {
          return EmitError(3, "verify", err);
        }
      }
      std::cout << motion::JsonReportVerify(report, ms, true, "") << "\n";
      return 0;
    }

    if (optimize->parsed()) {
      motion::RobotModel model;
      std::string err;
      if (!motion::LoadRobotModelFromUrdf(urdf, tool, model, &err)) {
        return EmitError(3, "optimize", err);
      }
      motion::PathDocument doc;
      if (!motion::LoadPathYaml(input, doc, &err)) return EmitError(3, "optimize", err);
      motion::Cr5Kinematics kin(model.limits);
      motion::OptimizeOptions oopt;
      oopt.grid_x = eff.grid_x;
      oopt.grid_y = eff.grid_y;
      oopt.grid_z = eff.grid_z;
      oopt.beam_width = beam;
      oopt.max_candidates_per_branch = max_per_branch;
      oopt.dense_verify = !no_dense;
      // 容差阶梯择优：CLI 显式传参 > 配置文件(eff) > 默认值。用 count()>0 判断是否显式给出，
      // 未给出时回落到 eff（无 --config 时 eff 即结构体默认值，与旧行为一致）。
      auto given_opt = [&](const char* name) {
        const auto* o = optimize->get_option_no_throw(name);
        return o != nullptr && o->count() > 0;
      };
      oopt.tol_ladder = given_opt("--no-tol-ladder") ? !no_tol_ladder : eff.tol_ladder;
      if (given_opt("--tol-ladder-scales")) {
        if (const auto sc = SplitCsv(ladder_scales); !sc.empty()) oopt.tol_ladder_scales = sc;
      } else if (!eff.tol_ladder_scales.empty()) {
        oopt.tol_ladder_scales = eff.tol_ladder_scales;
      }
      oopt.tol_ladder_stop_peak_ratio = given_opt("--tol-ladder-stop-ratio")
                                            ? ladder_stop_ratio
                                            : eff.tol_ladder_stop_peak_ratio;
      oopt.tol_ladder_max_pointing_deg = given_opt("--tol-ladder-max-pointing-deg")
                                             ? ladder_max_pointing
                                             : eff.tol_ladder_max_pointing_deg;
      // ⑤ 边内速度约束：执行线速度与校验器同源（speed），保证 DP 选边与终校 °/s 口径一致。
      oopt.exec_speed_mm_s = speed;
      oopt.enforce_vel_limit =
          given_opt("--no-opt-vel-limit") ? !no_opt_vel_limit : eff.opt_enforce_vel_limit;
      oopt.vel_soft_ratio =
          given_opt("--opt-vel-soft-ratio") ? opt_vel_soft_ratio : eff.opt_vel_soft_ratio;
      oopt.vel_cost_weight =
          given_opt("--opt-vel-cost-weight") ? opt_vel_cost_weight : eff.opt_vel_cost_weight;
      oopt.vel_hard_ratio =
          given_opt("--opt-vel-hard-ratio") ? opt_vel_hard_ratio : eff.opt_vel_hard_ratio;
      // ⑤-A 奇异降速：DP 选边(⑤)与终校共用同一缩放模型，两边必须同值。
      oopt.singularity_scaling =
          given_opt("--singularity-scaling") ? singularity_on : eff.singularity_scaling;
      oopt.singularity_ref_deg =
          given_opt("--singularity-ref-deg") ? sing_ref_deg : eff.singularity_ref_deg;
      oopt.singularity_min_scale =
          given_opt("--singularity-min-scale") ? sing_min_scale : eff.singularity_min_scale;
      if (auto e = oopt.Validate(); !e.empty()) return EmitError(2, "optimize", e);

      motion::VerifyOptions vopt;
      vopt.step_mm = step;
      vopt.speed_mm_s = speed;
      // 终校器与 DP 采用完全相同的奇异缩放，否则“DP 选出的可行解”与“终校判超速”口径不一致。
      vopt.singularity_scaling = oopt.singularity_scaling;
      vopt.singularity_ref_deg = oopt.singularity_ref_deg;
      vopt.singularity_min_scale = oopt.singularity_min_scale;
      motion::ChainVerifier verifier(kin, model.tool, vopt);
      motion::ViterbiOptimizer optimizer(kin, model.tool, oopt, &verifier);

      motion::AnchorSpec spec;
      spec.source = anchor_source;
      spec.ref_rpy_deg = eff.ref_rpy_deg;
      spec.tol_deg = eff.tol_deg;
      const auto home = SplitCsv(home_joints);
      if (home.size() == 6) {
        for (int i = 0; i < 6; ++i) spec.home_joints_rad[i] = motion::Rad(home[i]);
      }

      motion::PathDocument out_doc = doc;
      if (doc.paths.empty()) {
        // 旧代码在这种情况下会在后面的 doc.paths.back() 上未定义行为（空 vector）。
        return EmitError(2, "optimize", "input has no paths: " + input);
      }
      std::optional<motion::JointVec> last_q;
      motion::OptimizeResult last;
      double total_ms = 0.0;
      bool any_modified = false;
      for (size_t i = 0; i < doc.paths.size(); ++i) {
        const auto anchor = motion::ResolveAnchor(spec, kin, doc.paths[i]);
        motion::PrintOptimizePreamble(std::cerr, input, doc.paths[i], kin, spec, anchor, oopt,
                                      speed, step);
        last = optimizer.Optimize(doc.paths[i], anchor, last_q);
        out_doc.paths[i] = last.path;
        total_ms += last.elapsed_ms;
        any_modified = any_modified || last.modified;
        if (!last.joints_rad.empty()) last_q = last.joints_rad.back();
      }
      last.elapsed_ms = total_ms;
      last.modified = any_modified;
      // web 模板状态机：auto → auto_poi，raw/manual → poi。
      if (state_type != "poi" && state_type != "auto_poi") {
        return EmitError(2, "optimize", "--state-type must be poi or auto_poi");
      }
      out_doc.type = state_type;
      out_doc.state_type = state_type;
      out_doc.source_file = BaseName(input);
      out_doc.execution_speed_mm_s = speed;
      // 阶梯择优可能采纳了比请求更紧的包络；报表与落盘都以“采纳档”为准。
      const Eigen::Vector3d adopted_tol =
          last.adopted_tol_deg.maxCoeff() > 0.0 ? last.adopted_tol_deg : spec.tol_deg;
      const bool ladder_applied = (adopted_tol - spec.tol_deg).cwiseAbs().maxCoeff() > 1e-9;
      const std::optional<motion::JointVec> opt_seed =
          last.joints_rad.empty() ? std::nullopt : std::make_optional(last.joints_rad[0]);
      auto all = verifier.VerifyAll(out_doc.paths, opt_seed);
      if (!output.empty()) {
        motion::PoiConfig poi;
        poi.anchor_source = anchor_source;
        poi.mode = (anchor_source == "raw") ? "per_waypoint_nominal_envelope"
                                            : "absolute_anchor_tolerance";
        poi.ref_rpy_deg = spec.ref_rpy_deg;
        poi.tolerance_rpy_deg = spec.tol_deg;
        poi.adopted_tolerance_rpy_deg = adopted_tol;
        poi.tolerance_ladder_applied = ladder_applied;
        poi.has_ref_rpy = (anchor_source != "raw");
        if (!motion::SavePathYaml(output, out_doc, &all, &poi, &err)) {
          return EmitError(3, "optimize", err);
        }
      }
      const motion::PathVerifyReport* first =
          all.path_reports.empty() ? nullptr : &all.path_reports[0];
      const motion::Anchor last_anchor = motion::ResolveAnchor(spec, kin, doc.paths.back());
      motion::PrintOptimizeReport(std::cerr, kin, doc.paths.back(), last, last_anchor, adopted_tol,
                                  spec.home_joints_rad, first, output);
      std::cout << motion::JsonReportOptimize(last, &all, true, "") << "\n";
      return 0;
    }

    if (fk->parsed()) {
      const auto qdeg = SplitCsv(joints);
      if (qdeg.size() != 6) return EmitError(2, "fk", "need 6 joints");
      motion::JointVec q;
      for (int i = 0; i < 6; ++i) q[i] = motion::Rad(qdeg[i]);
      motion::Cr5Kinematics kin;
      Eigen::Vector3d xyz, rpy;
      kin.FkController(q, xyz, rpy);
      std::cout << "{\"success\":true,\"action\":\"fk\",\"xyz_mm\":[" << xyz[0] << "," << xyz[1]
                << "," << xyz[2] << "],\"rpy_deg\":[" << rpy[0] << "," << rpy[1] << "," << rpy[2]
                << "]}\n";
      return 0;
    }

    if (ik->parsed()) {
      const auto p = SplitCsv(pose);
      if (p.size() != 6) return EmitError(2, "ik", "need x,y,z,rx,ry,rz");
      motion::RobotLimits limits;
      if (!urdf.empty()) {
        motion::RobotModel model;
        std::string err;
        if (!motion::LoadRobotModelFromUrdf(urdf, tool, model, &err)) return EmitError(3, "ik", err);
        limits = model.limits;
      }
      motion::Cr5Kinematics kin(limits);
      const auto seed_deg = SplitCsv(joints);
      motion::JointVec seed = motion::JointVec::Zero();
      if (seed_deg.size() == 6)
        for (int i = 0; i < 6; ++i) seed[i] = motion::Rad(seed_deg[i]);
      const motion::Transform T =
          motion::PoseFromCtrlMmDeg({p[0], p[1], p[2]}, {p[3], p[4], p[5]});
      auto best = kin.BestIk(motion::CtrlToUrdf(T), seed);
      if (!best) return EmitError(4, "ik", "no IK");
      std::cout << "{\"success\":true,\"action\":\"ik\",\"q_rad\":[";
      for (int i = 0; i < 6; ++i) {
        if (i) std::cout << ",";
        std::cout << (*best)[i];
      }
      std::cout << "]}\n";
      return 0;
    }
  } catch (const std::exception& e) {
    const std::string act = optimize->parsed() ? "optimize" : (verify->parsed() ? "verify" : "cli");
    return EmitError(4, act, e.what());
  }
  return 2;
}

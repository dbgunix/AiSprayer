#include "motion/io.hpp"

#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <yaml-cpp/yaml.h>

namespace motion {
namespace {

Waypoint ParseWaypoint(const YAML::Node& n) {
  Waypoint wp;
  if (n["index"]) wp.index = n["index"].as<int>();
  if (n["pixel"] && n["pixel"].IsSequence() && n["pixel"].size() >= 2) {
    wp.pixel = Eigen::Vector2i(n["pixel"][0].as<int>(), n["pixel"][1].as<int>());
  }
  if (n["surface_point_base_mm"] && n["surface_point_base_mm"].IsSequence()) {
    wp.surface_point_m = Eigen::Vector3d(n["surface_point_base_mm"][0].as<double>(),
                                         n["surface_point_base_mm"][1].as<double>(),
                                         n["surface_point_base_mm"][2].as<double>()) /
                         kMmPerM;
  }
  if (n["surface_normal_base"] && n["surface_normal_base"].IsSequence()) {
    wp.surface_normal = Eigen::Vector3d(n["surface_normal_base"][0].as<double>(),
                                        n["surface_normal_base"][1].as<double>(),
                                        n["surface_normal_base"][2].as<double>());
  }
  if (n["standoff_distance_mm"]) wp.standoff_m = n["standoff_distance_mm"].as<double>() / kMmPerM;
  if (n["normal_2d_proj"] && n["normal_2d_proj"].IsSequence() && n["normal_2d_proj"].size() >= 2) {
    wp.normal_2d_proj =
        Eigen::Vector2d(n["normal_2d_proj"][0].as<double>(), n["normal_2d_proj"][1].as<double>());
  }
  YAML::Node pose = n["tcp_pose_base"] ? n["tcp_pose_base"] : n;
  const double x = pose["x"] ? pose["x"].as<double>() : 0.0;
  const double y = pose["y"] ? pose["y"].as<double>() : 0.0;
  const double z = pose["z"] ? pose["z"].as<double>() : 0.0;
  const double rx = pose["rx"] ? pose["rx"].as<double>() : 0.0;
  const double ry = pose["ry"] ? pose["ry"].as<double>() : 0.0;
  const double rz = pose["rz"] ? pose["rz"].as<double>() : 0.0;
  wp.tcp_pose = PoseFromCtrlMmDeg(Eigen::Vector3d(x, y, z), Eigen::Vector3d(rx, ry, rz));
  if (n["spraying"]) {
    const std::string s = n["spraying"].as<std::string>();
    wp.spraying = (s != "off" && s != "OFF" && s != "0" && s != "false");
  }
  if (n["is_jump"]) {
    wp.is_jump = n["is_jump"].as<bool>();
    wp.has_is_jump = true;
  }
  if (!wp.spraying) wp.is_jump = true;
  return wp;
}

PathItem ParsePathItem(const YAML::Node& n, int fallback_id) {
  PathItem item;
  item.path_id = n["path_id"] ? n["path_id"].as<int>() : fallback_id;
  item.name = n["name"] ? n["name"].as<std::string>() : ("Path " + std::to_string(item.path_id));
  if (n["points"]) {
    for (const auto& p : n["points"]) item.points.push_back(ParseWaypoint(p));
  }
  if (n["dense_surface_points_base_mm"] && n["dense_surface_points_base_mm"].IsSequence()) {
    for (const auto& d : n["dense_surface_points_base_mm"]) {
      if (!d.IsSequence() || d.size() < 3) continue;
      item.dense_surface_points_mm.emplace_back(d[0].as<double>(), d[1].as<double>(),
                                                d[2].as<double>());
    }
  }
  return item;
}

// yaml-cpp 对 double 走 %g，150.0 会退化成整数 150（PyYAML 侧就变 int），且默认
// 17 位精度会把 733.92 写成 733.91999999999996。统一用定点字符串标量输出：
// 内容不含特殊字符时 yaml-cpp 原样不加引号，既保住小数点也保住类型。
std::string Fixed(double v, int decimals) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(decimals) << v;
  std::string s = os.str();
  // 归一 -0.00 → 0.00（全部字符都是 '-'/'0'/'.' 即为零值）
  if (s[0] == '-' && s.find_first_not_of("-0.") == std::string::npos) s.erase(0, 1);
  return s;
}

void EmitFixedSeq(YAML::Emitter& e, const double* v, int n, int decimals) {
  e << YAML::Flow << YAML::BeginSeq;
  for (int i = 0; i < n; ++i) e << Fixed(v[i], decimals);
  e << YAML::EndSeq;
}

void EmitPose(YAML::Emitter& e, const Transform& T) {
  Eigen::Vector3d xyz, rpy;
  PoseToCtrlMmDeg(T, xyz, rpy);
  e << YAML::BeginMap;
  const char* keys[6] = {"x", "y", "z", "rx", "ry", "rz"};
  const double vals[6] = {xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]};
  for (int i = 0; i < 6; ++i) e << YAML::Key << keys[i] << YAML::Value << Fixed(vals[i], 2);
  e << YAML::EndMap;
}

// 逐字段对齐 Python 版 _clean_waypoints_data：优化只改 tcp_pose_base，
// pixel / surface_* / standoff / normal_2d_proj 必须原样带回。
void EmitWaypoint(YAML::Emitter& e, const Waypoint& wp) {
  e << YAML::BeginMap;
  e << YAML::Key << "index" << YAML::Value << wp.index;
  e << YAML::Key << "pixel" << YAML::Value << YAML::Flow << YAML::BeginSeq << wp.pixel[0]
    << wp.pixel[1] << YAML::EndSeq;
  const Eigen::Vector3d sp_mm = wp.surface_point_m * kMmPerM;
  e << YAML::Key << "surface_point_base_mm" << YAML::Value;
  EmitFixedSeq(e, sp_mm.data(), 3, 2);
  e << YAML::Key << "surface_normal_base" << YAML::Value;
  EmitFixedSeq(e, wp.surface_normal.data(), 3, 4);
  e << YAML::Key << "standoff_distance_mm" << YAML::Value << Fixed(wp.standoff_m * kMmPerM, 1);
  e << YAML::Key << "tcp_pose_base" << YAML::Value;
  EmitPose(e, wp.tcp_pose);
  if (wp.normal_2d_proj) {
    e << YAML::Key << "normal_2d_proj" << YAML::Value;
    EmitFixedSeq(e, wp.normal_2d_proj->data(), 2, 1);
  }
  // 必须加引号：YAML 1.1 会把裸 on/off 解析成布尔，Python 侧比较的是字符串。
  e << YAML::Key << "spraying" << YAML::Value << YAML::SingleQuoted
    << std::string(wp.spraying ? "on" : "off");
  if (wp.has_is_jump) e << YAML::Key << "is_jump" << YAML::Value << wp.is_jump;
  e << YAML::EndMap;
}

void EmitIssues(YAML::Emitter& e, const std::vector<Issue>& issues) {
  e << YAML::BeginSeq;
  for (const auto& iss : issues) {
    e << YAML::BeginMap;
    e << YAML::Key << "severity" << YAML::Value << iss.severity;
    e << YAML::Key << "type" << YAML::Value << iss.type;
    e << YAML::Key << "detail" << YAML::Value << iss.detail;
    e << YAML::Key << "step_index" << YAML::Value << iss.step_index;
    e << YAML::Key << "segment_index" << YAML::Value << iss.segment_index;
    // Python 侧键名是 location_xyz（不带 _mm 后缀）。
    e << YAML::Key << "location_xyz" << YAML::Value;
    EmitFixedSeq(e, iss.location_xyz_mm.data(), 3, 2);
    e << YAML::EndMap;
  }
  e << YAML::EndSeq;
}

void EmitVerification(YAML::Emitter& e, const VerifyReport& v) {
  e << YAML::Key << "verification" << YAML::Value << YAML::BeginMap;
  e << YAML::Key << "status" << YAML::Value << v.summary.status;
  e << YAML::Key << "summary" << YAML::Value << YAML::BeginMap;
  e << YAML::Key << "status" << YAML::Value << v.summary.status;
  e << YAML::Key << "total_paths" << YAML::Value << v.summary.total_paths;
  e << YAML::Key << "total_waypoints" << YAML::Value << v.summary.total_waypoints;
  e << YAML::Key << "total_steps" << YAML::Value << v.summary.total_steps;
  e << YAML::Key << "total_issues" << YAML::Value << v.summary.total_issues;
  e << YAML::Key << "singularity_count" << YAML::Value << v.summary.singularity_count;
  e << YAML::Key << "overspeed_count" << YAML::Value << v.summary.overspeed_count;
  e << YAML::Key << "unreachable_count" << YAML::Value << v.summary.unreachable_count;
  e << YAML::EndMap;
  e << YAML::Key << "nominal_speed_mm_s" << YAML::Value << Fixed(v.nominal_speed_mm_s, 1);
  e << YAML::Key << "slerp_step_mm" << YAML::Value << Fixed(v.slerp_step_mm, 2);
  e << YAML::Key << "max_joint_velocities_deg_s" << YAML::Value;
  EmitFixedSeq(e, v.max_joint_velocities_deg_s.data(), 6, 2);
  e << YAML::Key << "urdf_tcp" << YAML::Value << YAML::BeginMap;
  e << YAML::Key << "has_tool" << YAML::Value << v.urdf_tcp.has_tool;
  e << YAML::Key << "tool_name" << YAML::Value << v.urdf_tcp.tool_name;
  e << YAML::Key << "xyz_mm" << YAML::Value;
  EmitFixedSeq(e, v.urdf_tcp.xyz_mm.data(), 3, 2);
  e << YAML::Key << "rpy_deg" << YAML::Value;
  EmitFixedSeq(e, v.urdf_tcp.rpy_deg.data(), 3, 2);
  e << YAML::Key << "urdf_source" << YAML::Value << v.urdf_tcp.urdf_source;
  e << YAML::EndMap;
  e << YAML::Key << "path_reports" << YAML::Value << YAML::BeginSeq;
  for (const auto& pr : v.path_reports) {
    e << YAML::BeginMap;
    e << YAML::Key << "path_id" << YAML::Value << pr.path_id;
    e << YAML::Key << "name" << YAML::Value << pr.name;
    e << YAML::Key << "status" << YAML::Value << pr.status;
    e << YAML::Key << "total_interpolated" << YAML::Value << pr.total_interpolated;
    e << YAML::Key << "speed_mm_s" << YAML::Value << Fixed(pr.speed_mm_s, 1);
    e << YAML::Key << "recommended_safe_speed_mm_s" << YAML::Value
      << Fixed(pr.recommended_safe_speed_mm_s, 1);
    e << YAML::Key << "peak_joint_speeds_deg_s" << YAML::Value;
    EmitFixedSeq(e, pr.peak_joint_speeds_deg_s.data(), 6, 1);
    e << YAML::Key << "issues" << YAML::Value;
    EmitIssues(e, pr.issues);
    e << YAML::Key << "trajectory_q" << YAML::Value << YAML::BeginSeq;
    for (const auto& q : pr.trajectory_q) EmitFixedSeq(e, q.data(), 6, 4);
    e << YAML::EndSeq;
    e << YAML::Key << "trajectory_tcp" << YAML::Value << YAML::BeginSeq;
    for (const auto& t : pr.trajectory_tcp) EmitFixedSeq(e, t.data(), 6, 2);
    e << YAML::EndSeq;
    // ⑤-A 逐航点建议线速度剖面（仅开启奇异缩放时非空），供执行侧逐段设 TCPSpeed。
    if (!pr.waypoint_speed_mm_s.empty()) {
      e << YAML::Key << "waypoint_speed_mm_s" << YAML::Value << YAML::Flow << YAML::BeginSeq;
      for (double v : pr.waypoint_speed_mm_s) e << Fixed(v, 1);
      e << YAML::EndSeq;
    }
    e << YAML::EndMap;
  }
  e << YAML::EndSeq;
  e << YAML::EndMap;
}

}  // namespace

bool LoadPathYaml(const std::string& path, PathDocument& out, std::string* err) {
  try {
    YAML::Node root = YAML::LoadFile(path);
    out.template_name = root["template"] ? root["template"].as<std::string>() : "";
    out.type = root["type"] ? root["type"].as<std::string>() : "";
    out.state_type = root["state_type"] ? root["state_type"].as<std::string>() : out.type;
    out.source_file = root["source_file"] ? root["source_file"].as<std::string>() : "";
    out.coordinate_frame =
        root["coordinate_frame"] ? root["coordinate_frame"].as<std::string>() : "base_link";
    if (root["standoff_distance_mm"])
      out.standoff_distance_mm = root["standoff_distance_mm"].as<double>();
    if (root["execution_speed_mm_s"])
      out.execution_speed_mm_s = root["execution_speed_mm_s"].as<double>();

    YAML::Node paths = root["paths"];
    if (!paths) {
      out.paths.push_back(ParsePathItem(root, 1));
      return true;
    }
    int next_id = 1;
    for (const auto& pn : paths) out.paths.push_back(ParsePathItem(pn, next_id++));
    return true;
  } catch (const std::exception& e) {
    if (err) *err = e.what();
    return false;
  }
}

bool SavePathYaml(const std::string& path, const PathDocument& doc, const VerifyReport* verify,
                  const PoiConfig* poi, std::string* err) {
  try {
    YAML::Emitter e;
    e << YAML::BeginMap;
    e << YAML::Key << "standoff_distance_mm" << YAML::Value << Fixed(doc.standoff_distance_mm, 1);
    if (!doc.template_name.empty())
      e << YAML::Key << "template" << YAML::Value << doc.template_name;
    if (!doc.type.empty()) e << YAML::Key << "type" << YAML::Value << doc.type;
    if (!doc.state_type.empty()) e << YAML::Key << "state_type" << YAML::Value << doc.state_type;
    if (!doc.source_file.empty()) e << YAML::Key << "source_file" << YAML::Value << doc.source_file;
    e << YAML::Key << "updated_at" << YAML::Value
      << static_cast<long long>(std::time(nullptr));
    e << YAML::Key << "coordinate_frame" << YAML::Value << doc.coordinate_frame;
    e << YAML::Key << "execution_speed_mm_s" << YAML::Value << Fixed(doc.execution_speed_mm_s, 1);
    if (poi) {
      e << YAML::Key << "poi_config" << YAML::Value << YAML::BeginMap;
      e << YAML::Key << "mode" << YAML::Value << poi->mode;
      e << YAML::Key << "anchor_source" << YAML::Value << poi->anchor_source;
      e << YAML::Key << "ref_rpy_deg" << YAML::Value;
      if (poi->has_ref_rpy) {
        EmitFixedSeq(e, poi->ref_rpy_deg.data(), 3, 1);
      } else {
        e << YAML::Null;
      }
      e << YAML::Key << "tolerance_rpy_deg" << YAML::Value;
      EmitFixedSeq(e, poi->tolerance_rpy_deg.data(), 3, 1);
      // 阶梯择优可能采纳了比请求更紧的包络，这里把“实际用的那一档”也写回去。
      e << YAML::Key << "adopted_tolerance_rpy_deg" << YAML::Value;
      EmitFixedSeq(e, poi->adopted_tolerance_rpy_deg.data(), 3, 1);
      e << YAML::Key << "tolerance_ladder_applied" << YAML::Value << poi->tolerance_ladder_applied;
      e << YAML::Key << "euler_order" << YAML::Value << "xyz";
      e << YAML::Key << "units" << YAML::Value << "deg";
      e << YAML::EndMap;
    }
    if (verify) EmitVerification(e, *verify);
    e << YAML::Key << "paths" << YAML::Value << YAML::BeginSeq;
    for (const auto& item : doc.paths) {
      e << YAML::BeginMap;
      e << YAML::Key << "path_id" << YAML::Value << item.path_id;
      e << YAML::Key << "name" << YAML::Value << item.name;
      e << YAML::Key << "points" << YAML::Value << YAML::BeginSeq;
      for (const auto& wp : item.points) EmitWaypoint(e, wp);
      e << YAML::EndSeq;
      if (!item.dense_surface_points_mm.empty()) {
        e << YAML::Key << "dense_surface_points_base_mm" << YAML::Value << YAML::BeginSeq;
        for (const auto& p : item.dense_surface_points_mm) EmitFixedSeq(e, p.data(), 3, 2);
        e << YAML::EndSeq;
      }
      e << YAML::EndMap;
    }
    e << YAML::EndSeq;
    e << YAML::EndMap;
    std::ofstream ofs(path);
    if (!ofs) {
      if (err) *err = "cannot write " + path;
      return false;
    }
    ofs << e.c_str() << "\n";
    return true;
  } catch (const std::exception& ex) {
    if (err) *err = ex.what();
    return false;
  }
}

namespace {

std::string Dirname(const std::string& p) {
  const auto n = p.find_last_of("/\\");
  return n == std::string::npos ? std::string(".") : p.substr(0, n);
}

std::string JoinPath(const std::string& a, const std::string& b) {
  if (b.empty()) return a;
  if (!b.empty() && (b[0] == '/' || (b.size() > 1 && b[1] == ':'))) return b;
  if (a.empty() || a == ".") return b;
  if (a.back() == '/') return a + b;
  return a + "/" + b;
}

bool FileReadable(const std::string& p) {
  std::ifstream in(p);
  return static_cast<bool>(in);
}

std::string ResolveRel(const std::string& p, const std::string& config_path) {
  if (p.empty() || FileReadable(p)) return p;
  const std::string cfg_dir = Dirname(config_path);
  const std::string next_to_cfg = JoinPath(cfg_dir, p);
  if (FileReadable(next_to_cfg)) return next_to_cfg;
  // configs/aisprayer_config.yaml → 仓库根
  const std::string repo = JoinPath(Dirname(cfg_dir), p);
  if (FileReadable(repo)) return repo;
  return p;
}

AxisGrid ReadGrid(const YAML::Node& n, const AxisGrid& def) {
  if (!n || !n.IsSequence() || n.size() < 3) return def;
  return {n[0].as<double>(), n[1].as<double>(), n[2].as<double>()};
}

Eigen::Vector3d ReadVec3(const YAML::Node& n, const Eigen::Vector3d& def) {
  if (!n || !n.IsSequence() || n.size() < 3) return def;
  return {n[0].as<double>(), n[1].as<double>(), n[2].as<double>()};
}

}  // namespace

bool LoadSprayingConfig(const std::string& yaml_path, SprayingConfig& out, std::string* err) {
  try {
    YAML::Node root = YAML::LoadFile(yaml_path);
    const auto robot = root["hardware"]["robot"];
    const auto spraying = root["spraying"];
    if (robot) {
      if (robot["robot_urdf"]) out.urdf_path = robot["robot_urdf"].as<std::string>();
      if (robot["robot_tcp"]) out.tool_name = robot["robot_tcp"].as<std::string>();
    }
    if (spraying) {
      if (spraying["velocity"]) out.speed_mm_s = spraying["velocity"].as<double>();
      if (spraying["slerp_step_mm"]) out.step_mm = spraying["slerp_step_mm"].as<double>();
      if (spraying["poi_anchor_source"])
        out.anchor_source = spraying["poi_anchor_source"].as<std::string>();
      out.ref_rpy_deg = ReadVec3(spraying["poi_ref_rpy_deg"], out.ref_rpy_deg);
      out.tol_deg = ReadVec3(spraying["poi_tolerance_rpy_deg"], out.tol_deg);
      out.grid_x = ReadGrid(spraying["grid_tol_x_deg"], out.grid_x);
      out.grid_y = ReadGrid(spraying["grid_tol_y_deg"], out.grid_y);
      out.grid_z = ReadGrid(spraying["grid_tol_z_deg"], out.grid_z);
      // 容差阶梯择优：部署侧可不重编译就关掉阶梯或调整护栏（CLI 显式传参优先级更高）。
      if (spraying["tol_ladder"]) out.tol_ladder = spraying["tol_ladder"].as<bool>();
      if (const auto sc = spraying["tol_ladder_scales"]; sc && sc.IsSequence() && sc.size() > 0) {
        out.tol_ladder_scales.clear();
        for (const auto& v : sc) out.tol_ladder_scales.push_back(v.as<double>());
      }
      if (spraying["tol_ladder_stop_peak_ratio"])
        out.tol_ladder_stop_peak_ratio = spraying["tol_ladder_stop_peak_ratio"].as<double>();
      if (spraying["tol_ladder_max_pointing_deg"])
        out.tol_ladder_max_pointing_deg = spraying["tol_ladder_max_pointing_deg"].as<double>();
      // ⑤ 边内关节速度约束：让 DP 选边时就按段时长折算真实 °/s，超速边加罚/硬禁。
      // 与 tol_ladder 一样放配置里，部署侧可不重编调整；CLI 显式传参优先级更高。
      if (spraying["opt_enforce_vel_limit"])
        out.opt_enforce_vel_limit = spraying["opt_enforce_vel_limit"].as<bool>();
      if (spraying["opt_vel_soft_ratio"])
        out.opt_vel_soft_ratio = spraying["opt_vel_soft_ratio"].as<double>();
      if (spraying["opt_vel_cost_weight"])
        out.opt_vel_cost_weight = spraying["opt_vel_cost_weight"].as<double>();
      if (spraying["opt_vel_hard_ratio"])
        out.opt_vel_hard_ratio = spraying["opt_vel_hard_ratio"].as<double>();
      // ⑤-A 腕部奇异自适应降速：按 |sin(J5)| 在近奇异段压低线速度，兼顾贴法向与不超速。
      if (spraying["singularity_speed_scaling"])
        out.singularity_scaling = spraying["singularity_speed_scaling"].as<bool>();
      if (spraying["singularity_ref_deg"])
        out.singularity_ref_deg = spraying["singularity_ref_deg"].as<double>();
      if (spraying["singularity_min_scale"])
        out.singularity_min_scale = spraying["singularity_min_scale"].as<double>();
    }
    out.urdf_path = ResolveRel(out.urdf_path, yaml_path);
    return true;
  } catch (const std::exception& e) {
    if (err) *err = e.what();
    return false;
  }
}

}  // namespace motion

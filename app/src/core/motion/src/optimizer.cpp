#include "motion/optimizer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace motion {
namespace {

constexpr double kSegmentTravelDeg = 120.0;
constexpr double kSegmentTravelCapDeg = 170.0;
constexpr double kSegmentTravelDegPerMm = 0.9;

JointVec DefaultSeed() {
  return (JointVec() << 0.0, 0.0, -kPi / 2.0, -kPi / 2.0, -kPi / 2.0, 0.0).finished();
}

struct DpNode {
  Transform T;
  JointVec q = JointVec::Zero();
  JointVec q_branch = JointVec::Zero();
  double pose_dev = 0.0;  // 候选姿态相对该航点名义(法向)姿态的加权偏离
  int branch_id = 0;
  int ew_family = 0;
};

struct CandPack {
  std::vector<Transform> T;
  std::vector<double> geo;       // 与名义姿态的测地距 (deg)
  std::vector<double> pose_dev;  // 与名义姿态的加权欧拉偏离 (deg²)
  std::vector<JointVec> q;
  std::vector<int> pose_idx;
  std::vector<int> branch_id;
};

// 候选排序分：姿态保真度为主（离名义法向越近越好），测地距为次。
// 分数只取决于姿态、与具体 IK 解无关 —— 同一姿态的 8 个分支同分。
// 分支内取前 max_candidates_per_branch 个。
double PoseScore(const CandPack& pack, int m) {
  const size_t pi = static_cast<size_t>(pack.pose_idx[static_cast<size_t>(m)]);
  return pack.pose_dev[pi] + 0.01 * pack.geo[pi];
}

DpNode MakeNode(const CandPack& pack, int m) {
  const size_t mm = static_cast<size_t>(m);
  const size_t pi = static_cast<size_t>(pack.pose_idx[mm]);
  DpNode nd;
  nd.T = pack.T[pi];
  nd.q = pack.q[mm];
  nd.q_branch = nd.q;
  nd.pose_dev = pack.pose_dev[pi];
  nd.branch_id = pack.branch_id[mm];
  nd.ew_family = nd.branch_id & 3;
  return nd;
}

// 展开全部候选（不做分支内截断），用于 beam 剪枝后整层不可达时的回退重试。
std::vector<DpNode> Materialize(const CandPack& pack) {
  std::vector<DpNode> nodes;
  nodes.reserve(pack.q.size());
  for (size_t m = 0; m < pack.q.size(); ++m) nodes.push_back(MakeNode(pack, static_cast<int>(m)));
  return nodes;
}

Eigen::Matrix3d AxisRot(double deg, int axis) {
  const double a = Rad(deg), c = std::cos(a), s = std::sin(a);
  Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
  if (axis == 0) {
    m << 1, 0, 0, 0, c, -s, 0, s, c;
  } else if (axis == 1) {
    m << c, 0, s, 0, 1, 0, -s, 0, c;
  } else {
    m << c, -s, 0, s, c, 0, 0, 0, 1;
  }
  return m;
}

std::vector<Eigen::Matrix3d> PrecomputeOffsets(const AxisGrid& gx, const AxisGrid& gy,
                                               const AxisGrid& gz) {
  const auto xs = ExpandAxisGrid(gx);
  const auto ys = ExpandAxisGrid(gy);
  const auto zs = ExpandAxisGrid(gz);
  std::vector<Eigen::Matrix3d> Rx, Ry, Rz;
  Rx.reserve(xs.size());
  Ry.reserve(ys.size());
  Rz.reserve(zs.size());
  for (double x : xs) Rx.push_back(AxisRot(x, 0));
  for (double y : ys) Ry.push_back(AxisRot(y, 1));
  for (double z : zs) Rz.push_back(AxisRot(z, 2));
  std::vector<Eigen::Matrix3d> out;
  out.reserve(xs.size() * ys.size() * zs.size());
  for (const auto& rz : Rz)
    for (const auto& ry : Ry)
      for (const auto& rx : Rx) out.push_back(rz * ry * rx);

  // The grid is commonly symmetric with an even step, e.g. [-15, 15, 2].
  // Such a grid does not contain zero, so without these explicit candidates the
  // optimizer cannot retain the nominal attitude or change only the spray-axis
  // spin.  Those are the first attitudes that must be considered.
  out.push_back(Eigen::Matrix3d::Identity());
  for (double z : zs) out.push_back(AxisRot(z, 2));
  return out;
}

// Add a low-cost, normal-preserving rescue family.  A wide user envelope must
// not be silently narrowed to the configured grid: when the nominal pose has
// no IK, a rotation about the nozzle axis can often restore IK without moving
// the spray direction.  We deliberately add only this one-dimensional family
// here; expanding all three axes across a ±180° envelope would turn every
// waypoint into an impractical three-dimensional exhaustive search.
void AppendFullSpinOffsets(std::vector<Eigen::Matrix3d>& offsets, const AxisGrid& grid,
                           double envelope_z_deg) {
  const double limit = std::min(180.0, std::abs(envelope_z_deg));
  if (limit <= 0.0) return;
  const double step = grid.step_deg > 0.0 ? grid.step_deg : 5.0;
  for (double z = -limit; z <= limit + 1e-9; z += step) {
    offsets.push_back(AxisRot(std::max(-limit, std::min(limit, z)), 2));
  }
  offsets.push_back(AxisRot(-limit, 2));
  offsets.push_back(AxisRot(limit, 2));
}

Eigen::Matrix3d ProjectToAnchor(const Eigen::Matrix3d& R_cand, const Eigen::Matrix3d& R_anc,
                                const Eigen::Vector3d& tol_deg) {
  Eigen::Vector3d rel = CtrlRpyDegFromRot(R_anc.transpose() * R_cand);
  rel[0] = Wrap180(rel[0]);
  rel[1] = Wrap180(rel[1]);
  rel[2] = Wrap180(rel[2]);
  Eigen::Vector3d clipped;
  for (int i = 0; i < 3; ++i) {
    const double t = std::abs(tol_deg[i]);
    clipped[i] = std::min(t, std::max(-t, rel[i]));
  }
  if ((clipped - rel).cwiseAbs().maxCoeff() <= 1e-12) return R_cand;
  return R_anc * RotFromCtrlRpyDeg(clipped);
}

std::optional<JointVec> UnwrapOnto(const JointVec& q_sol, const JointVec& q_ref,
                                   const Cr5Kinematics& kin) {
  const JointVec q_u = q_ref + WrapPi(q_sol - q_ref);
  if (kin.IsJointValid(q_u)) return q_u;
  if (kin.IsJointValid(q_sol)) return q_sol;
  return std::nullopt;
}

bool IsSafeQ(const Cr5Kinematics& kin, const JointVec& q, const Transform& T_gun,
             const ToolOffset& tool) {
  if (!kin.IsJointValid(q)) return false;
  if (std::abs(std::sin(q[4])) < kSingSin || std::abs(std::sin(q[2])) < kSingSin) return false;
  const Transform T_urdf = CtrlToUrdf(T_gun * tool.T_tcp_inv);
  return ShoulderHalfRad(T_urdf) >= kShoulderHalfRad;
}

std::vector<double> AlphasFor(int n_mid, std::unordered_map<int, std::vector<double>>& cache) {
  auto it = cache.find(n_mid);
  if (it != cache.end()) return it->second;
  std::vector<double> a;
  a.reserve(static_cast<size_t>(n_mid + 1));
  const int n = n_mid + 2;
  for (int i = 1; i < n; ++i) a.push_back(static_cast<double>(i) / static_cast<double>(n - 1));
  cache[n_mid] = a;
  return a;
}

struct QuatKey {
  long long x, y, z, w;
  bool operator==(const QuatKey& o) const {
    return x == o.x && y == o.y && z == o.z && w == o.w;
  }
};
struct QuatKeyHash {
  size_t operator()(const QuatKey& k) const {
    return static_cast<size_t>(k.x * 1315423911LL ^ k.y ^ (k.z << 7) ^ (k.w << 13));
  }
};
QuatKey MakeQuatKey(const Eigen::Vector4d& q) {
  Eigen::Vector4d qq = q[3] < 0.0 ? -q : q;
  auto rnd = [](double v) { return static_cast<long long>(std::llround(v * 1e5)); };
  return {rnd(qq[0]), rnd(qq[1]), rnd(qq[2]), rnd(qq[3])};
}

std::pair<std::vector<DpNode>, CandPack> GenerateCandidates(
    const Cr5Kinematics& kin, const ToolOffset& tool, const Transform& T_nom,
    const std::optional<Eigen::Matrix3d>& R_anchor, const Eigen::Vector3d& tol,
    const std::vector<Eigen::Matrix3d>& R_off, const OptimizeOptions& opt) {
  const Eigen::Vector3d pos = T_nom.translation();
  const Eigen::Matrix3d R_nom = T_nom.linear();
  std::vector<Eigen::Matrix3d> offsets = R_off;
  // Add the full requested self-spin interval independently of the coarse
  // three-axis grid.  Projection keeps global-anchor candidates inside their
  // envelope; in raw mode this is a direction-preserving recovery family.
  if (R_anchor) AppendFullSpinOffsets(offsets, opt.grid_z, tol[2]);
  const size_t N = offsets.size();
  std::vector<Eigen::Matrix3d> R_cands(N);
  for (size_t i = 0; i < N; ++i) {
    R_cands[i] = R_nom * offsets[i];
    if (R_anchor) R_cands[i] = ProjectToAnchor(R_cands[i], *R_anchor, tol);
  }

  std::vector<double> geo(N), pose_dev(N);
  std::vector<Eigen::Vector4d> quats(N);
  // key → keep 中的槽位。Python 侧 dict.values() 是「首次插入序」，而 unordered_map
  // 的迭代序未定义；候选顺序会经 pose_idx 传导到后面 stable_sort 的平局判定，
  // 所以这里必须显式保留首次出现序，否则结果随 STL 实现漂移。
  std::unordered_map<QuatKey, size_t, QuatKeyHash> slot_of;
  std::vector<size_t> keep;
  for (size_t i = 0; i < N; ++i) {
    geo[i] = GeodesicDeg(R_nom, R_cands[i]);
    quats[i] = QuatXyzw(R_cands[i]);
    // The primary quality target is the nozzle direction relative to the
    // original surface normal, not an Euler-coordinate distance.  Spin is
    // retained as a very small tie breaker for asymmetric nozzles.
    const double pointing = PointingDeg(R_nom, R_cands[i]);
    Eigen::Vector3d e = CtrlRpyDegFromRot(R_nom.transpose() * R_cands[i]);
    e[2] = Wrap180(e[2]);
    pose_dev[i] = opt.pointing_cost_weight * pointing * pointing +
                  opt.spin_cost_weight * e[2] * e[2];
    const QuatKey key = MakeQuatKey(quats[i]);
    auto it = slot_of.find(key);
    if (it == slot_of.end()) {
      slot_of.emplace(key, keep.size());
      keep.push_back(i);
    } else if (geo[i] < geo[keep[it->second]]) {
      keep[it->second] = i;  // 同姿态取测地距更小者，槽位（即顺序）不变
    }
  }
  const int P = static_cast<int>(keep.size());
  CandPack empty;
  if (P == 0) return {{}, empty};

  std::vector<Transform> T_ctrl(P), T_urdf(P);
  std::vector<double> geo_keep(P), dev_keep(P);
  for (int i = 0; i < P; ++i) {
    const size_t src = keep[static_cast<size_t>(i)];
    T_ctrl[i] = Transform::Identity();
    T_ctrl[i].linear() = R_cands[src];
    T_ctrl[i].translation() = pos;
    T_urdf[i] = CtrlToUrdf(T_ctrl[i] * tool.T_tcp_inv);
    geo_keep[i] = geo[src];
    dev_keep[i] = pose_dev[src];
  }

  std::vector<JointVec> q_sols(static_cast<size_t>(P) * 8);
  std::vector<int> n_sols(P);
  kin.IkBatch(T_urdf.data(), P, q_sols.data(), n_sols.data());

  CandPack pack;
  std::vector<int> remap(P, -1);
  int kept = 0;
  for (int i = 0; i < P; ++i) {
    if (n_sols[i] <= 0) continue;
    if (ShoulderHalfRad(T_urdf[i]) < kShoulderHalfRad) continue;
    remap[i] = kept++;
    pack.T.push_back(T_ctrl[i]);
    pack.geo.push_back(geo_keep[i]);
    pack.pose_dev.push_back(dev_keep[i]);
  }
  if (kept == 0) return {{}, empty};

  for (int i = 0; i < P; ++i) {
    if (remap[i] < 0) continue;
    for (int k = 0; k < n_sols[i] && k < 8; ++k) {
      const JointVec& q = q_sols[static_cast<size_t>(i) * 8 + k];
      if (!kin.IsJointValid(q)) continue;
      if (std::abs(std::sin(q[4])) < kSingSin || std::abs(std::sin(q[2])) < kSingSin) continue;
      pack.q.push_back(q);
      pack.pose_idx.push_back(remap[i]);
      pack.branch_id.push_back(BranchKey(q));
    }
  }
  if (pack.q.empty()) return {{}, empty};

  std::vector<std::vector<int>> buckets(8);
  for (size_t m = 0; m < pack.q.size(); ++m) {
    buckets[static_cast<size_t>(pack.branch_id[m])].push_back(static_cast<int>(m));
  }
  std::vector<DpNode> fast;
  for (int b = 0; b < 8; ++b) {
    auto& idx = buckets[static_cast<size_t>(b)];
    std::stable_sort(idx.begin(), idx.end(), [&](int a, int b2) {
      const double sa = PoseScore(pack, a);
      const double sb = PoseScore(pack, b2);
      if (sa < sb) return true;
      if (sa > sb) return false;
      return a < b2;
    });
    const int take = std::min(opt.max_candidates_per_branch, static_cast<int>(idx.size()));
    for (int t = 0; t < take; ++t) fast.push_back(MakeNode(pack, idx[static_cast<size_t>(t)]));
  }
  // 到这里 pack.q 非空 ⇒ 每个非空分支桶至少贡献 1 个节点（take ≥ 1），fast 必非空；
  // Validate() 已保证 max_candidates_per_branch ≥ 1，所以无需再做全量兜底。
  return {fast, pack};
}

struct EdgeOut {
  bool ok = false;
  double cost = std::numeric_limits<double>::infinity();
  JointVec q = JointVec::Zero();
};

EdgeOut CheckMoveL(const Cr5Kinematics& kin, const ToolOffset& tool, const OptimizeOptions& opt,
                   const DpNode& a, const DpNode& b, const JointVec& q_start, bool is_jump,
                   std::unordered_map<int, std::vector<double>>& alpha_cache) {
  EdgeOut out;
  if (is_jump) {
    auto hint = UnwrapOnto(b.q_branch, q_start, kin);
    if (!hint) hint = b.q_branch;
    if (!IsSafeQ(kin, *hint, b.T, tool)) return out;
    JointVec dq;
    for (int j = 0; j < 6; ++j) {
      const double span = kin.limits().max_rad[j] - kin.limits().min_rad[j];
      const double unwrapped_j = q_start[j] + WrapPi((*hint)[j] - q_start[j]);
      if (span > 2.0 * kPi + 0.1 || (unwrapped_j >= kin.limits().min_rad[j] - kJointTol &&
                                     unwrapped_j <= kin.limits().max_rad[j] + kJointTol)) {
        dq[j] = WrapPi((*hint)[j] - q_start[j]);
      } else {
        // 有界关节无法跨越 ±180° 硬件限位，其实际转动量为未环绕的真实角位移
        dq[j] = (*hint)[j] - q_start[j];
      }
    }
    double cost = 0.0;
    for (int j = 0; j < 6; ++j) {
      const double d = Deg(dq[j]);
      cost += opt.joint_weights[j] * d * d;
    }
    out.ok = true;
    out.cost = cost;
    out.q = *hint;
    return out;
  }

  auto hint = UnwrapOnto(b.q_branch, q_start, kin);
  if (!hint) return out;
  if ((a.ew_family) != (b.ew_family)) return out;

  // Keep the DP edge model identical to the executed / verified command:
  // interpolate a TCP MoveL, then apply the fixed TCP→flange transform for IK.
  const Eigen::Vector3d p0 = a.T.translation();
  const Eigen::Vector3d p1 = b.T.translation();
  const double dist_mm = (p1 - p0).norm() * kMmPerM;
  // 段时长 dt_seg = 段长(mm) / 线速度(mm/s) → 秒；用于把关节位移折算成真实 °/s。
  const double dt_seg = (opt.exec_speed_mm_s > 0.0) ? dist_mm / opt.exec_speed_mm_s : 0.0;
  double travel = 0.0;
  {
    const JointVec dq = WrapPi(*hint - q_start);
    for (int j = 0; j < 6; ++j) travel = std::max(travel, std::abs(Deg(dq[j])));
  }
  const double travel_lim =
      std::min(kSegmentTravelCapDeg, std::max(kSegmentTravelDeg, kSegmentTravelDegPerMm * dist_mm));
  if (travel > travel_lim) return out;

  const int n_est = static_cast<int>(std::lround(dist_mm / opt.movel_spacing_mm));
  const int n_mid = std::min(opt.movel_checks_max, std::max(opt.movel_checks_min, n_est));
  const auto alphas = AlphasFor(n_mid, alpha_cache);

  MoveLQuery q;
  q.p_start_m = p0;
  q.p_end_m = p1;
  q.quat1_xyzw = QuatXyzw(a.T.linear());
  q.quat2_xyzw = QuatXyzw(b.T.linear());
  q.q_start = q_start;
  q.alphas = alphas;
  q.q_branch_end = b.q_branch;
  q.check_end_branch = true;
  q.max_jump_rad = Rad(kBranchJumpDeg);
  q.match_rad = Rad(kEndBranchMatchDeg);
  q.weights = opt.joint_weights;
  auto walk = SegmentChecker(kin, tool).Walk(q);
  if (!walk) return out;
  out.ok = true;
  out.cost = walk->cost;
  out.q = walk->q_end;

  // ⑤ 边内关节速度约束：把该段按执行线速度折算成真实 °/s。
  // 超硬阈(ratio > vel_hard_ratio) 的边直接判不可行 → DP 换 IK 分支或在包络内换姿态；
  // 超软阈(ratio > vel_soft_ratio) 的边加二次惩罚 → 未达硬阈时也倾向选更慢的链。
  // 注意：此处用段两端关节差（持续型超速，如 J4~200°/s）折算，与校验器同口径。
  if (opt.enforce_vel_limit && dt_seg > 1e-9) {
    // ⑤-A 与校验器同口径：近奇异段执行侧会降速，dt 放大 1/scale，
    // 故该边折算角速度按降速后评估，避免 DP 把奇异段误判为不可行而退到更差分支。
    double eff_dt = dt_seg;
    if (opt.singularity_scaling) {
      const double sc = std::min(
          SingularitySpeedScale(q_start[4], Rad(opt.singularity_ref_deg), opt.singularity_min_scale),
          SingularitySpeedScale(out.q[4], Rad(opt.singularity_ref_deg), opt.singularity_min_scale));
      if (sc > 0.0) eff_dt = dt_seg / sc;
    }
    const JointVec dq_seg = WrapPi(out.q - q_start);
    double vel_pen = 0.0;
    bool over_hard = false;
    for (int j = 0; j < 6; ++j) {
      const double lim = kin.limits().max_vel_deg_s[j];
      if (lim <= 0.0) continue;
      const double ratio = (std::abs(Deg(dq_seg[j])) / eff_dt) / lim;  // 峰值角速度 / 限速
      if (opt.vel_hard_ratio > 0.0 && ratio > opt.vel_hard_ratio) {
        over_hard = true;
        break;
      }
      if (ratio > opt.vel_soft_ratio) {
        const double x = ratio - opt.vel_soft_ratio;
        vel_pen += x * x;
      }
    }
    if (over_hard) {
      out.ok = false;
      out.q = JointVec::Zero();
      return out;
    }
    out.cost += opt.vel_cost_weight * vel_pen;
  }
  return out;
}

void BeamKeep(std::vector<double>& cost, int beam) {
  std::vector<int> finite;
  for (int i = 0; i < static_cast<int>(cost.size()); ++i) {
    if (std::isfinite(cost[static_cast<size_t>(i)])) finite.push_back(i);
  }
  if (static_cast<int>(finite.size()) <= beam) return;
  std::sort(finite.begin(), finite.end(),
            [&](int a, int b) { return cost[static_cast<size_t>(a)] < cost[static_cast<size_t>(b)]; });
  for (size_t i = static_cast<size_t>(beam); i < finite.size(); ++i) {
    cost[static_cast<size_t>(finite[i])] = std::numeric_limits<double>::infinity();
  }
}

}  // namespace

std::string OptimizeOptions::Validate() const {
  if (beam_width < 8) return "beam_width must be >= 8";
  if (max_candidates_per_branch < 1) return "max_candidates_per_branch must be >= 1";
  if (movel_spacing_mm <= 0.0) return "movel_spacing_mm must be > 0";
  if (movel_checks_min < 1 || movel_checks_max < movel_checks_min) {
    return "movel_checks_min/max must satisfy 1 <= min <= max";
  }
  if (!(pointing_cost_weight > 0.0) || !(spin_cost_weight >= 0.0)) {
    return "pointing_cost_weight must be > 0 and spin_cost_weight must be >= 0";
  }
  if (enforce_vel_limit) {
    // 速度约束必须拿到正线速度才能折算 dt；soft/hard 比例需为正且 hard ≥ soft。
    if (!(exec_speed_mm_s > 0.0)) return "exec_speed_mm_s must be > 0 when enforce_vel_limit";
    if (!(vel_soft_ratio > 0.0) || vel_soft_ratio > 2.0) return "vel_soft_ratio must be within (0, 2]";
    if (!(vel_cost_weight >= 0.0)) return "vel_cost_weight must be >= 0";
    if (vel_hard_ratio < 0.0) return "vel_hard_ratio must be >= 0 (0 = penalty only, no hard ban)";
    if (vel_hard_ratio > 0.0 && vel_hard_ratio < vel_soft_ratio)
      return "vel_hard_ratio must be >= vel_soft_ratio (or 0 to disable hard ban)";
  }
  if (singularity_scaling) {
    // |J5| 参考角必须在 (0,90]；缩放下限在 (0,1]（=1 等于不减速）。
    if (!(singularity_ref_deg > 0.0) || singularity_ref_deg > 90.0)
      return "singularity_ref_deg must be within (0, 90]";
    if (!(singularity_min_scale > 0.0) || singularity_min_scale > 1.0)
      return "singularity_min_scale must be within (0, 1]";
  }
  if (tol_ladder) {
    // 比例必须在 (0,1)：≥1 等于重跑请求档，≤0 会把包络塌缩成空集。
    for (double s : tol_ladder_scales) {
      if (!(s > 0.0) || !(s < 1.0)) return "tol_ladder_scales must all be within (0, 1)";
    }
    if (!(tol_ladder_stop_peak_ratio >= 0.0) || tol_ladder_stop_peak_ratio > 1.0) {
      return "tol_ladder_stop_peak_ratio must be within [0, 1]";
    }
    if (!(tol_ladder_max_pointing_deg >= 0.0) || tol_ladder_max_pointing_deg > 180.0) {
      return "tol_ladder_max_pointing_deg must be within [0, 180] (0 = unlimited)";
    }
  }
  return {};
}

ViterbiOptimizer::ViterbiOptimizer(const Cr5Kinematics& kin, const ToolOffset& tool,
                                   OptimizeOptions opt, const ChainVerifier* verifier)
    : kin_(kin), tool_(tool), opt_(std::move(opt)), verifier_(verifier) {}

OptimizeResult ViterbiOptimizer::OptimizeOnce(const PathItem& path, const Anchor& anchor,
                                             std::optional<JointVec> init_q,
                                             const std::string& log_tag, bool log_segments) const {
  OptimizeResult result;
  result.path = path;
  result.adopted_tol_deg = anchor.tol_deg;
  if (path.points.empty()) return result;

  const auto t0 = std::chrono::steady_clock::now();
  const JointVec q_seed = init_q.value_or(DefaultSeed());
  const auto R_off = PrecomputeOffsets(opt_.grid_x, opt_.grid_y, opt_.grid_z);
  const int n = static_cast<int>(path.points.size());

  const auto t_cands0 = std::chrono::steady_clock::now();
  std::vector<std::vector<DpNode>> stages(n);
  std::vector<CandPack> packs(n);
  for (int i = 0; i < n; ++i) {
    std::optional<Eigen::Matrix3d> R_a;
    if (anchor.has_global()) {
      R_a = *anchor.R;
    } else {
      // A zero raw envelope means "retain this exact nominal attitude", not
      // "disable the envelope".  Keep the local anchor for every tolerance.
      R_a = path.points[static_cast<size_t>(i)].tcp_pose.linear();
    }
    auto [fast, pack] = GenerateCandidates(kin_, tool_, path.points[static_cast<size_t>(i)].tcp_pose,
                                           R_a, anchor.tol_deg, R_off, opt_);
    if (fast.empty() && pack.q.empty()) {
      throw std::runtime_error("Waypoint [" + std::to_string(i) +
                               "] has no non-singular in-limit IK inside the attitude envelope.");
    }
    stages[static_cast<size_t>(i)] = std::move(fast);
    packs[static_cast<size_t>(i)] = std::move(pack);
  }
  {
    int total_fast = 0, total_full = 0;
    for (int i = 0; i < n; ++i) {
      total_fast += static_cast<int>(stages[static_cast<size_t>(i)].size());
      total_full += static_cast<int>(packs[static_cast<size_t>(i)].q.size());
    }
    const double t_cands_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_cands0).count();
    std::cerr << "⏱️ [SprayOpt]" << log_tag << " Stage candidates generated: " << n
              << " waypoints, fast_candidates=" << total_fast
              << " (diverse 8-branch pruned), full_nodes=" << total_full
              << ", elapsed=" << std::fixed << std::setprecision(2) << t_cands_ms << " ms\n"
              << std::flush;
  }

  if (n == 1) {
    auto& s0 = stages[0];
    int best = 0;
    double best_c = std::numeric_limits<double>::infinity();
    for (int j = 0; j < static_cast<int>(s0.size()); ++j) {
      const double c = s0[static_cast<size_t>(j)].pose_dev +
                       WrapPi(s0[static_cast<size_t>(j)].q - q_seed).squaredNorm();
      if (c < best_c) {
        best_c = c;
        best = j;
      }
    }
    const DpNode& picked = s0[static_cast<size_t>(best)];
    result.modified = !picked.T.linear().isApprox(path.points[0].tcp_pose.linear(), 1e-6);
    result.path.points[0].tcp_pose = picked.T;
    result.joints_rad.push_back(picked.q);
    result.objective = best_c;
    // 单航点路径以前会在早退里漏掉耗时统计，报表永远显示 0.00 ms。
    result.elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return result;
  }

  const auto t_dp0 = std::chrono::steady_clock::now();
  std::vector<std::vector<double>> dp_cost(n);
  std::vector<std::vector<int>> dp_parent(n);
  std::vector<std::vector<JointVec>> dp_q(n);
  for (int i = 0; i < n; ++i) {
    const size_t m = stages[static_cast<size_t>(i)].size();
    dp_cost[static_cast<size_t>(i)].assign(m, std::numeric_limits<double>::infinity());
    dp_parent[static_cast<size_t>(i)].assign(m, -1);
    dp_q[static_cast<size_t>(i)].assign(m, JointVec::Zero());
  }

  for (int j = 0; j < static_cast<int>(stages[0].size()); ++j) {
    auto& node = stages[0][static_cast<size_t>(j)];
    auto q0 = UnwrapOnto(node.q, q_seed, kin_);
    if (!q0 || !IsSafeQ(kin_, *q0, node.T, tool_)) continue;
    const JointVec d = WrapPi(*q0 - q_seed);
    double extra = 0.0;
    for (int k = 0; k < 6; ++k) extra += opt_.joint_weights[k] * Deg(d[k]) * Deg(d[k]);
    dp_cost[0][static_cast<size_t>(j)] = node.pose_dev + 0.05 * extra;
    dp_q[0][static_cast<size_t>(j)] = *q0;
    node.q = *q0;
  }
  BeamKeep(dp_cost[0], opt_.beam_width);

  std::unordered_map<int, std::vector<double>> alpha_cache;
  struct LayerOut {
    std::vector<double> cost;
    std::vector<int> parent;
    std::vector<JointVec> qq;
    int tested = 0;
    int valid = 0;
  };
  auto eval_layer = [&](int i, std::vector<DpNode>& curr) -> LayerOut {
    LayerOut out;
    const size_t m = curr.size();
    out.cost.assign(m, std::numeric_limits<double>::infinity());
    out.parent.assign(m, -1);
    out.qq.assign(m, JointVec::Zero());
    const bool is_jump = path.points[static_cast<size_t>(i)].is_jump ||
                         !path.points[static_cast<size_t>(i)].spraying;
    for (int pk = 0; pk < static_cast<int>(stages[static_cast<size_t>(i - 1)].size()); ++pk) {
      if (!std::isfinite(dp_cost[static_cast<size_t>(i - 1)][static_cast<size_t>(pk)])) continue;
      const JointVec& q_prev = dp_q[static_cast<size_t>(i - 1)][static_cast<size_t>(pk)];
      const auto& prev = stages[static_cast<size_t>(i - 1)][static_cast<size_t>(pk)];
      for (int cj = 0; cj < static_cast<int>(curr.size()); ++cj) {
        ++out.tested;
        const EdgeOut e = CheckMoveL(kin_, tool_, opt_, prev, curr[static_cast<size_t>(cj)], q_prev,
                                     is_jump, alpha_cache);
        if (!e.ok) continue;
        ++out.valid;
        const double total =
            dp_cost[static_cast<size_t>(i - 1)][static_cast<size_t>(pk)] + e.cost +
            curr[static_cast<size_t>(cj)].pose_dev;
        if (total < out.cost[static_cast<size_t>(cj)]) {
          out.cost[static_cast<size_t>(cj)] = total;
          out.parent[static_cast<size_t>(cj)] = pk;
          out.qq[static_cast<size_t>(cj)] = e.q;
        }
      }
    }
    return out;
  };

  int total_edges = 0, total_feasible = 0, total_fallbacks = 0;
  for (int i = 1; i < n; ++i) {
    const auto t_seg0 = std::chrono::steady_clock::now();
    int edges_tested = 0;
    int valid_edges = 0;
    LayerOut layer = eval_layer(i, stages[static_cast<size_t>(i)]);
    edges_tested += layer.tested;
    valid_edges += layer.valid;
    bool any = false;
    for (double v : layer.cost)
      if (std::isfinite(v)) any = true;
    if (!any && static_cast<int>(packs[static_cast<size_t>(i)].q.size()) >
                    static_cast<int>(stages[static_cast<size_t>(i)].size())) {
      const int full_n = static_cast<int>(packs[static_cast<size_t>(i)].q.size());
      ++total_fallbacks;
      std::cerr << "⚠️ [SprayOpt]" << log_tag << " Segment " << (i - 1) << "->" << i
                << " failed with pruned candidates (" << stages[static_cast<size_t>(i)].size()
                << "). Triggering adaptive fallback with ALL " << full_n << " candidates...\n"
                << std::flush;
      stages[static_cast<size_t>(i)] = Materialize(packs[static_cast<size_t>(i)]);
      layer = eval_layer(i, stages[static_cast<size_t>(i)]);
      edges_tested += layer.tested;
      valid_edges += layer.valid;
      any = false;
      for (double v : layer.cost)
        if (std::isfinite(v)) any = true;
    }
    BeamKeep(layer.cost, opt_.beam_width);
    total_edges += edges_tested;
    total_feasible += valid_edges;
    if (log_segments) {
      const double t_seg_ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_seg0).count();
      std::cerr << "⏱️ [SprayOpt] DP Segment " << (i - 1) << "->" << i << ": edges=" << edges_tested
                << " (feasible=" << valid_edges << "), elapsed=" << std::fixed << std::setprecision(2)
                << t_seg_ms << " ms\n"
                << std::flush;
    }
    if (!any) {
      throw std::runtime_error("Global search failed at segment " + std::to_string(i - 1) + "->" +
                               std::to_string(i));
    }
    dp_cost[static_cast<size_t>(i)] = std::move(layer.cost);
    dp_parent[static_cast<size_t>(i)] = std::move(layer.parent);
    dp_q[static_cast<size_t>(i)] = std::move(layer.qq);
  }
  {
    const double t_dp_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_dp0).count();
    // 阶梯模式下不打逐段明细，这里把全量边统计补回来，保留剪枝/回退的可观测性。
    std::cerr << "⏱️ [SprayOpt]" << log_tag << " Total Viterbi DP search: " << (n - 1)
              << " segments, edges=" << total_edges << " (feasible=" << total_feasible
              << ", fallback=" << total_fallbacks << "), elapsed=" << std::fixed
              << std::setprecision(2) << t_dp_ms << " ms\n"
              << std::flush;
  }

  int last = 0;
  double best = std::numeric_limits<double>::infinity();
  for (int j = 0; j < static_cast<int>(dp_cost.back().size()); ++j) {
    if (dp_cost.back()[static_cast<size_t>(j)] < best) {
      best = dp_cost.back()[static_cast<size_t>(j)];
      last = j;
    }
  }
  std::vector<int> chain(n);
  chain[static_cast<size_t>(n - 1)] = last;
  for (int i = n - 1; i > 0; --i) {
    last = dp_parent[static_cast<size_t>(i)][static_cast<size_t>(last)];
    // 代价有限的节点必然有父节点；若不变式被破坏，宁可报错也不要负索引越界。
    if (last < 0) {
      throw std::runtime_error("Backtrack broken at waypoint " + std::to_string(i) +
                               " (no parent for a finite-cost node).");
    }
    chain[static_cast<size_t>(i - 1)] = last;
  }

  result.joints_rad.resize(n);
  bool modified = false;
  for (int i = 0; i < n; ++i) {
    const auto& nd = stages[static_cast<size_t>(i)][static_cast<size_t>(chain[static_cast<size_t>(i)])];
    if (!nd.T.linear().isApprox(path.points[static_cast<size_t>(i)].tcp_pose.linear(), 1e-6)) {
      modified = true;
    }
    result.path.points[static_cast<size_t>(i)].tcp_pose = nd.T;
    result.joints_rad[static_cast<size_t>(i)] = dp_q[static_cast<size_t>(i)][static_cast<size_t>(chain[static_cast<size_t>(i)])];
  }
  result.modified = modified;
  result.objective = best;  // DP 回溯起点的累计代价（已含末节点的姿态偏置）

  if (verifier_ && opt_.dense_verify && n >= 2) {
    const JointVec seed_for_verify = result.joints_rad.empty() ? q_seed : result.joints_rad[0];
    result.verify = verifier_->Verify(result.path, seed_for_verify);
    bool hard = result.verify.status == "FAILED";
    for (const auto& iss : result.verify.issues) {
      if (iss.severity == "ERROR" || iss.type.find("SINGULARITY") != std::string::npos) hard = true;
    }
    if (hard) {
      result.verify.status = "FAILED";
    }
  }

  const auto t1 = std::chrono::steady_clock::now();
  result.elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return result;
}

namespace {

// 阶梯档位：请求档 + 按比例逐分量收紧的若干档（去重、丢弃退化档）。
// 每一档都满足 tol_k ≤ tol_request（逐分量），所以任何一档的解都在用户请求的包络内。
std::vector<Eigen::Vector3d> BuildToleranceLadder(const Eigen::Vector3d& tol,
                                                 const OptimizeOptions& opt) {
  std::vector<Eigen::Vector3d> rungs;
  rungs.push_back(tol);
  if (!opt.tol_ladder) return rungs;
  for (double s : opt.tol_ladder_scales) {
    const Eigen::Vector3d t = tol * s;
    if (t.maxCoeff() <= 0.0) continue;  // 全零包络只剩锚点本身，没有搜索价值
    bool dup = false;
    for (const auto& r : rungs) {
      if ((r - t).cwiseAbs().maxCoeff() < 1e-9) {
        dup = true;
        break;
      }
    }
    if (!dup) rungs.push_back(t);
  }
  return rungs;
}

// 择优标尺（全部与容差无关，所以跳档可比）：
//   ① 校验状态 PASS(0) < WARNING/UNVERIFIED(1) < FAILED(2)
//   ② 指向偏量护栏（未越界优先）
//   ③ 最大指向偏量（喷嘴偏离表面法向）
//   ④ 峰值角速度 / 关节限速
//   ⑤ DP 总代价 J
int StatusRank(const std::string& status) {
  if (status == "PASS") return 0;
  if (status == "FAILED") return 2;
  return 1;
}

// 搜索网格每轴的最大偏移绝对值（候选姿态相对名义姿态的摆幅）。
Eigen::Vector3d GridRadius(const OptimizeOptions& opt) {
  auto r = [](const AxisGrid& g) { return std::max(std::abs(g.min_deg), std::abs(g.max_deg)); };
  return {r(opt.grid_x), r(opt.grid_y), r(opt.grid_z)};
}

// 某一档包络是否“惰性”：包络大到不会钳位任何候选时，它与更宽的档产出完全相同的
// 候选集（因而相同的 DP 结果），再跑一次纯属浪费时间。raw 模式下尤其明显：
// 锚点就是逐点名义姿态，候选的相对偏移恰好等于网格本身，包络 [30,30,180] 对
// 网格 (±5,±5,±30) 而言四档全部等价。
// 全局锚点模式下欧拉分量合成不是严格可加的，这里用 |rel_nom| + Σ网格半径 做保守上界：
// 只在“确定不会钳位”时跳过（估大了只是多跑一档，不会错跳）。
bool EnvelopeInert(const PathItem& path, const Anchor& anchor, const Eigen::Vector3d& tol,
                   const OptimizeOptions& opt) {
  const Eigen::Vector3d grid = GridRadius(opt);
  if (!anchor.has_global()) {
    for (int i = 0; i < 3; ++i) {
      if (std::abs(tol[i]) < grid[i]) return false;
    }
    return true;
  }
  Eigen::Vector3d max_rel = Eigen::Vector3d::Zero();
  for (const auto& wp : path.points) {
    const Eigen::Vector3d rel = CtrlRpyDegFromRot(anchor.R->transpose() * wp.tcp_pose.linear());
    for (int i = 0; i < 3; ++i) max_rel[i] = std::max(max_rel[i], std::abs(Wrap180(rel[i])));
  }
  const double grid_sum = grid[0] + grid[1] + grid[2];
  for (int i = 0; i < 3; ++i) {
    if (std::abs(tol[i]) < max_rel[i] + grid_sum) return false;
  }
  return true;
}

double PeakMax(const PathVerifyReport& rep) {
  return *std::max_element(rep.peak_joint_speeds_deg_s.begin(), rep.peak_joint_speeds_deg_s.end());
}

// 枪尖指向偏量：优化后姿态与原始(名义)姿态的工具 Z 轴夹角，即喷嘴偏离表面法向多少度。
// 包络只约束“相对锚点”的偏移，而锚点本身可能离名义法向很远（本工件达 82° 自旋 + 5.4° 仰角），
// 所以指向偏量是与包络无关的独立质量维度，必须单独盯住。
double MaxPointingDeg(const PathItem& raw, const PathItem& optimized) {
  double worst = 0.0;
  const size_t n = std::min(raw.points.size(), optimized.points.size());
  for (size_t i = 0; i < n; ++i) {
    worst = std::max(worst, PointingDeg(raw.points[i].tcp_pose.linear(),
                                        optimized.points[i].tcp_pose.linear()));
  }
  return worst;
}

// 指向偏量护栏：只在显式设了上限时生效（base 为请求档的指向偏量）。
bool PointingOk(const LadderRung& r, double base_pointing, const OptimizeOptions& opt) {
  if (!(opt.tol_ladder_max_pointing_deg > 0.0)) return true;
  if (!std::isfinite(base_pointing)) return true;  // 请求档抛异常时无基准，不拦
  return r.max_pointing_deg <= base_pointing + opt.tol_ladder_max_pointing_deg;
}

double PeakRatio(const PathVerifyReport& rep, const RobotLimits& limits) {
  double r = 0.0;
  for (int j = 0; j < 6; ++j) {
    const double lim = limits.max_vel_deg_s[j];
    if (lim > 0.0) r = std::max(r, rep.peak_joint_speeds_deg_s[j] / lim);
  }
  return r;
}

bool RungBetter(const LadderRung& a, const LadderRung& b, double base_pointing,
                const OptimizeOptions& opt) {
  const int ra = StatusRank(a.status), rb = StatusRank(b.status);
  if (ra != rb) return ra < rb;
  const bool ga = PointingOk(a, base_pointing, opt), gb = PointingOk(b, base_pointing, opt);
  if (ga != gb) return ga;
  if (a.max_pointing_deg != b.max_pointing_deg) return a.max_pointing_deg < b.max_pointing_deg;
  if (a.peak_ratio != b.peak_ratio) return a.peak_ratio < b.peak_ratio;
  return a.objective < b.objective;
}

std::string Num(double v, int prec = 1) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(prec) << v;
  return os.str();
}

std::string FmtTol(const Eigen::Vector3d& t) {
  return "[" + Num(t[0]) + ", " + Num(t[1]) + ", " + Num(t[2]) + "]";
}

}  // namespace

OptimizeResult ViterbiOptimizer::Optimize(const PathItem& path, const Anchor& anchor,
                                          std::optional<JointVec> init_q) const {
  // In per-waypoint mode, retaining every original pose is the best possible
  // normal result.  Test that exact chain before introducing any orientation
  // freedom.  This also avoids changing a path that only needed a different IK
  // branch.  If it is unreachable or fails dense verification, the regular
  // envelope search below is allowed to repair it.
  if (!anchor.has_global() && verifier_ && opt_.dense_verify && path.points.size() >= 2) {
    OptimizeOptions exact_opt = opt_;
    exact_opt.grid_x = {0.0, 0.0, 1.0};
    exact_opt.grid_y = {0.0, 0.0, 1.0};
    exact_opt.grid_z = {0.0, 0.0, 1.0};
    exact_opt.tol_ladder = false;
    try {
      ViterbiOptimizer exact(kin_, tool_, exact_opt, verifier_);
      OptimizeResult retained = exact.OptimizeOnce(path, anchor, init_q, " [原姿态预检]", false);
      if (retained.verify.status == "PASS") {
        std::cerr << "✅ [SprayOpt] 原始姿态已形成安全连续逆解链；保留全部原始法向。\n"
                  << std::flush;
        return retained;
      }
    } catch (const std::exception&) {
      // A strict nominal chain is only a preflight.  The requested envelope
      // remains available for recovery below, where the actionable error is
      // reported if all candidates fail.
    }
  }

  const std::vector<Eigen::Vector3d> rungs = BuildToleranceLadder(anchor.tol_deg, opt_);
  if (rungs.size() <= 1) {
    // 单档（阶梯关闭或比例列表为空）：行为与日志跟以前完全一致。
    return OptimizeOnce(path, anchor, init_q, "", true);
  }

  const bool has_verify = verifier_ && opt_.dense_verify && path.points.size() >= 2;
  const auto t_all0 = std::chrono::steady_clock::now();
  std::cerr << "\U0001FA9C [SprayOpt] 容差阶梯择优: 请求包络 " << FmtTol(anchor.tol_deg) << " → "
            << rungs.size() << " 档，标尺 = 校验状态 → 峰值/限速 → DP 代价 J"
            << (has_verify ? "" : "（未开启密集校验，退化为仅比 J）") << "\n"
            << std::flush;

  std::vector<LadderRung> ladder;
  OptimizeResult best;
  LadderRung best_rung;
  bool have_best = false;
  double base_pointing = std::numeric_limits<double>::quiet_NaN();  // 请求档的指向偏量（护栏基准）
  std::string first_error;  // 请求档抛出的异常：全部档位均失败时原样抛出，保持旧语义

  for (size_t k = 0; k < rungs.size(); ++k) {
    // 请求档必跑；更紧的档如果根本不会钳位任何候选，结果与上一档逐位相同，直接跳过。
    if (k > 0 && EnvelopeInert(path, anchor, rungs[k], opt_)) {
      std::cerr << "   [" << (k + 1) << "/" << rungs.size() << "] tol=" << FmtTol(rungs[k])
                << " 包络宽于搜索网格，不会钳位任何候选（与更宽档等价），已跳过\n"
                << std::flush;
      continue;
    }
    LadderRung rung;
    rung.tol_deg = rungs[k];
    const std::string head = "   [" + std::to_string(k + 1) + "/" + std::to_string(rungs.size()) +
                             "] tol=" + FmtTol(rungs[k]);
    const std::string tag = " [档 " + std::to_string(k + 1) + "/" + std::to_string(rungs.size()) +
                            " tol=" + FmtTol(rungs[k]) + "]";
    Anchor rung_anchor = anchor;
    rung_anchor.tol_deg = rungs[k];
    try {
      // 逐段明细只给第一档打（否则 N 档 × 每段一行会把日志淹没）。
      OptimizeResult r = OptimizeOnce(path, rung_anchor, init_q, tag, k == 0);
      rung.objective = r.objective;
      rung.elapsed_ms = r.elapsed_ms;
      rung.max_pointing_deg = MaxPointingDeg(path, r.path);
      if (k == 0) base_pointing = rung.max_pointing_deg;
      if (has_verify) {
        rung.status = r.verify.status;
        rung.peak_deg_s = PeakMax(r.verify);
        rung.peak_ratio = PeakRatio(r.verify, kin_.limits());
      } else {
        rung.status = "UNVERIFIED";  // verify 是默认构造的，不能当 PASS 参与排序
      }
      ladder.push_back(rung);
      std::cerr << head << " status=" << rung.status << " peak=" << Num(rung.peak_deg_s)
                << "°/s (" << Num(rung.peak_ratio * 100.0) << "% 限速) 指向偏量="
                << Num(rung.max_pointing_deg, 2) << "° J=" << Num(rung.objective) << " elapsed="
                << Num(rung.elapsed_ms, 2) << " ms"
                << (PointingOk(rung, base_pointing, opt_) ? "" : " [超指向护栏]") << "\n"
                << std::flush;
      if (!have_best || RungBetter(rung, best_rung, base_pointing, opt_)) {
        best = std::move(r);
        best_rung = rung;
        have_best = true;
        std::cerr << head << "    ↑ 当前最优\n" << std::flush;
      }
      // A lower-speed rung is never allowed to stop the search when it has
      // worse pointing.  Early exit is safe only once the original normal is
      // retained exactly: no later rung can improve on zero angular error.
      if (has_verify && have_best && best_rung.status == "PASS" &&
          best_rung.max_pointing_deg <= 1e-6 && opt_.tol_ladder_stop_peak_ratio > 0.0 &&
          best_rung.peak_ratio <= opt_.tol_ladder_stop_peak_ratio) {
        std::cerr << "   ✅ 早停: 已保持原始法向且峰值为限速的 "
                  << Num(best_rung.peak_ratio * 100.0) << "%\n"
                  << std::flush;
        break;
      }
    } catch (const std::exception& e) {
      rung.status = "ERROR";
      rung.error = e.what();
      ladder.push_back(rung);
      std::cerr << "⚠️ [SprayOpt]" << head << " 该档无可行解，已跳过: " << e.what() << "\n"
                << std::flush;
      if (k == 0) first_error = e.what();
    }
  }

  if (!have_best) {
    throw std::runtime_error(first_error.empty()
                                 ? "All tolerance-ladder rungs failed to produce a solution."
                                 : first_error);
  }

  // 只跑了一档（其余被惰性跳过）时不回填 ladder，报表/JSON 与旧版单档输出保持一致。
  const bool multi_rung = ladder.size() > 1;
  // 阶梯对外报告的是用户真正等待的总时长（而不是胜出档的单次耗时）。
  best.elapsed_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_all0).count();
  size_t adopted_idx = 0;
  for (size_t i = 0; i < ladder.size(); ++i) {
    if ((ladder[i].tol_deg - best.adopted_tol_deg).cwiseAbs().maxCoeff() < 1e-9) {
      adopted_idx = i;
      break;
    }
  }
  std::cerr << "✅ [SprayOpt] 采纳包络 " << FmtTol(best.adopted_tol_deg)
            << (multi_rung ? "（阶梯第 " + std::to_string(adopted_idx + 1) + "/" +
                                 std::to_string(ladder.size()) + " 档）"
                           : "（请求档；其余档与之等价或无可行解）")
            << "，峰值 " << Num(best_rung.peak_deg_s) << "°/s (" << Num(best_rung.peak_ratio * 100.0)
            << "% 限速)，status=" << best_rung.status << "，阶梯总耗时 " << Num(best.elapsed_ms, 2)
            << " ms\n"
            << std::flush;
  // 阶梯真正起了作用时，把“相对请求档改善了多少”说清楚（请求档抛异常时无可比基准）。
  const LadderRung& req = ladder.front();
  if (adopted_idx > 0 && req.status != "ERROR" && req.peak_deg_s > 0.0) {
    std::cerr << "   ↳ 请求档 " << FmtTol(req.tol_deg) << " 峰值 " << Num(req.peak_deg_s)
              << "°/s → 采纳档降低 " << Num((1.0 - best_rung.peak_deg_s / req.peak_deg_s) * 100.0)
              << "%\n"
              << std::flush;
  }
  // 指向偏量是收紧包络的代价（喷嘴偏离表面法向），必须显式提醒，不能默默牺牲涂层质量。
  if (adopted_idx > 0 && std::isfinite(base_pointing) &&
      best_rung.max_pointing_deg > base_pointing + 1.0) {
    std::cerr << "⚠️ [SprayOpt] 采纳档的最大指向偏量 " << Num(best_rung.max_pointing_deg, 2)
              << "° 大于请求档的 " << Num(base_pointing, 2) << "°（用指向精度换峰值速度）；"
              << "若涂层质量优先，可用 --tol-ladder-max-pointing-deg 限制或 --no-tol-ladder 关闭阶梯\n"
              << std::flush;
  }
  if (multi_rung) best.ladder = std::move(ladder);
  return best;
}

Anchor ResolveAnchor(const AnchorSpec& spec, const Cr5Kinematics& kin, const PathItem& path) {
  Anchor a;
  a.tol_deg = spec.tol_deg;
  const std::string src = spec.source;
  if (src == "raw") {
    a.R = std::nullopt;
    return a;
  }
  if (src == "home") {
    Eigen::Vector3d xyz, rpy;
    kin.FkController(spec.home_joints_rad, xyz, rpy);
    a.R = RotFromCtrlRpyDeg(rpy);
    return a;
  }
  // config / live：RPY 已由调用方解析好
  a.R = RotFromCtrlRpyDeg(spec.ref_rpy_deg);
  (void)path;
  return a;
}

}  // namespace motion

#pragma once

#include "motion/kinematics.hpp"
#include "motion/report.hpp"
#include "motion/segment.hpp"
#include "motion/types.hpp"
#include "motion/verifier.hpp"

namespace motion {

class ViterbiOptimizer {
 public:
  ViterbiOptimizer(const Cr5Kinematics& kin, const ToolOffset& tool,
                   OptimizeOptions opt, const ChainVerifier* verifier = nullptr);

  // 对一条路径做容差阶梯择优：请求包络 + 若干收紧档各跑一次 OptimizeOnce，
  // 按「校验状态 → 指向护栏 → 指向偏量 → 峰值/限速比 → DP 代价 J」字典序取最优（标尺与容差无关）。
  // 关闭阶梯（opt.tol_ladder=false）时等价于只跑请求包络一次。
  OptimizeResult Optimize(const PathItem& path, const Anchor& anchor,
                          std::optional<JointVec> init_q = std::nullopt) const;

 private:
  // 单档包络下的完整 Viterbi DP + 可选密集校验。
  // log_tag 为日志前缀（单档时为空，保持旧日志格式不变）；
  // log_segments=false 时不打印逐段明细（阶梯模式下只保留第一档的）。
  OptimizeResult OptimizeOnce(const PathItem& path, const Anchor& anchor,
                              std::optional<JointVec> init_q, const std::string& log_tag,
                              bool log_segments) const;

  const Cr5Kinematics& kin_;
  const ToolOffset& tool_;
  OptimizeOptions opt_;
  const ChainVerifier* verifier_;
};

Anchor ResolveAnchor(const AnchorSpec& spec, const Cr5Kinematics& kin,
                     const PathItem& path);

}  // namespace motion

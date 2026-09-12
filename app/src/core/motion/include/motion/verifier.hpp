#pragma once

#include "motion/segment.hpp"
#include "motion/kinematics.hpp"
#include "motion/report.hpp"
#include "motion/types.hpp"

#include <optional>

namespace motion {

class ChainVerifier {
 public:
  ChainVerifier(const Cr5Kinematics& kin, const ToolOffset& tool, VerifyOptions opt);
  const VerifyOptions& options() const { return opt_; }

  PathVerifyReport Verify(const PathItem& path, std::optional<JointVec> init_q) const;
  // init_q 只作用于第一条路径，后续路径以上一条的末关节续接（与 Python 侧链式验证一致）。
  VerifyReport VerifyAll(const std::vector<PathItem>& paths,
                         std::optional<JointVec> init_q = std::nullopt) const;

 private:
  Transform ToUrdfFlange(const Transform& T_gun) const;
  // 返回奇异标志，避免调用方重复做一次 ToUrdfFlange + CheckSingularity。
  SingularityFlags Diagnose(const JointVec& q, const Transform& T_gun, int step_idx, int seg_idx,
                            std::vector<Issue>& issues, bool prev[3], bool emit_always) const;

  const Cr5Kinematics& kin_;
  const ToolOffset& tool_;
  VerifyOptions opt_;
  Interpolator interp_;
};

}  // namespace motion

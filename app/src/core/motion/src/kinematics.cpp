#include "motion/kinematics.hpp"

#include <cmath>
#include <cstring>

namespace motion {
namespace {

// DH 闭式解：表达式从 cr5_ur_kin.cpp 原样搬入，不改运算顺序。
constexpr double ZERO_THRESH = 0.00000001;
inline int SIGN(double x) { return (x > 0) - (x < 0); }
constexpr double PI = M_PI;
constexpr double d1 = DhParams::d1;
constexpr double a2 = DhParams::a2;
constexpr double a3 = DhParams::a3;
constexpr double d4 = DhParams::d4;
constexpr double d5 = DhParams::d5;
constexpr double d6 = DhParams::d6;

void DhForwardImpl(const double* q, double* T) {
  double s1 = sin(*q), c1 = cos(*q); q++;
  double q23 = *q, q234 = *q, s2 = sin(*q), c2 = cos(*q); q++;
  double s3 = sin(*q), c3 = cos(*q); q23 += *q; q234 += *q; q++;
  double s4 = sin(*q), c4 = cos(*q); q234 += *q; q++;
  double s5 = sin(*q), c5 = cos(*q); q++;
  double s6 = sin(*q), c6 = cos(*q);
  double s23 = sin(q23), c23 = cos(q23);
  double s234 = sin(q234), c234 = cos(q234);
  *T = c234*c1*s5 - c5*s1; T++;
  *T = c6*(s1*s5 + c234*c1*c5) - s234*c1*s6; T++;
  *T = -s6*(s1*s5 + c234*c1*c5) - s234*c1*c6; T++;
  *T = d6*c234*c1*s5 - a3*c23*c1 - a2*c1*c2 - d6*c5*s1 - d5*s234*c1 - d4*s1; T++;
  *T = c1*c5 + c234*s1*s5; T++;
  *T = -c6*(c1*s5 - c234*c5*s1) - s234*s1*s6; T++;
  *T = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1; T++;
  *T = d6*(c1*c5 + c234*s1*s5) + d4*c1 - a3*c23*s1 - a2*c2*s1 - d5*s234*s1; T++;
  *T = -s234*s5; T++;
  *T = -c234*s6 - s234*c5*c6; T++;
  *T = s234*c5*s6 - c234*c6; T++;
  *T = d1 + a3*s23 + a2*s2 - d5*(c23*c4 - s23*s4) - d6*s5*(c23*s4 + s23*c4); T++;
  *T = 0.0; T++; *T = 0.0; T++; *T = 0.0; T++; *T = 1.0;
  (void)s3; (void)c3; (void)s4; (void)c4;
}

int DhInverseImpl(const double* T, double* q_sols, double q6_des) {
  int num_sols = 0;
  double T02 = -*T; T++; double T00 =  *T; T++; double T01 =  *T; T++; double T03 = -*T; T++;
  double T12 = -*T; T++; double T10 =  *T; T++; double T11 =  *T; T++; double T13 = -*T; T++;
  double T22 =  *T; T++; double T20 = -*T; T++; double T21 = -*T; T++; double T23 =  *T;

  double q1[2];
  {
    double A = d6*T12 - T13;
    double B = d6*T02 - T03;
    double R = A*A + B*B;
    if(fabs(A) < ZERO_THRESH) {
      double div;
      if(fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
        div = -SIGN(d4)*SIGN(B);
      else
        div = -d4/B;
      double arcsin = asin(div);
      if(fabs(arcsin) < ZERO_THRESH)
        arcsin = 0.0;
      if(arcsin < 0.0)
        q1[0] = arcsin + 2.0*PI;
      else
        q1[0] = arcsin;
      q1[1] = PI - arcsin;
    }
    else if(fabs(B) < ZERO_THRESH) {
      double div;
      if(fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
        div = SIGN(d4)*SIGN(A);
      else
        div = d4/A;
      double arccos = acos(div);
      q1[0] = arccos;
      q1[1] = 2.0*PI - arccos;
    }
    else if(d4*d4 > R) {
      return num_sols;
    }
    else {
      double arccos = acos(d4 / sqrt(R)) ;
      double arctan = atan2(-B, A);
      double pos = arccos + arctan;
      double neg = -arccos + arctan;
      if(fabs(pos) < ZERO_THRESH)
        pos = 0.0;
      if(fabs(neg) < ZERO_THRESH)
        neg = 0.0;
      if(pos >= 0.0)
        q1[0] = pos;
      else
        q1[0] = 2.0*PI + pos;
      if(neg >= 0.0)
        q1[1] = neg;
      else
        q1[1] = 2.0*PI + neg;
    }
  }

  double q5[2][2];
  {
    for(int i=0;i<2;i++) {
      double numer = (T03*sin(q1[i]) - T13*cos(q1[i])-d4);
      double div;
      if(fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
        div = SIGN(numer) * SIGN(d6);
      else
        div = numer / d6;
      double arccos = acos(div);
      q5[i][0] = arccos;
      q5[i][1] = 2.0*PI - arccos;
    }
  }

  {
    for(int i=0;i<2;i++) {
      for(int j=0;j<2;j++) {
        double c1 = cos(q1[i]), s1 = sin(q1[i]);
        double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
        double q6;
        if(fabs(s5) < ZERO_THRESH)
          q6 = q6_des;
        else {
          q6 = atan2(SIGN(s5)*-(T01*s1 - T11*c1),
                     SIGN(s5)*(T00*s1 - T10*c1));
          if(fabs(q6) < ZERO_THRESH)
            q6 = 0.0;
          if(q6 < 0.0)
            q6 += 2.0*PI;
        }

        double q2[2], q3[2], q4[2];
        double c6 = cos(q6), s6 = sin(q6);
        double x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1));
        double x04y = c5*(T20*c6 - T21*s6) - T22*s5;
        double p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) +
                      T03*c1 + T13*s1;
        double p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6);

        double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
        if(fabs(fabs(c3) - 1.0) < ZERO_THRESH)
          c3 = SIGN(c3);
        else if(fabs(c3) > 1.0) {
          continue;
        }
        double arccos = acos(c3);
        q3[0] = arccos;
        q3[1] = 2.0*PI - arccos;
        double denom = a2*a2 + a3*a3 + 2*a2*a3*c3;
        double s3 = sin(arccos);
        double A = (a2 + a3*c3), B = a3*s3;
        q2[0] = atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
        q2[1] = atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);
        double c23_0 = cos(q2[0]+q3[0]);
        double s23_0 = sin(q2[0]+q3[0]);
        double c23_1 = cos(q2[1]+q3[1]);
        double s23_1 = sin(q2[1]+q3[1]);
        q4[0] = atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
        q4[1] = atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);
        for(int k=0;k<2;k++) {
          if(fabs(q2[k]) < ZERO_THRESH)
            q2[k] = 0.0;
          else if(q2[k] < 0.0) q2[k] += 2.0*PI;
          if(fabs(q4[k]) < ZERO_THRESH)
            q4[k] = 0.0;
          else if(q4[k] < 0.0) q4[k] += 2.0*PI;
          q_sols[num_sols*6+0] = q1[i];    q_sols[num_sols*6+1] = q2[k];
          q_sols[num_sols*6+2] = q3[k];    q_sols[num_sols*6+3] = q4[k];
          q_sols[num_sols*6+4] = q5[i][j]; q_sols[num_sols*6+5] = q6;
          num_sols++;
        }
      }
    }
  }
  return num_sols;
}

void UrdfForward(const double* q_urdf, double* T) {
  double q_dh[6];
  q_dh[0] = q_urdf[0];
  q_dh[1] = q_urdf[1] - M_PI / 2.0;
  q_dh[2] = q_urdf[2];
  q_dh[3] = q_urdf[3] - M_PI / 2.0;
  q_dh[4] = q_urdf[4];
  q_dh[5] = q_urdf[5];
  DhForwardImpl(q_dh, T);
}

int UrdfInverse(const double* T, double* q_sols) {
  double q_dh_sols[8 * 6];
  int num_sols = DhInverseImpl(T, q_dh_sols, 0.0);
  for (int i = 0; i < num_sols; i++) {
    q_sols[i * 6 + 0] = q_dh_sols[i * 6 + 0];
    q_sols[i * 6 + 1] = q_dh_sols[i * 6 + 1] + M_PI / 2.0;
    q_sols[i * 6 + 2] = q_dh_sols[i * 6 + 2];
    q_sols[i * 6 + 3] = q_dh_sols[i * 6 + 3] + M_PI / 2.0;
    q_sols[i * 6 + 4] = q_dh_sols[i * 6 + 4];
    q_sols[i * 6 + 5] = q_dh_sols[i * 6 + 5];
    for (int j = 0; j < 6; j++) {
      while (q_sols[i * 6 + j] > M_PI) q_sols[i * 6 + j] -= 2 * M_PI;
      while (q_sols[i * 6 + j] < -M_PI) q_sols[i * 6 + j] += 2 * M_PI;
    }
  }
  return num_sols;
}

void ControllerPoseToUrdf(const double* xyz_mm, const double* rpy_deg, double* T_urdf) {
  const double rx = rpy_deg[0] * M_PI / 180.0;
  const double ry = rpy_deg[1] * M_PI / 180.0;
  const double rz = rpy_deg[2] * M_PI / 180.0;
  const double sx = sin(rx), cx = cos(rx);
  const double sy = sin(ry), cy = cos(ry);
  const double sz = sin(rz), cz = cos(rz);

  const double r00 = cz * cy;
  const double r01 = cz * sy * sx - sz * cx;
  const double r02 = cz * sy * cx + sz * sx;
  const double r10 = sz * cy;
  const double r11 = sz * sy * sx + cz * cx;
  const double r12 = sz * sy * cx - cz * sx;
  const double r20 = -sy;
  const double r21 = cy * sx;
  const double r22 = cy * cx;

  const double x = xyz_mm[0] / 1000.0;
  const double y = xyz_mm[1] / 1000.0;
  const double z = xyz_mm[2] / 1000.0;

  T_urdf[0] = -r02; T_urdf[1] =  r00; T_urdf[2] =  r01; T_urdf[3] = -x;
  T_urdf[4] = -r12; T_urdf[5] =  r10; T_urdf[6] =  r11; T_urdf[7] = -y;
  T_urdf[8] =  r22; T_urdf[9] = -r20; T_urdf[10] = -r21; T_urdf[11] = z;
  T_urdf[12] = 0.0; T_urdf[13] = 0.0; T_urdf[14] = 0.0; T_urdf[15] = 1.0;
}

}  // namespace

void Cr5Kinematics::DhFk(const double* q_dh, double* T16) { DhForwardImpl(q_dh, T16); }
int Cr5Kinematics::DhIk(const double* T16, double* q_sols, double q6_des) {
  return DhInverseImpl(T16, q_sols, q6_des);
}
void Cr5Kinematics::FkRaw(const double* q_urdf, double* T16) { UrdfForward(q_urdf, T16); }
int Cr5Kinematics::IkRaw(const double* T_urdf, double* q_sols) { return UrdfInverse(T_urdf, q_sols); }

Cr5Kinematics::Cr5Kinematics(RobotLimits limits) : limits_(std::move(limits)) {}

Transform Cr5Kinematics::Fk(const JointVec& q_urdf) const {
  double T[16];
  UrdfForward(q_urdf.data(), T);
  return TransformFromRowMajor(T);
}

int Cr5Kinematics::Ik(const Transform& T_urdf, JointVec* out_sols) const {
  double T[16];
  double sols[48];
  TransformToRowMajor(T_urdf, T);
  const int n = UrdfInverse(T, sols);
  for (int i = 0; i < n; ++i) out_sols[i] = Eigen::Map<const JointVec>(sols + i * 6);
  return n;
}

std::optional<JointVec> Cr5Kinematics::NearestInLimits(
    const JointVec& q, const JointVec& reference) const {
  if (!q.allFinite() || !reference.allFinite()) return std::nullopt;
  JointVec result;
  for (int j = 0; j < 6; ++j) {
    const double lo = std::ceil((limits_.min_rad[j] - kJointTol - q[j]) / (2.0 * kPi));
    const double hi = std::floor((limits_.max_rad[j] + kJointTol - q[j]) / (2.0 * kPi));
    if (lo > hi) return std::nullopt;
    const double turns = std::clamp(std::round((reference[j] - q[j]) / (2.0 * kPi)), lo, hi);
    result[j] = q[j] + turns * (2.0 * kPi);
  }
  return result;
}

int Cr5Kinematics::BestIkRaw(const double* T_urdf, const double* q_seed, const double* weights,
                             double* q_out) const {
  double sols[48];
  const int n = UrdfInverse(T_urdf, sols);
  double best_dist = 1e300;
  bool found = false;
  const JointVec seed = Eigen::Map<const JointVec>(q_seed);
  for (int i = 0; i < n; ++i) {
    const auto candidate = NearestInLimits(Eigen::Map<const JointVec>(sols + i * 6), seed);
    if (!candidate) continue;
    double dist = 0.0;
    for (int j = 0; j < 6; ++j) {
      // Never wrap again after choosing a physically valid angle: a bounded
      // J4 move from -179 to +179 degrees is 358 degrees, not -2 degrees.
      const double delta = (*candidate)[j] - seed[j];
      dist += (weights ? weights[j] : 1.0) * delta * delta;
    }
    if (dist < best_dist) {
      best_dist = dist;
      std::memcpy(q_out, candidate->data(), 6 * sizeof(double));
      found = true;
    }
  }
  return found ? 1 : 0;
}

std::optional<JointVec> Cr5Kinematics::BestIk(const Transform& T_urdf, const JointVec& q_seed,
                                              const JointVec* weights) const {
  double T[16];
  double q_out[6];
  TransformToRowMajor(T_urdf, T);
  const double* w = weights ? weights->data() : nullptr;
  if (!BestIkRaw(T, q_seed.data(), w, q_out)) return std::nullopt;
  return Eigen::Map<const JointVec>(q_out);
}

void Cr5Kinematics::FkController(const JointVec& q_urdf, Eigen::Vector3d& xyz_mm,
                                 Eigen::Vector3d& rpy_deg) const {
  double T_urdf[16];
  UrdfForward(q_urdf.data(), T_urdf);

  double T_bt[16];
  for (int k = 0; k < 16; k++) T_bt[k] = T_urdf[k];
  for (int k = 0; k < 4; k++) {
    T_bt[0 * 4 + k] = -T_urdf[0 * 4 + k];
    T_bt[1 * 4 + k] = -T_urdf[1 * 4 + k];
  }

  double T_ctrl[16];
  for (int k = 0; k < 3; k++) {
    T_ctrl[k * 4 + 0] = -T_bt[k * 4 + 1];
    T_ctrl[k * 4 + 1] = -T_bt[k * 4 + 2];
    T_ctrl[k * 4 + 2] = T_bt[k * 4 + 0];
    T_ctrl[k * 4 + 3] = T_bt[k * 4 + 3];
  }

  xyz_mm[0] = T_ctrl[3] * 1000.0;
  xyz_mm[1] = T_ctrl[7] * 1000.0;
  xyz_mm[2] = T_ctrl[11] * 1000.0;

  const double sol_rz = atan2(T_ctrl[1 * 4 + 0], T_ctrl[0 * 4 + 0]);
  const double sol_ry =
      atan2(-T_ctrl[2 * 4 + 0],
            sqrt(T_ctrl[2 * 4 + 1] * T_ctrl[2 * 4 + 1] + T_ctrl[2 * 4 + 2] * T_ctrl[2 * 4 + 2]));
  const double sol_rx = atan2(T_ctrl[2 * 4 + 1], T_ctrl[2 * 4 + 2]);
  rpy_deg[0] = sol_rx * 180.0 / M_PI;
  rpy_deg[1] = sol_ry * 180.0 / M_PI;
  rpy_deg[2] = sol_rz * 180.0 / M_PI;
}

int Cr5Kinematics::IkController(const Eigen::Vector3d& xyz_mm, const Eigen::Vector3d& rpy_deg,
                                JointVec* out_sols) const {
  double T_urdf[16];
  ControllerPoseToUrdf(xyz_mm.data(), rpy_deg.data(), T_urdf);
  double sols[48];
  const int n = UrdfInverse(T_urdf, sols);
  for (int i = 0; i < n; ++i) out_sols[i] = Eigen::Map<const JointVec>(sols + i * 6);
  return n;
}

bool Cr5Kinematics::IsJointValid(const JointVec& q) const {
  for (int i = 0; i < 6; ++i) {
    if (q[i] < limits_.min_rad[i] - kJointTol || q[i] > limits_.max_rad[i] + kJointTol) {
      return false;
    }
  }
  return true;
}

SingularityFlags Cr5Kinematics::CheckSingularity(const JointVec& q,
                                                 const Transform& T_urdf) const {
  SingularityFlags f;
  f.wrist = std::abs(std::sin(q[4])) < kSingSin;
  f.elbow = std::abs(std::sin(q[2])) < kSingSin;
  const double half = ShoulderHalfRad(T_urdf);
  f.shoulder = half < kShoulderHalfRad;
  f.wrist_angle_deg = Deg(q[4]);
  f.elbow_angle_deg = Deg(q[2]);
  f.shoulder_q1_separation_deg = Deg(2.0 * half);
  return f;
}

int Cr5Kinematics::IkBatch(const Transform* T, int n, JointVec* out, int* n_sols) const {
  int total = 0;
  for (int i = 0; i < n; ++i) {
    JointVec sols[8];
    const int ns = Ik(T[i], sols);
    n_sols[i] = ns;
    total += ns;
    for (int k = 0; k < ns; ++k) out[i * 8 + k] = sols[k];
  }
  return total;
}

}  // namespace motion

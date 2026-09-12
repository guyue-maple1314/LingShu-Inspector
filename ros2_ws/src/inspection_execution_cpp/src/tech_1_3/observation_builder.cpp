#include "inspection_execution_cpp/tech_1_3/observation_builder.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace inspection_execution {
namespace tech_1_3 {

namespace {

// 四元数 → roll/pitch/yaw（简化：小角度场景足够；不处理万向节奇异性）
std::array<double, 3> QuatToRpy(double qx, double qy, double qz, double qw) {
  const double n2 = qw * qw + qx * qx + qy * qy + qz * qz;
  if (n2 <= 1e-12) return {0.0, 0.0, 0.0};
  const double s = 2.0 / n2;
  const double xs = qx * s, ys = qy * s, zs = qz * s;
  const double wx = qw * xs, wy = qw * ys, wz = qw * zs;
  const double xx = qx * xs, xy = qx * ys, xz = qx * zs;
  const double yy = qy * ys, yz = qy * zs, zz = qz * zs;

  const double m00 = 1.0 - (yy + zz);
  const double m10 = xy - wz;
  const double m20 = xz + wy;
  const double m21 = yz - wx;
  const double m22 = 1.0 - (xx + yy);

  const double pitch = std::atan2(-m20, std::sqrt(m21 * m21 + m22 * m22));
  const double roll = std::atan2(m21, m22);
  const double yaw = std::atan2(m10, m00);
  return {roll, pitch, yaw};
}

}  // namespace

ObservationBuilder::ObservationBuilder() : expected_dim_(kDefaultObservationDim) {}

void ObservationBuilder::SetExpectedDimension(std::uint32_t dim) {
  expected_dim_ = dim == 0 ? kDefaultObservationDim : dim;
}

bool ObservationBuilder::Build(const ObservationInput& in, std::vector<float>* out) const {
  if (!out) return false;
  out->assign(expected_dim_, 0.0f);
  if (expected_dim_ < kDefaultObservationDim) return false;

  const auto rpy = QuatToRpy(in.imu.qx, in.imu.qy, in.imu.qz, in.imu.qw);

  // [0:3] rpy
  (*out)[0] = static_cast<float>(rpy[0]);
  (*out)[1] = static_cast<float>(rpy[1]);
  (*out)[2] = static_cast<float>(rpy[2]);
  // [3:6] angular vel
  (*out)[3] = static_cast<float>(in.imu.gx);
  (*out)[4] = static_cast<float>(in.imu.gy);
  (*out)[5] = static_cast<float>(in.imu.gz);
  // [6:9] linear vel（若未知则 0）
  (*out)[6] = static_cast<float>(in.body_linear_vel[0]);
  (*out)[7] = static_cast<float>(in.body_linear_vel[1]);
  (*out)[8] = static_cast<float>(in.body_linear_vel[2]);
  // [9:13] contact flags
  for (std::size_t i = 0; i < kFootCount; ++i) {
    (*out)[9 + i] = in.foot_force.contact_flags[i] > 0.0 ? 1.0f : 0.0f;
  }
  // [13:17] foot force normalized by 50N threshold（PPT 1.6：50N 足端力约束）
  constexpr double kForceNorm = 50.0;
  for (std::size_t i = 0; i < kFootCount; ++i) {
    const double v = in.foot_force.normal_forces[i] / kForceNorm;
    (*out)[13 + i] = static_cast<float>(std::min(5.0, std::max(0.0, v)));
  }

  last_terrain_hint_ = in.terrain_type;
  return true;
}

}  // namespace tech_1_3
}  // namespace inspection_execution

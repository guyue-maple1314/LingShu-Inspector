#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "inspection_execution_cpp/adapters/foot_force_adapter.hpp"
#include "inspection_execution_cpp/adapters/imu_adapter.hpp"

namespace inspection_execution {
namespace tech_1_3 {

// 观测维度来源（PPT 1.3：姿态 / 接触 / 速度）
// 布局（默认值，便于 unittest；实际维度以 policy_metadata 为准）：
//   [0:3]  body_rpy      姿态（来自 IMU 四元数欧拉化）
//   [3:6]  body_angular_vel   角速度
//   [6:9]  body_linear_vel    线速度（缺省可由本体状态/里程计注入，此处留槽）
//   [9:13] foot_contact_4     四足足端接触（bool 转 float）
//   [13:17] foot_force_norm_4 四足足端力归一化
//  obs_dim 默认 = 17
inline constexpr std::uint32_t kDefaultObservationDim = 17;
inline constexpr std::size_t kFootCount = 4;

struct ObservationInput {
  ImuSample imu;                                     // 1000 Hz
  FootForceSample foot_force;                        // 500 Hz
  std::array<double, 3> body_linear_vel{};           // x/y/z m/s（若本体可提供）
  std::string terrain_type;                          // flat/grating/ramp/stairs/narrow_corridor
  std::uint64_t timestamp_ns{0};
};

class ObservationBuilder {
 public:
  ObservationBuilder();

  // 观测维数可由外部通过 policy metadata 设置，不强制默认值；
  // 但 Build 会检查 obs 长度，不够则 resize。
  void SetExpectedDimension(std::uint32_t dim);
  std::uint32_t ExpectedDimension() const { return expected_dim_; }

  // 把输入装配成 observation 向量；返回是否成功（输入失效/越界会返回 false）。
  bool Build(const ObservationInput& in, std::vector<float>* out) const;

  // 便捷接口：从最近 Build 结果派生 terrain_type 标签（只读，不参与训练）。
  const std::string& LastTerrainHint() const { return last_terrain_hint_; }

 private:
  std::uint32_t expected_dim_;
  mutable std::string last_terrain_hint_;
};

}  // namespace tech_1_3
}  // namespace inspection_execution

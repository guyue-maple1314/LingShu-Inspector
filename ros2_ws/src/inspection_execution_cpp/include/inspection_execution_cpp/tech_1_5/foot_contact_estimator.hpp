#pragma once

#include <array>
#include <cstdint>

#include "inspection_execution_cpp/adapters/foot_force_adapter.hpp"

namespace inspection_execution {
namespace tech_1_5 {

// 接触状态分类
enum class ContactQuality : int {
  kNoContact = 0,       // 无接触（足在空中）
  kSolidContact = 1,    // 稳固接触（实心地面）
  kGratingContact = 2,  // 钢格网接触（力稳定但偏低）
  kFalseContact = 3,    // 误判接触（格网孔洞，力突变/不稳定）
};

// 单足接触估计结果
struct FootContactState {
  ContactQuality quality{ContactQuality::kNoContact};
  double confidence{0.0};       // 0..1
  double filtered_force{0.0};    // 滤波后法向力
  double force_variance{0.0};   // 近窗力方差
};

// 四足整体接触估计
struct ContactEstimate {
  std::array<FootContactState, 4> feet{};
  std::uint64_t timestamp_ns{0};
  bool all_feet_valid{false};
};

/// 足端接触估计器：500Hz 足端力 → 接触状态
/// 滑动窗口滤波 + 方差检测，区分实心地面、钢格网、格网孔洞误判
class FootContactEstimator {
 public:
  static constexpr std::size_t kWindow = 8;       // 滑窗大小（500Hz × 8 = 16ms）
  static constexpr double kSolidThreshold = 30.0;  // 实心地面力阈值 (N)
  static constexpr double kGratingMin = 8.0;       // 钢格网最低力阈值 (N)
  static constexpr double kFalseVariance = 400.0;  // 误判方差阈值 (N²)

  FootContactEstimator();

  /// 更新单足估计（500Hz 调用）
  void Update(std::size_t foot_idx, double force_n, std::uint64_t ts_ns);

  /// 获取当前四足接触估计
  ContactEstimate Estimate() const;

  /// 设置钢格网模式（改变阈值）
  void SetGratingMode(bool enabled);

 private:
  struct WindowBuf {
    std::array<double, kWindow> buf{};
    std::size_t head{0};
    bool filled{false};
    double sum{0.0};
    double sum_sq{0.0};
  };
  std::array<WindowBuf, 4> windows_{};
  bool grating_mode_{false};
};

}  // namespace tech_1_5
}  // namespace inspection_execution

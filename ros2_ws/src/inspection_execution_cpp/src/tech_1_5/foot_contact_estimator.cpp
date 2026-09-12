#include "inspection_execution_cpp/tech_1_5/foot_contact_estimator.hpp"

#include <cmath>

namespace inspection_execution {
namespace tech_1_5 {

FootContactEstimator::FootContactEstimator() = default;

void FootContactEstimator::Update(std::size_t foot_idx, double force_n,
                                  std::uint64_t ts_ns) {
  (void)ts_ns;  // 预留时间戳接口，当前滑窗逻辑不需要
  if (foot_idx >= 4) return;
  auto& w = windows_[foot_idx];

  // 减去旧值
  if (w.filled) {
    w.sum -= w.buf[w.head];
    w.sum_sq -= w.buf[w.head] * w.buf[w.head];
  }

  // 写入新值
  w.buf[w.head] = force_n;
  w.sum += force_n;
  w.sum_sq += force_n * force_n;

  // 推进
  w.head = (w.head + 1) % kWindow;
  if (w.head == 0) w.filled = true;
}

ContactEstimate FootContactEstimator::Estimate() const {
  ContactEstimate result;
  result.all_feet_valid = true;

  for (std::size_t i = 0; i < 4; ++i) {
    const auto& w = windows_[i];
    auto& foot = result.feet[i];

    if (!w.filled) {
      result.all_feet_valid = false;
      foot.quality = ContactQuality::kNoContact;
      foot.confidence = 0.0;
      continue;
    }

    const double mean = w.sum / kWindow;
    const double variance = (w.sum_sq / kWindow) - (mean * mean);
    foot.filtered_force = mean;
    foot.force_variance = variance;

    if (mean < kGratingMin) {
      // 力太低，无接触
      foot.quality = ContactQuality::kNoContact;
      foot.confidence = 0.9;
    } else if (variance > kFalseVariance) {
      // 力方差过大 → 格网孔洞误判
      foot.quality = ContactQuality::kFalseContact;
      foot.confidence = std::min(1.0, variance / (kFalseVariance * 2.0));
    } else if (mean < kSolidThreshold || grating_mode_) {
      // 力中等或钢格网模式 → 钢格网接触
      foot.quality = ContactQuality::kGratingContact;
      foot.confidence = 0.8;
    } else {
      // 力稳定且高 → 实心地面
      foot.quality = ContactQuality::kSolidContact;
      foot.confidence = 0.95;
    }
  }

  return result;
}

void FootContactEstimator::SetGratingMode(bool enabled) {
  grating_mode_ = enabled;
}

}  // namespace tech_1_5
}  // namespace inspection_execution

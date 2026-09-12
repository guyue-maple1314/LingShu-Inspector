#include "inspection_execution_cpp/tech_1_5/vibration_estimator.hpp"

#include <cmath>

namespace inspection_execution {
namespace tech_1_5 {

VibrationEstimator::VibrationEstimator() = default;

void VibrationEstimator::UpdateImu(const ImuSample& sample) {
  // 计算加速度幅值（去重力后的绝对值近似）
  const double ax = sample.ax;
  const double ay = sample.ay;
  // 去掉重力分量（az 通常约 -9.8）
  const double az_no_g = sample.az + 9.81;
  const double mag = std::sqrt(ax * ax + ay * ay + az_no_g * az_no_g);

  // 更新 RMS 滑窗
  if (imu_filled_) {
    imu_sum_sq_ -= imu_abs_buf_[imu_head_] * imu_abs_buf_[imu_head_];
  }
  imu_abs_buf_[imu_head_] = mag;
  imu_sum_sq_ += mag * mag;

  // 更新 ax 滑窗（用于频率估计）
  imu_ax_buf_[imu_ax_head_] = ax;

  imu_head_ = (imu_head_ + 1) % kImuWindow;
  imu_ax_head_ = (imu_ax_head_ + 1) % kImuWindow;
  if (imu_head_ == 0) imu_filled_ = true;
  ++imu_count_;

  latest_ts_ = sample.timestamp_ns;
}

void VibrationEstimator::UpdateFootForce(const FootForceSample& sample) {
  for (std::size_t i = 0; i < 4; ++i) {
    auto& w = force_windows_[i];
    if (w.filled) {
      w.sum -= w.buf[w.head];
      w.sum_sq -= w.buf[w.head] * w.buf[w.head];
    }
    w.buf[w.head] = sample.normal_forces[i];
    w.sum += sample.normal_forces[i];
    w.sum_sq += sample.normal_forces[i] * sample.normal_forces[i];
    w.head = (w.head + 1) % kForceWindow;
    if (w.head == 0) w.filled = true;
  }
  latest_ts_ = sample.timestamp_ns;
}

double VibrationEstimator::ComputeRms() const {
  if (!imu_filled_) return 0.0;
  // 正确 RMS = sqrt(mean(mag²))
  return std::sqrt(imu_sum_sq_ / kImuWindow);
}

double VibrationEstimator::EstimateFreq() const {
  if (!imu_filled_) return 0.0;

  // 零交叉频率估计：统计 ax 信号过零次数
  int zero_crossings = 0;
  for (std::size_t i = 0; i < kImuWindow; ++i) {
    const std::size_t curr = (imu_ax_head_ + i) % kImuWindow;
    const std::size_t next = (curr + 1) % kImuWindow;
    if (imu_ax_buf_[curr] * imu_ax_buf_[next] < 0.0) {
      ++zero_crossings;
    }
  }

  // 频率 = 零交叉数 / (2 × 窗时长)
  // 窗时长 = kImuWindow / 1000 Hz = kImuWindow ms
  const double window_sec = static_cast<double>(kImuWindow) / 1000.0;
  const double freq = zero_crossings / (2.0 * window_sec);
  return freq;
}

VibrationSeverity VibrationEstimator::Classify(double rms) {
  if (rms >= kSevereThreshold) return VibrationSeverity::kSevere;
  if (rms >= kModerateThreshold) return VibrationSeverity::kModerate;
  if (rms >= kMildThreshold) return VibrationSeverity::kMild;
  return VibrationSeverity::kNone;
}

VibrationState VibrationEstimator::Estimate() const {
  VibrationState state;
  state.timestamp_ns = latest_ts_;

  state.accel_rms = ComputeRms();
  state.dominant_freq_hz = EstimateFreq();
  state.severity = Classify(state.accel_rms);

  // 钢格网共振特征：频率在 3-12Hz 且振动 >= Moderate
  state.resonance_detected =
      (state.dominant_freq_hz >= kResonanceFreqMin &&
       state.dominant_freq_hz <= kResonanceFreqMax &&
       state.severity >= VibrationSeverity::kModerate);

  // 足力 RMS
  for (std::size_t i = 0; i < 4; ++i) {
    const auto& w = force_windows_[i];
    if (w.filled) {
      state.foot_force_rms[i] = std::sqrt(w.sum_sq / kForceWindow);
    }
  }

  return state;
}

}  // namespace tech_1_5
}  // namespace inspection_execution

#include "inspection_execution_cpp/tech_1_5/grating_metrics_recorder.hpp"

namespace inspection_execution {
namespace tech_1_5 {

GratingMetricsRecorder::GratingMetricsRecorder() = default;

void GratingMetricsRecorder::RecordStep(std::uint64_t ts_ns) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (start_ns_ == 0) {
    start_ns_ = ts_ns;
  }
  ++total_steps_;
  last_ts_ = ts_ns;
}

void GratingMetricsRecorder::RecordAnomaly(const GaitAnomalyEvent& event) {
  std::lock_guard<std::mutex> lock(mtx_);
  ++anomaly_count_;
  if (start_ns_ == 0) {
    start_ns_ = event.timestamp_ns;
  }
}

void GratingMetricsRecorder::UpdateSpeed(double speed_mps,
                                          std::uint64_t ts_ns) {
  std::lock_guard<std::mutex> lock(mtx_);
  speed_sum_ += speed_mps;
  ++speed_count_;

  // 累计距离（梯形积分：用新旧速度均值）
  if (last_ts_ != 0 && ts_ns > last_ts_) {
    const double dt_sec =
        static_cast<double>(ts_ns - last_ts_) / 1e9;
    total_distance_ += (last_speed_ + speed_mps) / 2.0 * dt_sec;
  }
  last_speed_ = speed_mps;
  last_ts_ = ts_ns;
}

void GratingMetricsRecorder::UpdateVibration(double rms) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (rms > max_vibration_rms_) {
    max_vibration_rms_ = rms;
  }
}

void GratingMetricsRecorder::SetGratingMode(bool active) {
  std::lock_guard<std::mutex> lock(mtx_);
  grating_mode_active_ = active;
}

GratingMetrics GratingMetricsRecorder::Snapshot() const {
  std::lock_guard<std::mutex> lock(mtx_);
  GratingMetrics m;
  m.total_steps = total_steps_;
  m.anomaly_count = anomaly_count_;
  m.anomaly_rate =
      total_steps_ > 0 ? static_cast<double>(anomaly_count_) /
                             static_cast<double>(total_steps_)
                       : 0.0;
  m.avg_speed = speed_count_ > 0 ? speed_sum_ / speed_count_ : 0.0;
  m.total_distance = total_distance_;
  m.max_vibration_rms = max_vibration_rms_;
  m.grating_mode_active = grating_mode_active_;
  m.measurement_start_ns = start_ns_;
  m.measurement_end_ns = last_ts_;
  return m;
}

void GratingMetricsRecorder::Reset() {
  std::lock_guard<std::mutex> lock(mtx_);
  total_steps_ = 0;
  anomaly_count_ = 0;
  speed_sum_ = 0.0;
  speed_count_ = 0;
  total_distance_ = 0.0;
  max_vibration_rms_ = 0.0;
  grating_mode_active_ = false;
  start_ns_ = 0;
  last_ts_ = 0;
  last_speed_ = 0.0;
}

bool GratingMetricsRecorder::ThresholdsPass() const {
  std::lock_guard<std::mutex> lock(mtx_);
  if (total_steps_ == 0) return false;
  const double anomaly_rate =
      static_cast<double>(anomaly_count_) / static_cast<double>(total_steps_);
  const double avg_speed = speed_count_ > 0 ? speed_sum_ / speed_count_ : 0.0;
  return anomaly_rate <= kAnomalyRateTarget && avg_speed >= kMinAvgSpeed &&
         total_distance_ >= kMinDistance;
}

}  // namespace tech_1_5
}  // namespace inspection_execution

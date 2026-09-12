#include "inspection_execution_cpp/tech_1_1/abnormal_switch_monitor.hpp"

#include <algorithm>

namespace inspection_execution {
namespace tech_1_1 {

void AbnormalSwitchMonitor::MarkArrival(Clock::time_point t) {
  arrival_ = t;
}

void AbnormalSwitchMonitor::MarkSwitchComplete(Clock::time_point t) {
  if (!arrival_) {
    return;
  }
  const double ms = std::chrono::duration<double, std::milli>(t - *arrival_).count();
  latencies_ms_.push_back(ms);
  arrival_.reset();
}

double AbnormalSwitchMonitor::LastLatencyMs() const {
  return latencies_ms_.empty() ? 0.0 : latencies_ms_.back();
}

double AbnormalSwitchMonitor::MaxLatencyMs() const {
  if (latencies_ms_.empty()) {
    return 0.0;
  }
  return *std::max_element(latencies_ms_.begin(), latencies_ms_.end());
}

std::size_t AbnormalSwitchMonitor::Count() const {
  return latencies_ms_.size();
}

}  // namespace tech_1_1
}  // namespace inspection_execution

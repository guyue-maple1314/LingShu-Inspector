#include "inspection_execution_cpp/tech_1_2/preemption_latency_monitor.hpp"

#include <algorithm>

namespace inspection_execution {
namespace tech_1_2 {

void PreemptionLatencyMonitor::MarkArrival(Clock::time_point t) {
  arrival_ = t;
}

void PreemptionLatencyMonitor::MarkSwitchComplete(Clock::time_point t) {
  if (!arrival_) {
    return;
  }
  const double ms = std::chrono::duration<double, std::milli>(t - *arrival_).count();
  latencies_ms_.push_back(ms);
  arrival_.reset();
}

double PreemptionLatencyMonitor::LastLatencyMs() const {
  return latencies_ms_.empty() ? 0.0 : latencies_ms_.back();
}

double PreemptionLatencyMonitor::MaxLatencyMs() const {
  if (latencies_ms_.empty()) {
    return 0.0;
  }
  return *std::max_element(latencies_ms_.begin(), latencies_ms_.end());
}

std::size_t PreemptionLatencyMonitor::Count() const {
  return latencies_ms_.size();
}

}  // namespace tech_1_2
}  // namespace inspection_execution

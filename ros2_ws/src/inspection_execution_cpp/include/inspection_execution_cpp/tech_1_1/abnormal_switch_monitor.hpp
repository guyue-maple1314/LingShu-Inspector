#pragma once

#include <chrono>
#include <cstddef>
#include <optional>
#include <vector>

namespace inspection_execution {
namespace tech_1_1 {

class AbnormalSwitchMonitor {
 public:
  using Clock = std::chrono::steady_clock;

  void MarkArrival(Clock::time_point t);
  void MarkSwitchComplete(Clock::time_point t);
  double LastLatencyMs() const;
  double MaxLatencyMs() const;
  std::size_t Count() const;

 private:
  std::optional<Clock::time_point> arrival_;
  std::vector<double> latencies_ms_;
};

}  // namespace tech_1_1
}  // namespace inspection_execution

#pragma once

#include <chrono>
#include <optional>

namespace inspection_execution {

class DataFreshnessGuard {
 public:
  using Clock = std::chrono::steady_clock;

  explicit DataFreshnessGuard(std::chrono::milliseconds max_age);

  void Update(Clock::time_point stamp);
  bool IsFresh(Clock::time_point now) const;
  bool IsStale(Clock::time_point now) const;
  void Reset();

 private:
  std::chrono::milliseconds max_age_;
  std::optional<Clock::time_point> last_update_;
};

}  // namespace inspection_execution

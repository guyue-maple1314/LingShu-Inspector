#include "inspection_execution_cpp/common/data_freshness_guard.hpp"

namespace inspection_execution {

DataFreshnessGuard::DataFreshnessGuard(std::chrono::milliseconds max_age)
    : max_age_(max_age) {}

void DataFreshnessGuard::Update(Clock::time_point stamp) {
  last_update_ = stamp;
}

bool DataFreshnessGuard::IsFresh(Clock::time_point now) const {
  return last_update_.has_value() && (now - *last_update_) <= max_age_;
}

bool DataFreshnessGuard::IsStale(Clock::time_point now) const {
  return !IsFresh(now);
}

void DataFreshnessGuard::Reset() {
  last_update_.reset();
}

}  // namespace inspection_execution

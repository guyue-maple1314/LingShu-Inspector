#include "inspection_execution_cpp/tech_1_6/navigation_executor.hpp"

#include <algorithm>
#include <cmath>

namespace inspection_execution {
namespace tech_1_6 {

NavigationExecutor::NavigationExecutor() = default;

double NavigationExecutor::Distance(const NavWaypoint& a,
                                    const NavWaypoint& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool NavigationExecutor::ReachedWaypoint(const FusionPoseOutput& pose,
                                          const NavWaypoint& wp) const {
  const double dx = pose.x - wp.x;
  const double dy = pose.y - wp.y;
  const double dz = pose.z - wp.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz) <= kWaypointTolerance;
}

bool NavigationExecutor::LoadPath(const std::vector<NavWaypoint>& path) {
  if (path.empty()) return false;
  path_ = path;
  current_wp_ = 0;
  state_ = NavigationState::kIdle;
  return true;
}

bool NavigationExecutor::Start() {
  if (path_.empty()) return false;
  current_wp_ = 0;
  state_ = NavigationState::kNavigating;
  return true;
}

NavigationSnapshot NavigationExecutor::Update(const FusionPoseOutput& pose) {
  NavigationSnapshot snap;
  snap.total_waypoints = path_.size();
  snap.current_waypoint = current_wp_;
  snap.timestamp_ns = pose.timestamp_ns;

  if (state_ == NavigationState::kIdle ||
      state_ == NavigationState::kCompleted ||
      state_ == NavigationState::kFailed) {
    snap.state = state_;
    snap.progress = (state_ == NavigationState::kCompleted) ? 1.0 : 0.0;
    return snap;
  }

  // 退化时暂停推进（不放弃，等待恢复）
  if (degraded_) {
    state_ = NavigationState::kDegradedPause;
    snap.state = state_;
    snap.progress = path_.empty() ? 0.0
                                  : static_cast<double>(current_wp_) /
                                        static_cast<double>(path_.size());
    return snap;
  }

  // 恢复后继续导航
  if (state_ == NavigationState::kDegradedPause) {
    state_ = NavigationState::kNavigating;
  }

  // 检查是否到达当前航点
  if (current_wp_ < path_.size()) {
    const auto& wp = path_[current_wp_];
    if (wp.is_stairs) {
      state_ = NavigationState::kOnStairs;
    } else if (wp.floor_id != 0 && current_wp_ > 0 &&
               path_[current_wp_ - 1].floor_id != wp.floor_id) {
      state_ = NavigationState::kFloorTransition;
    } else {
      state_ = NavigationState::kNavigating;
    }
    snap.current_floor_label = "F" + std::to_string(wp.floor_id);

    if (ReachedWaypoint(pose, wp)) {
      ++current_wp_;
      if (current_wp_ >= path_.size()) {
        state_ = NavigationState::kCompleted;
      }
    }
  }

  // 计算剩余距离
  double remaining = 0.0;
  if (current_wp_ < path_.size()) {
    NavWaypoint cur;
    cur.x = pose.x; cur.y = pose.y; cur.z = pose.z;
    remaining = Distance(cur, path_[current_wp_]);
    for (std::size_t i = current_wp_; i + 1 < path_.size(); ++i) {
      remaining += Distance(path_[i], path_[i + 1]);
    }
  }
  snap.distance_remaining = remaining;

  snap.state = state_;
  snap.current_waypoint = current_wp_;
  snap.progress = path_.empty() ? 0.0
                                : std::min(1.0, static_cast<double>(current_wp_) /
                                                    static_cast<double>(path_.size()));
  return snap;
}

void NavigationExecutor::Complete() {
  state_ = NavigationState::kCompleted;
  current_wp_ = path_.size();
}

void NavigationExecutor::OnDegradation(bool degraded) {
  degraded_ = degraded;
}

NavigationState NavigationExecutor::State() const { return state_; }

double NavigationExecutor::Progress() const {
  if (path_.empty()) return 0.0;
  return std::min(1.0, static_cast<double>(current_wp_) /
                           static_cast<double>(path_.size()));
}

}  // namespace tech_1_6
}  // namespace inspection_execution

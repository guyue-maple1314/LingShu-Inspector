#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "inspection_execution_cpp/tech_1_6/tightly_coupled_localizer.hpp"

namespace inspection_execution {
namespace tech_1_6 {

// 跨楼层导航执行状态
enum class NavigationState : int {
  kIdle = 0,            // 空闲
  kNavigating = 1,      // 平地导航中
  kOnStairs = 2,        // 楼梯/钢格网段
  kFloorTransition = 3, // 楼层切换中
  kDegradedPause = 4,   // 定位退化暂停
  kCompleted = 5,       // 完成
  kFailed = 6,          // 失败
};

// 单个路径航点
struct NavWaypoint {
  double x{0.0};
  double y{0.0};
  double z{0.0};        // 楼层高度
  int floor_id{0};      // 楼层编号
  bool is_stairs{false}; // 是否楼梯段
};

// 导航执行快照
struct NavigationSnapshot {
  NavigationState state{NavigationState::kIdle};
  std::size_t current_waypoint{0};
  std::size_t total_waypoints{0};
  double progress{0.0};          // 0..1
  double distance_remaining{0.0}; // 剩余距离 m
  std::string current_floor_label;
  std::uint64_t timestamp_ns{0};
};

/// 跨楼层路径执行器
/// 基于紧耦合定位位姿推进路径，检测退化条件时暂停
/// 解决钢格网楼梯感知盲区：退化时减速/暂停，恢复后继续
class NavigationExecutor {
 public:
  static constexpr double kWaypointTolerance = 0.10;   // 航点到达容差 10cm
  static constexpr double kDegradedSpeedFactor = 0.3;  // 降级时速度因子

  NavigationExecutor();

  /// 装载跨楼层路径
  bool LoadPath(const std::vector<NavWaypoint>& path);

  /// 开始执行
  bool Start();

  /// 每周期更新（传入最新融合位姿）
  /// 返回当前导航快照
  NavigationSnapshot Update(const FusionPoseOutput& pose);

  /// 完成
  void Complete();

  /// 退化状态注入（由退化监控器驱动）
  void OnDegradation(bool degraded);

  /// 当前状态
  NavigationState State() const;

  /// 当前进度
  double Progress() const;

 private:
  std::vector<NavWaypoint> path_{};
  std::size_t current_wp_{0};
  NavigationState state_{NavigationState::kIdle};
  bool degraded_{false};

  static double Distance(const NavWaypoint& a, const NavWaypoint& b);
  bool ReachedWaypoint(const FusionPoseOutput& pose,
                       const NavWaypoint& wp) const;
};

}  // namespace tech_1_6
}  // namespace inspection_execution

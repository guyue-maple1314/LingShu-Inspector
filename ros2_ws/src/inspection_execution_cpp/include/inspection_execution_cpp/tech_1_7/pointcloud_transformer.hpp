#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace inspection_execution {
namespace tech_1_7 {

// 3D 点（机器人本体坐标系或地图坐标系）
struct Point3D {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// 机器人位姿（来自 1.6 FusionPose，用于点云坐标转换）
struct RobotPose {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qw{1.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  std::uint64_t timestamp_ns{0};
  bool valid{false};
  std::string localization_state{"unknown"};
};

// 转换后的点云（地图/BIM 对齐坐标系）
struct TransformedCloud {
  std::vector<Point3D> points;
  std::string target_frame{"map"};
  std::uint64_t timestamp_ns{0};
  bool valid{false};
};

/// 点云坐标转换器（技术 1.7）
///
/// 把机器人本体坐标系下的点云转换到地图/BIM 对齐坐标系。
/// 依赖 1.6 的 FusionPose 提供机器人位姿。
///
/// 红线：位姿无效（localization_state=="lost" 或 valid=false）时
///   不强行转换，返回 valid=false（不虚构物理位置）。
class PointcloudTransformer {
 public:
  /// 设置目标坐标系（默认 "map"，可对齐 BIM）
  void SetTargetFrame(const std::string& frame);

  /// 转换点云：robot_cloud（本体坐标系） + robot_pose（FusionPose） → map 系
  /// 位姿无效时返回 valid=false
  TransformedCloud Transform(const std::vector<Point3D>& robot_cloud,
                              const RobotPose& robot_pose) const;

  /// 最近一次目标坐标系
  const std::string& TargetFrame() const;

 private:
  std::string target_frame_{"map"};

  // 四元数 → 旋转矩阵 + 平移，应用到一个点
  static Point3D ApplyPose(const Point3D& p, const RobotPose& pose);
};

}  // namespace tech_1_7
}  // namespace inspection_execution

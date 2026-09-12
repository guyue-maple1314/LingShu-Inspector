#include "inspection_execution_cpp/tech_1_7/pointcloud_transformer.hpp"

#include <cmath>

namespace inspection_execution {
namespace tech_1_7 {

void PointcloudTransformer::SetTargetFrame(const std::string& frame) {
  target_frame_ = frame.empty() ? "map" : frame;
}

const std::string& PointcloudTransformer::TargetFrame() const {
  return target_frame_;
}

Point3D PointcloudTransformer::ApplyPose(const Point3D& p,
                                          const RobotPose& pose) {
  // 四元数 q = (qw, qx, qy, qz) 旋转点 p
  // 标准公式：p' = q * p * q_conj + t
  const double qw = pose.qw;
  const double qx = pose.qx;
  const double qy = pose.qy;
  const double qz = pose.qz;

  // 四元数归一化（防御性，输入应已归一化）
  const double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (norm < 1e-12) {
    // 单位四元数退化，仅平移
    return Point3D{p.x + pose.x, p.y + pose.y, p.z + pose.z};
  }
  const double nw = qw / norm;
  const double nx = qx / norm;
  const double ny = qy / norm;
  const double nz = qz / norm;

  // 旋转矩阵形式（避免四元数乘法展开错误）
  const double tx = p.x;
  const double ty = p.y;
  const double tz = p.z;
  const double rx = (1.0 - 2.0 * (ny * ny + nz * nz)) * tx +
                    2.0 * (nx * ny - nw * nz) * ty +
                    2.0 * (nx * nz + nw * ny) * tz;
  const double ry = 2.0 * (nx * ny + nw * nz) * tx +
                    (1.0 - 2.0 * (nx * nx + nz * nz)) * ty +
                    2.0 * (ny * nz - nw * nx) * tz;
  const double rz = 2.0 * (nx * nz - nw * ny) * tx +
                    2.0 * (ny * nz + nw * nx) * ty +
                    (1.0 - 2.0 * (nx * nx + ny * ny)) * tz;

  return Point3D{rx + pose.x, ry + pose.y, rz + pose.z};
}

TransformedCloud PointcloudTransformer::Transform(
    const std::vector<Point3D>& robot_cloud, const RobotPose& robot_pose) const {
  TransformedCloud out;
  out.target_frame = target_frame_;
  out.timestamp_ns = robot_pose.timestamp_ns;

  // 红线：位姿无效或定位丢失时不虚构转换结果
  if (!robot_pose.valid || robot_pose.localization_state == "lost") {
    out.valid = false;
    return out;
  }

  out.points.reserve(robot_cloud.size());
  for (const auto& p : robot_cloud) {
    out.points.push_back(ApplyPose(p, robot_pose));
  }
  out.valid = true;
  return out;
}

}  // namespace tech_1_7
}  // namespace inspection_execution

#include <cmath>

#include "inspection_execution_cpp/tech_1_8/thermal_image_stabilizer.hpp"

namespace inspection_execution {
namespace tech_1_8 {

ThermalImageStabilizer::ThermalImageStabilizer(double exposure_ms,
                                               double focal_factor)
    : exposure_ms_(exposure_ms), focal_factor_(focal_factor) {}

StabilizedThermal ThermalImageStabilizer::Stabilize(
    const SyncedThermalPacket& packet) {
  StabilizedThermal out;
  out.timestamp_ns = packet.synced_timestamp_ns;

  // 红线：红外帧无效时不稳像
  if (!packet.thermal_valid) {
    out.error_state = "thermal_invalid";
    return out;
  }
  // 红线：IMU 无效时不虚构位移补偿
  if (!packet.imu_valid) {
    out.error_state = "imu_invalid";
    return out;
  }

  // 用 IMU 角速度（rad/s）估计曝光时间内的图像位移
  // 位移量 = 角速度 × 曝光时间 × 焦距系数
  const double dt_sec = exposure_ms_ * 0.001;
  out.displacement_x = packet.imu.gy * dt_sec * focal_factor_;
  out.displacement_y = -packet.imu.gx * dt_sec * focal_factor_;  // x/y 轴对应

  // 稳像后温度值不变（同帧像素），透传测量条件
  out.stabilized_temperature = packet.thermal.raw_temperature;
  out.angle = packet.thermal.angle;
  out.distance = packet.thermal.distance;
  out.emissivity = packet.thermal.emissivity;
  out.correction_factor = packet.thermal.correction_factor;
  out.valid = true;
  out.error_state = "ok";
  return out;
}

double ThermalImageStabilizer::ExposureMs() const { return exposure_ms_; }
double ThermalImageStabilizer::FocalFactor() const { return focal_factor_; }

}  // namespace tech_1_8
}  // namespace inspection_execution

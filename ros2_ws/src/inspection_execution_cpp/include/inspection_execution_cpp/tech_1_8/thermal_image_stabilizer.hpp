#pragma once

#include <cstdint>
#include <string>

#include "inspection_execution_cpp/tech_1_8/thermal_visual_imu_synchronizer.hpp"

namespace inspection_execution {
namespace tech_1_8 {

/// 稳像后的红外测量
struct StabilizedThermal {
  double stabilized_temperature{0.0};  // 稳像后温度（值与原始相同，标记已补偿位移）
  double displacement_x{0.0};          // 图像 X 位移补偿量（像素 / 归一化）
  double displacement_y{0.0};          // 图像 Y 位移补偿量
  double angle{0.0};                   // 透传测量角度
  double distance{0.0};                // 透传测量距离
  double emissivity{1.0};              // 透传辐射率
  double correction_factor{1.0};       // 透传修正系数
  std::uint64_t timestamp_ns{0};
  bool valid{false};
  std::string error_state;  // "ok" / "imu_invalid" / "thermal_invalid"
};

/// 热像稳像器（技术 1.8）
///
/// 补偿机器人步态振动造成的红外图像位移：
///   - 用 IMU 角速度（gx/gy）估计曝光时间内的图像位移
///   - 位移量 = 角速度 × 曝光时间 × 焦距系数
///   - 稳像后温度值不变（同帧像素），但记录位移量供后续补偿链使用
///
/// 红线：
///   - IMU 无效时不虚构位移补偿（valid=false, error_state="imu_invalid"）
///   - 红外帧无效时不稳像（valid=false, error_state="thermal_invalid"）
///   - 稳像在 C++ 执行（Python 不重复实现，红线）
class ThermalImageStabilizer {
 public:
  static constexpr double kDefaultExposureMs = 8.0;      // 默认曝光 8ms
  static constexpr double kDefaultFocalFactor = 400.0;   // 焦距系数（像素/(rad·ms)）

  explicit ThermalImageStabilizer(double exposure_ms = kDefaultExposureMs,
                                  double focal_factor = kDefaultFocalFactor);

  /// 对同步包做稳像
  StabilizedThermal Stabilize(const SyncedThermalPacket& packet);

  /// 曝光时间（ms）
  double ExposureMs() const;

  /// 焦距系数
  double FocalFactor() const;

 private:
  double exposure_ms_{kDefaultExposureMs};
  double focal_factor_{kDefaultFocalFactor};
};

}  // namespace tech_1_8
}  // namespace inspection_execution

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "inspection_execution_cpp/tech_1_7/pointcloud_transformer.hpp"

namespace inspection_execution {
namespace tech_1_7 {

// 单个栅格单元
struct GridCell {
  int index_x{0};
  int index_y{0};
  double weight{0.0};             // 综合权重 [0,1]
  double bim_weight{0.0};          // BIM 先验权重
  double slam_weight{0.0};         // SLAM 实时权重
  double cloud_weight{0.0};        // 点云实测权重
  std::string bim_id;              // 对应 BIM 构件标识（可空）
  bool has_measurement{false};     // 是否有实测数据
};

// 栅格地图
struct WeightedGrid {
  std::vector<GridCell> cells;
  double origin_x{0.0};
  double origin_y{0.0};
  double resolution_m{0.05};       // 栅格分辨率（默认 5cm，对应厘米级映射）
  int size_x{0};
  int size_y{0};
  std::uint64_t timestamp_ns{0};
  bool valid{false};
};

// 输入：BIM 构件（先验，来自 Python bim_loader 抽象接口的简化结构）
struct BimElement {
  std::string id;
  double center_x{0.0};
  double center_y{0.0};
  double center_z{0.0};
  double prior_weight{0.5};       // BIM 先验置信度
};

// 输入：SLAM 观测（实时，来自 1.6 FusionPose 周边观测）
struct SlamObservation {
  std::vector<Point3D> landmarks;
  double confidence{0.5};
};

/// 栅格赋权器（技术 1.7）
///
/// 按输入信息（点云 / BIM 先验 / SLAM 实时）为栅格赋予不同权重：
///   - BIM 先验：项目方批准的静态结构信息
///   - SLAM 实时：1.6 紧耦合定位周边观测
///   - 点云实测：机器人订阅点云转换后落入栅格
///
/// 综合权重 = 加权和，三种来源都缺失时 weight=0（不虚构）。
class WeightedGridBuilder {
 public:
  static constexpr double kDefaultResolutionM = 0.05;  // 5cm 厘米级
  static constexpr double kBimWeightFactor = 0.4;
  static constexpr double kSlamWeightFactor = 0.3;
  static constexpr double kCloudWeightFactor = 0.3;

  explicit WeightedGridBuilder(double resolution_m = kDefaultResolutionM);

  /// 设置地图原点与尺寸
  void SetMapExtent(double origin_x, double origin_y, int size_x, int size_y);

  /// 装入 BIM 先验
  void LoadBim(const std::vector<BimElement>& bim_elements);

  /// 装入 SLAM 实时观测
  void LoadSlam(const SlamObservation& slam);

  /// 装入转换后的点云
  void LoadCloud(const TransformedCloud& cloud);

  /// 构建带权重的栅格地图
  /// 无任何数据源时返回 valid=false（不虚构）
  WeightedGrid Build(std::uint64_t timestamp_ns) const;

  /// 最近一次 BIM 元素数
  std::size_t BimCount() const;

 private:
  double resolution_m_{kDefaultResolutionM};
  double origin_x_{0.0};
  double origin_y_{0.0};
  int size_x_{100};
  int size_y_{100};

  std::vector<BimElement> bim_elements_{};
  SlamObservation slam_{};
  TransformedCloud cloud_{};

  // 把连续坐标量化到栅格索引
  void ToIndex(double x, double y, int* ix, int* iy) const;
};

}  // namespace tech_1_7
}  // namespace inspection_execution

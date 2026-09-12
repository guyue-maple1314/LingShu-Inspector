#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "inspection_execution_cpp/common/node_names.hpp"
#include "inspection_execution_cpp/common/topic_names.hpp"
#include "inspection_execution_cpp/tech_1_7/pointcloud_transformer.hpp"
#include "inspection_execution_cpp/tech_1_7/weighted_grid_builder.hpp"
#include "inspection_execution_cpp/tech_1_7/position_matcher.hpp"
#include "inspection_execution_cpp/tech_1_7/alarm_location_publisher.hpp"
#include "inspection_interfaces/msg/fusion_pose.hpp"
#include "inspection_interfaces/msg/inspection_alert.hpp"
#include "inspection_interfaces/msg/robot_state.hpp"
#include "inspection_interfaces/msg/semantic_alarm.hpp"

namespace inspection_execution {
namespace tech_1_7 {

/// 高精度时空融合语义地图与定位节点（技术 1.7）
///
/// 装配四组件：
///   PointcloudTransformer → WeightedGridBuilder
///   → PositionMatcher → AlarmLocationPublisher
///
/// 数据流：
///   订阅 /fusion_pose（1.6 输出）→ 机器人位姿
///   订阅 /inspection_alert（1.1/1.8/1.9 检测事件）→ 触发位置匹配
///   订阅 /robot_state（备用位姿源）
///   发布 /semantic_alarm（带置信度 + 坐标来源）
///
/// 红线：
///   - 位姿无效（lost）时不强行转换点云（不虚构物理位置）
///   - 无法对应时保持"未定位"（localized=false），不输出虚假物理监测点
///   - BIM 走 Python 抽象接口（AbstractBimLoader），C++ 侧只接收简化 BimElement
class Tech17Node : public rclcpp::Node {
 public:
  Tech17Node() : rclcpp::Node(node_names::kTech1_7Node) {
    this->declare_parameter("grid_resolution_m", 0.05);
    this->declare_parameter("search_radius_m", 0.30);
    this->declare_parameter("map_origin_x", 0.0);
    this->declare_parameter("map_origin_y", 0.0);
    this->declare_parameter("map_size_x", 100);
    this->declare_parameter("map_size_y", 100);
    this->declare_parameter("control_period_ms", 200);

    // 初始化四组件
    const double resolution = this->get_parameter("grid_resolution_m").as_double();
    grid_builder_ = std::make_unique<WeightedGridBuilder>(resolution);
    grid_builder_->SetMapExtent(
        this->get_parameter("map_origin_x").as_double(),
        this->get_parameter("map_origin_y").as_double(),
        this->get_parameter("map_size_x").as_int(),
        this->get_parameter("map_size_y").as_int());

    const double search_r = this->get_parameter("search_radius_m").as_double();
    position_matcher_ = std::make_unique<PositionMatcher>(search_r);

    // 发布 /semantic_alarm
    alarm_pub_ = this->create_publisher<inspection_interfaces::msg::SemanticAlarm>(
        topic_names::kSemanticAlarm, 10);

    // 订阅 /fusion_pose → 机器人位姿（来自 1.6）
    fusion_sub_ = this->create_subscription<inspection_interfaces::msg::FusionPose>(
        topic_names::kFusionPose, 10,
        [this](const inspection_interfaces::msg::FusionPose::SharedPtr msg) {
          OnFusionPose(*msg);
        });

    // 订阅 /inspection_alert → 检测事件（来自 1.1/1.8/1.9）
    alert_sub_ = this->create_subscription<inspection_interfaces::msg::InspectionAlert>(
        topic_names::kInspectionAlert, 10,
        [this](const inspection_interfaces::msg::InspectionAlert::SharedPtr msg) {
          OnInspectionAlert(*msg);
        });

    // 订阅 /robot_state → 备用位姿源
    robot_state_sub_ = this->create_subscription<inspection_interfaces::msg::RobotState>(
        topic_names::kRobotState, 10,
        [this](const inspection_interfaces::msg::RobotState::SharedPtr msg) {
          OnRobotState(*msg);
        });

    const int period_ms = this->get_parameter("control_period_ms").as_int();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        [this]() { Tick(); });

    RCLCPP_INFO(this->get_logger(),
                "%s started: 4-component semantic map (BIM/SLAM/cloud fusion)",
                node_names::kTech1_7Node);
  }

  // ---- 适配器/数据注入接口 ----
  void SetRobotCloud(const std::vector<Point3D>& cloud) {
    pending_robot_cloud_ = cloud;
  }

 private:
  void OnFusionPose(const inspection_interfaces::msg::FusionPose& msg) {
    latest_pose_.x = msg.pose.pose.position.x;
    latest_pose_.y = msg.pose.pose.position.y;
    latest_pose_.z = msg.pose.pose.position.z;
    latest_pose_.qw = msg.pose.pose.orientation.w;
    latest_pose_.qx = msg.pose.pose.orientation.x;
    latest_pose_.qy = msg.pose.pose.orientation.y;
    latest_pose_.qz = msg.pose.pose.orientation.z;
    latest_pose_.timestamp_ns = static_cast<std::uint64_t>(msg.header.stamp.nanosec) +
        static_cast<std::uint64_t>(msg.header.stamp.sec) * 1000000000ULL;
    latest_pose_.valid = true;
    latest_pose_.localization_state = msg.localization_state;
  }

  void OnRobotState(const inspection_interfaces::msg::RobotState& msg) {
    // 仅在 FusionPose 未到达时作降级位姿
    if (!latest_pose_.valid) {
      latest_pose_.x = msg.pose.position.x;
      latest_pose_.y = msg.pose.position.y;
      latest_pose_.z = msg.pose.position.z;
      latest_pose_.qw = msg.pose.orientation.w;
      latest_pose_.qx = msg.pose.orientation.x;
      latest_pose_.qy = msg.pose.orientation.y;
      latest_pose_.qz = msg.pose.orientation.z;
      latest_pose_.timestamp_ns = static_cast<std::uint64_t>(msg.header.stamp.nanosec) +
          static_cast<std::uint64_t>(msg.header.stamp.sec) * 1000000000ULL;
      latest_pose_.valid = true;
      latest_pose_.localization_state = "robot_state";
    }
  }

  void OnInspectionAlert(const inspection_interfaces::msg::InspectionAlert& msg) {
    DetectionEvent ev;
    ev.event_id = msg.source_device + ":" + msg.alert_type;
    ev.alert_type = msg.alert_type;
    ev.source_device = msg.source_device;
    ev.detected_value = msg.detected_value;
    ev.detection_x = msg.detection_pose.position.x;
    ev.detection_y = msg.detection_pose.position.y;
    ev.detection_z = msg.detection_pose.position.z;
    ev.timestamp_ns = static_cast<std::uint64_t>(msg.header.stamp.nanosec) +
        static_cast<std::uint64_t>(msg.header.stamp.sec) * 1000000000ULL;
    // 位姿是否可用由消息显式声明（InspectionAlert.pose_valid）：
    // 未携带有效位姿的告警不做位置匹配，只发布"未定位"（红线）
    ev.valid = msg.pose_valid;
    pending_events_.push_back(ev);
  }

  void Tick() {
    // 1. 点云转换（位姿无效时不虚构，transformer 内部处理）
    if (!pending_robot_cloud_.empty()) {
      const auto transformed = transformer_.Transform(pending_robot_cloud_, latest_pose_);
      if (transformed.valid) {
        grid_builder_->LoadCloud(transformed);
      }
      pending_robot_cloud_.clear();
    }

    // 2. 构建栅格地图
    const std::uint64_t now_ns = static_cast<std::uint64_t>(this->now().nanoseconds());
    const auto grid = grid_builder_->Build(now_ns);
    if (!grid.valid) {
      // 无任何数据源 → 待事件到达后只发"未定位"（不虚构栅格）
      PublishUnlocalizedForPendingEvents(now_ns);
      return;
    }

    // 3. 批量匹配检测事件
    if (pending_events_.empty()) return;
    const auto results = position_matcher_->MatchAll(pending_events_, grid);
    pending_events_.clear();

    // 4. 发布 SemanticAlarm
    for (const auto& r : results) {
      const auto fields = AlarmLocationPublisher::Publish(r);
      inspection_interfaces::msg::SemanticAlarm msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = "map";
      msg.detection_event = fields.detection_event;
      if (fields.localized) {
        msg.physical_location.position.x = fields.location_x;
        msg.physical_location.position.y = fields.location_y;
        msg.physical_location.position.z = fields.location_z;
        msg.physical_location.orientation.w = fields.orientation_qw;
        msg.physical_location.orientation.x = fields.orientation_qx;
        msg.physical_location.orientation.y = fields.orientation_qy;
        msg.physical_location.orientation.z = fields.orientation_qz;
      }
      // 未定位时 physical_location 保持默认（不写虚假监测点）
      msg.bim_id = fields.bim_id;
      msg.slam_id = fields.slam_id;
      msg.confidence = fields.confidence;
      alarm_pub_->publish(msg);

      if (!fields.localized) {
        RCLCPP_WARN(this->get_logger(),
                    "event %s unlocalized (no matching grid cell, confidence=0)",
                    fields.detection_event.c_str());
      }
    }
  }

  void PublishUnlocalizedForPendingEvents(std::uint64_t now_ns) {
    for (const auto& ev : pending_events_) {
      inspection_interfaces::msg::SemanticAlarm msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = "map";
      msg.detection_event = ev.event_id;
      // physical_location 保持默认零值（未定位，不虚构）
      msg.bim_id.clear();
      msg.slam_id.clear();
      msg.confidence = 0.0;
      alarm_pub_->publish(msg);
      (void)now_ns;
    }
    pending_events_.clear();
  }

  PointcloudTransformer transformer_{};
  std::unique_ptr<WeightedGridBuilder> grid_builder_{};
  std::unique_ptr<PositionMatcher> position_matcher_{};
  AlarmLocationPublisher publisher_{};

  RobotPose latest_pose_{};
  std::vector<Point3D> pending_robot_cloud_{};
  std::vector<DetectionEvent> pending_events_{};

  rclcpp::Publisher<inspection_interfaces::msg::SemanticAlarm>::SharedPtr alarm_pub_{};
  rclcpp::Subscription<inspection_interfaces::msg::FusionPose>::SharedPtr fusion_sub_{};
  rclcpp::Subscription<inspection_interfaces::msg::InspectionAlert>::SharedPtr alert_sub_{};
  rclcpp::Subscription<inspection_interfaces::msg::RobotState>::SharedPtr robot_state_sub_{};
  rclcpp::TimerBase::SharedPtr timer_{};
};

}  // namespace tech_1_7
}  // namespace inspection_execution

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<inspection_execution::tech_1_7::Tech17Node>());
  rclcpp::shutdown();
  return 0;
}

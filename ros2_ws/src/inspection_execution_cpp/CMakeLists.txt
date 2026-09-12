cmake_minimum_required(VERSION 3.8)
project(inspection_execution_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 著作权锚点：编译期编入全部目标（strings 二进制可验证），请勿删除。
add_compile_definitions(
  INSPECTION_OWNERSHIP_MARK="qdgw/lszx_byc 2026 青岛港湾职业技术学院 灵枢智行"
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(inspection_interfaces REQUIRED)

add_library(inspection_execution_common
  src/common/error_codes.cpp
  src/common/data_freshness_guard.cpp
  src/common/execution_result.cpp
)
target_include_directories(inspection_execution_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(cpp_heartbeat_node src/connectivity/cpp_heartbeat_node.cpp)
target_link_libraries(cpp_heartbeat_node inspection_execution_common)
ament_target_dependencies(cpp_heartbeat_node
  rclcpp
  std_msgs
  geometry_msgs
  builtin_interfaces
  inspection_interfaces
)

add_executable(tech_1_1_node
  src/tech_1_1/tech_1_1_node.cpp
  src/tech_1_1/bt_task_executor.cpp
  src/tech_1_1/behavior_tree_nodes.cpp
  src/tech_1_1/goal_safety_validator.cpp
  src/tech_1_1/abnormal_switch_monitor.cpp
)
target_link_libraries(tech_1_1_node inspection_execution_common)
target_link_libraries(tech_1_1_node BT::behaviortree_cpp)
ament_target_dependencies(tech_1_1_node
  rclcpp
  std_msgs
  geometry_msgs
  builtin_interfaces
  inspection_interfaces
)

add_executable(tech_1_2_node
  src/tech_1_2/tech_1_2_node.cpp
  src/tech_1_2/task_lifecycle_executor.cpp
  src/tech_1_2/task_resume_executor.cpp
  src/tech_1_2/task_progress_store.cpp
  src/tech_1_2/preemption_latency_monitor.cpp
  src/tech_1_1/goal_safety_validator.cpp
)
target_link_libraries(tech_1_2_node inspection_execution_common)
ament_target_dependencies(tech_1_2_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  builtin_interfaces
  inspection_interfaces
)

add_executable(tech_1_3_node
  src/tech_1_3/tech_1_3_node.cpp
  src/tech_1_3/ppo_policy_loader.cpp
  src/tech_1_3/observation_builder.cpp
  src/tech_1_3/policy_output_validator.cpp
  src/tech_1_3/locomotion_command_adapter.cpp
)
target_link_libraries(tech_1_3_node inspection_execution_common)
ament_target_dependencies(tech_1_3_node
  rclcpp
  std_msgs
  geometry_msgs
  builtin_interfaces
  inspection_interfaces
)

add_executable(tech_1_4_node
  src/tech_1_4/tech_1_4_node.cpp
  src/tech_1_4/dynamic_pointcloud_segmenter.cpp
  src/tech_1_4/visual_texture_validator.cpp
  src/tech_1_4/corridor_fusion.cpp
  src/tech_1_4/narrow_corridor_executor.cpp
)
target_link_libraries(tech_1_4_node inspection_execution_common)
ament_target_dependencies(tech_1_4_node
  rclcpp
  std_msgs
  geometry_msgs
  builtin_interfaces
  inspection_interfaces
)

add_executable(tech_1_5_node
  src/tech_1_5/tech_1_5_node.cpp
  src/tech_1_5/foot_contact_estimator.cpp
  src/tech_1_5/vibration_estimator.cpp
  src/tech_1_5/mpc_vibration_controller.cpp
  src/tech_1_5/grating_metrics_recorder.cpp
)
target_link_libraries(tech_1_5_node inspection_execution_common)
ament_target_dependencies(tech_1_5_node
  rclcpp
  std_msgs
  geometry_msgs
  builtin_interfaces
  inspection_interfaces
)

add_executable(tech_1_6_node
  src/tech_1_6/tech_1_6_node.cpp
  src/tech_1_6/multi_sensor_synchronizer.cpp
  src/tech_1_6/kinematic_constraint_builder.cpp
  src/tech_1_6/tightly_coupled_localizer.cpp
  src/tech_1_6/localization_degradation_monitor.cpp
  src/tech_1_6/navigation_executor.cpp
)
target_link_libraries(tech_1_6_node inspection_execution_common)
ament_target_dependencies(tech_1_6_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  builtin_interfaces
  inspection_interfaces
)

add_executable(tech_1_7_node
  src/tech_1_7/tech_1_7_node.cpp
  src/tech_1_7/pointcloud_transformer.cpp
  src/tech_1_7/weighted_grid_builder.cpp
  src/tech_1_7/position_matcher.cpp
  src/tech_1_7/alarm_location_publisher.cpp
)
target_link_libraries(tech_1_7_node inspection_execution_common)
ament_target_dependencies(tech_1_7_node
  rclcpp
  std_msgs
  geometry_msgs
  builtin_interfaces
  inspection_interfaces
)

add_executable(tech_1_8_node
  src/tech_1_8/tech_1_8_node.cpp
  src/tech_1_8/thermal_visual_imu_synchronizer.cpp
  src/tech_1_8/thermal_image_stabilizer.cpp
  src/tech_1_8/emissivity_compensator.cpp
  src/tech_1_8/angle_distance_compensator.cpp
  src/tech_1_8/thermal_range_validator.cpp
)
target_link_libraries(tech_1_8_node inspection_execution_common)
ament_target_dependencies(tech_1_8_node
  rclcpp
  std_msgs
  geometry_msgs
  builtin_interfaces
  inspection_interfaces
)

add_executable(tech_1_9_node
  src/tech_1_9/tech_1_9_node.cpp
  src/tech_1_9/microphone_synchronizer.cpp
  src/tech_1_9/directional_beamformer.cpp
  src/tech_1_9/spatial_filter.cpp
  src/tech_1_9/snr_estimator.cpp
)
target_link_libraries(tech_1_9_node inspection_execution_common)
ament_target_dependencies(tech_1_9_node
  rclcpp
  std_msgs
  geometry_msgs
  builtin_interfaces
  inspection_interfaces
)

install(TARGETS
  cpp_heartbeat_node
  tech_1_1_node
  tech_1_2_node
  tech_1_3_node
  tech_1_4_node
  tech_1_5_node
  tech_1_6_node
  tech_1_7_node
  tech_1_8_node
  tech_1_9_node
  inspection_execution_common
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config behavior_trees DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(inspection_execution_common)

ament_package()

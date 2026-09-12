#pragma once

namespace inspection_execution {
namespace topic_names {

// 共享业务 Topic
inline constexpr const char* kRobotState = "/robot_state";
inline constexpr const char* kOperatorInstruction = "/operator_instruction";
inline constexpr const char* kInspectionAlert = "/inspection_alert";
inline constexpr const char* kStructuredGoal = "/structured_goal";
inline constexpr const char* kTask = "/task";
inline constexpr const char* kEnergyConstraint = "/energy_constraint";
inline constexpr const char* kTaskDecision = "/task_decision";
inline constexpr const char* kTerrainObservation = "/terrain_observation";
inline constexpr const char* kCorridorState = "/corridor_state";
inline constexpr const char* kGratingStatus = "/grating_status";
inline constexpr const char* kFusionPose = "/fusion_pose";
inline constexpr const char* kSemanticAlarm = "/semantic_alarm";
inline constexpr const char* kThermalMeasurement = "/thermal_measurement";
inline constexpr const char* kAcousticDiagnosis = "/acoustic_diagnosis";
inline constexpr const char* kAcousticMono = "/acoustic_mono";

// 原始传感器 Topic
inline constexpr const char* kImu = "/imu";
inline constexpr const char* kFootForce = "/foot_force";
inline constexpr const char* kLidarScan = "/lidar_scan";
inline constexpr const char* kImage = "/image";
inline constexpr const char* kThermalImage = "/thermal_image";
inline constexpr const char* kAudioMultiChannel = "/audio_multi_channel";

// Service / Action 名称
inline constexpr const char* kValidateGoalService = "/validate_goal";
inline constexpr const char* kResumeTaskService = "/resume_task";
inline constexpr const char* kExecuteTaskAction = "/execute_task";
inline constexpr const char* kNavigateGoalAction = "/navigate_goal";

}  // namespace topic_names
}  // namespace inspection_execution

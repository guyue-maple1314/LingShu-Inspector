#include "inspection_execution_cpp/common/error_codes.hpp"

namespace inspection_execution {

const char* ErrorCodeToString(ErrorCode code) {
  switch (code) {
    case ErrorCode::kOk: return "OK";
    case ErrorCode::kInvalidGoal: return "INVALID_GOAL";
    case ErrorCode::kGoalExpired: return "GOAL_EXPIRED";
    case ErrorCode::kStaleSensorData: return "STALE_SENSOR_DATA";
    case ErrorCode::kSensorInvalid: return "SENSOR_INVALID";
    case ErrorCode::kSafetyRejected: return "SAFETY_REJECTED";
    case ErrorCode::kExecutionFailed: return "EXECUTION_FAILED";
    case ErrorCode::kCorridorInvalid: return "CORRIDOR_INVALID";
    case ErrorCode::kVibrationAnomaly: return "VIBRATION_ANOMALY";
    case ErrorCode::kMpcSolverNotReady: return "MPC_SOLVER_NOT_READY";
    case ErrorCode::kLocalizationDegraded: return "LOCALIZATION_DEGRADED";
    case ErrorCode::kThermalOutOfRange: return "THERMAL_OUT_OF_RANGE";
    case ErrorCode::kAcousticLowConfidence: return "ACOUSTIC_LOW_CONFIDENCE";
  }
  return "UNKNOWN";
}

}  // namespace inspection_execution

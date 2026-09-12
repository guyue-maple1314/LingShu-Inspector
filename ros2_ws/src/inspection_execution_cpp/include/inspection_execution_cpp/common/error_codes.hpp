#pragma once

#include <cstdint>

namespace inspection_execution {

enum class ErrorCode : std::int32_t {
  kOk = 0,
  kInvalidGoal = 1001,
  kGoalExpired = 1002,
  kStaleSensorData = 2001,
  kSensorInvalid = 2002,
  kSafetyRejected = 3001,
  kExecutionFailed = 4001,
  kCorridorInvalid = 5001,
  kVibrationAnomaly = 5501,
  kMpcSolverNotReady = 5502,
  kLocalizationDegraded = 6001,
  kThermalOutOfRange = 7001,
  kAcousticLowConfidence = 8001,
};

const char* ErrorCodeToString(ErrorCode code);

}  // namespace inspection_execution

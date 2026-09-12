#pragma once

#include <cstdint>
#include <string>

namespace inspection_execution {

struct ExecutionResult {
  bool success{false};
  std::int32_t error_code{0};
  std::string message;

  static ExecutionResult Ok(const std::string& message = "ok");
  static ExecutionResult Failure(std::int32_t error_code, const std::string& message);
};

}  // namespace inspection_execution

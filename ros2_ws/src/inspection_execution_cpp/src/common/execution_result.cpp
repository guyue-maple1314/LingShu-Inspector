#include "inspection_execution_cpp/common/execution_result.hpp"

namespace inspection_execution {

ExecutionResult ExecutionResult::Ok(const std::string& message) {
  ExecutionResult result;
  result.success = true;
  result.error_code = 0;
  result.message = message;
  return result;
}

ExecutionResult ExecutionResult::Failure(std::int32_t error_code, const std::string& message) {
  ExecutionResult result;
  result.success = false;
  result.error_code = error_code;
  result.message = message;
  return result;
}

}  // namespace inspection_execution

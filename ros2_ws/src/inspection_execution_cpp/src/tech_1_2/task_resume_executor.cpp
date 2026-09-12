#include "inspection_execution_cpp/tech_1_2/task_resume_executor.hpp"

namespace inspection_execution {
namespace tech_1_2 {

TaskResumeExecutor::TaskResumeExecutor(const TaskProgressStore& store) : store_(store) {}

bool TaskResumeExecutor::CanResume(const std::string& task_id) const {
  return store_.Has(task_id);
}

bool TaskResumeExecutor::LoadResumePoint(const std::string& task_id, TaskProgress* progress) const {
  return store_.Load(task_id, progress);
}

}  // namespace tech_1_2
}  // namespace inspection_execution

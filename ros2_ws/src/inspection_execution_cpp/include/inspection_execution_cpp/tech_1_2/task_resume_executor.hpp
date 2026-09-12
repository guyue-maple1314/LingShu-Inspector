#pragma once

#include <string>

#include "inspection_execution_cpp/tech_1_2/task_progress_store.hpp"

namespace inspection_execution {
namespace tech_1_2 {

// 接收恢复决定，从已保存进度构造可恢复点。
class TaskResumeExecutor {
 public:
  explicit TaskResumeExecutor(const TaskProgressStore& store);
  bool CanResume(const std::string& task_id) const;
  bool LoadResumePoint(const std::string& task_id, TaskProgress* progress) const;

 private:
  const TaskProgressStore& store_;
};

}  // namespace tech_1_2
}  // namespace inspection_execution

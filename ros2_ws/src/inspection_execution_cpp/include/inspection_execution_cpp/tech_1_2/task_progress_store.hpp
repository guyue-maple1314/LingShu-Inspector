#pragma once

#include <cstddef>
#include <mutex>
#include <string>
#include <unordered_map>

namespace inspection_execution {
namespace tech_1_2 {

struct TaskProgress {
  double progress{0.0};  // 0.0..1.0
  std::string state;
};

/// 任务进度存储：Action 执行线程（可多个）写、resume 服务线程读，必须加锁。
class TaskProgressStore {
 public:
  void Save(const std::string& task_id, const TaskProgress& progress);
  bool Load(const std::string& task_id, TaskProgress* progress) const;
  bool Has(const std::string& task_id) const;
  void Clear(const std::string& task_id);
  std::size_t Size() const;

 private:
  mutable std::mutex mtx_;
  std::unordered_map<std::string, TaskProgress> store_;
};

}  // namespace tech_1_2
}  // namespace inspection_execution

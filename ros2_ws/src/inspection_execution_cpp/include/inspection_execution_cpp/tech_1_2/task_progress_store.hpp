#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>

namespace inspection_execution {
namespace tech_1_2 {

struct TaskProgress {
  double progress{0.0};  // 0.0..1.0
  std::string state;
};

class TaskProgressStore {
 public:
  void Save(const std::string& task_id, const TaskProgress& progress);
  bool Load(const std::string& task_id, TaskProgress* progress) const;
  bool Has(const std::string& task_id) const;
  void Clear(const std::string& task_id);
  std::size_t Size() const;

 private:
  std::unordered_map<std::string, TaskProgress> store_;
};

}  // namespace tech_1_2
}  // namespace inspection_execution

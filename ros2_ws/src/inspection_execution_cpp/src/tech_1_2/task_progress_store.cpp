#include "inspection_execution_cpp/tech_1_2/task_progress_store.hpp"

namespace inspection_execution {
namespace tech_1_2 {

void TaskProgressStore::Save(const std::string& task_id, const TaskProgress& progress) {
  std::lock_guard<std::mutex> lock(mtx_);
  store_[task_id] = progress;
}

bool TaskProgressStore::Load(const std::string& task_id, TaskProgress* progress) const {
  std::lock_guard<std::mutex> lock(mtx_);
  auto it = store_.find(task_id);
  if (it == store_.end()) {
    return false;
  }
  if (progress != nullptr) {
    *progress = it->second;
  }
  return true;
}

bool TaskProgressStore::Has(const std::string& task_id) const {
  std::lock_guard<std::mutex> lock(mtx_);
  return store_.find(task_id) != store_.end();
}

void TaskProgressStore::Clear(const std::string& task_id) {
  std::lock_guard<std::mutex> lock(mtx_);
  store_.erase(task_id);
}

std::size_t TaskProgressStore::Size() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return store_.size();
}

}  // namespace tech_1_2
}  // namespace inspection_execution

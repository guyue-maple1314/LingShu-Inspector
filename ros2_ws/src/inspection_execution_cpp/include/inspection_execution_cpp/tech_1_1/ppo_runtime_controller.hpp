#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace inspection_execution {
namespace tech_1_1 {

struct PolicyMetadata {
  std::string version;
  std::uint32_t observation_dim{0};
  std::uint32_t action_dim{0};
};

// PPO 策略运行时抽象接口；实机模型获批后实现，不虚构推理结果。
class PpoPolicyRuntime {
 public:
  virtual ~PpoPolicyRuntime() = default;
  virtual bool Load(const std::string& policy_path) = 0;
  virtual bool IsLoaded() const = 0;
  virtual const PolicyMetadata& Metadata() const = 0;
  virtual bool Infer(const std::vector<float>& observation, std::vector<float>* action) = 0;
};

}  // namespace tech_1_1
}  // namespace inspection_execution

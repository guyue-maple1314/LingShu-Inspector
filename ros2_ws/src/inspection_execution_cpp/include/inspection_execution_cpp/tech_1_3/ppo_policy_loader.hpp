#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "inspection_execution_cpp/tech_1_1/ppo_runtime_controller.hpp"

namespace inspection_execution {
namespace tech_1_3 {

// C++ 侧读取 Python policy_exporter 写出的 policy_metadata.json 字段名
inline constexpr const char* kMetadataVersion = "version";
inline constexpr const char* kMetadataObsDim = "observation_dim";
inline constexpr const char* kMetadataActDim = "action_dim";
inline constexpr const char* kMetadataPolicyFilename = "policy_filename";
inline constexpr const char* kMetadataJson = "policy_metadata.json";

// PpoPolicyLoaderImpl：实现 tech_1_1::PpoPolicyRuntime 抽象接口
//
//  不虚构推理结果：
// - Load 只校验并解析 policy_metadata.json；如果找不到权重文件或训练框架未接入，
//   IsLoaded 仍为 false，Infer 返回 false，不伪造 action。
// - 权重加载钩子 LoadPolicyWeightsHook 在厂商/框架批准后由子类覆盖。
class PpoPolicyLoaderImpl : public tech_1_1::PpoPolicyRuntime {
 public:
  PpoPolicyLoaderImpl();
  ~PpoPolicyLoaderImpl() override = default;

  // 路径：export_dir = Python policy_exporter 写出的目录，里面含 policy_metadata.json
  bool Load(const std::string& policy_path) override;
  bool IsLoaded() const override;
  const tech_1_1::PolicyMetadata& Metadata() const override;
  bool Infer(const std::vector<float>& observation, std::vector<float>* action) override;

 protected:
  // 子类钩子：权重加载。默认不实现，返回 false — 不虚构推理。
  virtual bool LoadPolicyWeightsHook(const std::string& weight_path);
  virtual bool InferHook(const std::vector<float>& observation, std::vector<float>* action);

 private:
  bool ParseMetadataJson(const std::string& metadata_path);
  bool loaded_;
  tech_1_1::PolicyMetadata metadata_;
  std::string policy_dir_;
  std::string weight_filename_;
};

}  // namespace tech_1_3
}  // namespace inspection_execution

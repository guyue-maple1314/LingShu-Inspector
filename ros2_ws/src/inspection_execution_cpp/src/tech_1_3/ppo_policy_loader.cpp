#include "inspection_execution_cpp/tech_1_3/ppo_policy_loader.hpp"

#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>

namespace inspection_execution {
namespace tech_1_3 {

namespace {

// 极简 JSON 解析器：只提取 policy_metadata.json 顶层字段，不引入第三方库。
// 目的：保证 C++ smoke_test 可无依赖编译。ROS2 运行时若有 nlohmann_json，
// 接入时可以替换该函数实现；接口不变。
bool ExtractTopLevelString(const std::string& json, const std::string& key, std::string* out) {
  const std::string needle = "\"" + key + "\"";
  auto pos = json.find(needle);
  if (pos == std::string::npos) return false;
  pos = json.find(':', pos + needle.size());
  if (pos == std::string::npos) return false;
  ++pos;
  while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t')) ++pos;
  if (pos >= json.size()) return false;
  if (json[pos] == '"') {
    // string
    ++pos;
    std::ostringstream oss;
    while (pos < json.size() && json[pos] != '"') {
      if (json[pos] == '\\' && pos + 1 < json.size()) {
        oss << json[++pos];
      } else {
        oss << json[pos];
      }
      ++pos;
    }
    if (pos >= json.size()) return false;
    *out = oss.str();
    return true;
  }
  // number
  std::ostringstream oss;
  while (pos < json.size() && json[pos] != ',' && json[pos] != '}' && json[pos] != ' ' &&
         json[pos] != '\n' && json[pos] != '\r' && json[pos] != '\t') {
    oss << json[pos];
    ++pos;
  }
  *out = oss.str();
  return !out->empty();
}

bool ReadFile(const std::string& path, std::string* out) {
  std::ifstream f(path);
  if (!f.is_open()) return false;
  std::ostringstream ss;
  ss << f.rdbuf();
  *out = ss.str();
  return true;
}

}  // namespace

PpoPolicyLoaderImpl::PpoPolicyLoaderImpl() : loaded_(false) {}

bool PpoPolicyLoaderImpl::ParseMetadataJson(const std::string& metadata_path) {
  std::string raw;
  if (!ReadFile(metadata_path, &raw)) return false;
  std::string s_ver, s_ob, s_ac, s_file;
  if (!ExtractTopLevelString(raw, kMetadataVersion, &s_ver)) return false;
  if (!ExtractTopLevelString(raw, kMetadataObsDim, &s_ob)) return false;
  if (!ExtractTopLevelString(raw, kMetadataActDim, &s_ac)) return false;
  ExtractTopLevelString(raw, kMetadataPolicyFilename, &s_file);  // 可选

  try {
    metadata_.version = s_ver;
    metadata_.observation_dim = static_cast<std::uint32_t>(std::stoul(s_ob));
    metadata_.action_dim = static_cast<std::uint32_t>(std::stoul(s_ac));
  } catch (...) {
    return false;
  }
  weight_filename_ = s_file;
  return metadata_.observation_dim > 0 && metadata_.action_dim > 0;
}

bool PpoPolicyLoaderImpl::Load(const std::string& policy_path) {
  loaded_ = false;
  policy_dir_ = policy_path;
  const std::string metadata_path = policy_path + "/" + kMetadataJson;
  if (!ParseMetadataJson(metadata_path)) return false;
  // 如果有权重文件存在且钩子能加载 → loaded
  if (!weight_filename_.empty()) {
    const std::string weight_path = policy_path + "/" + weight_filename_;
    if (LoadPolicyWeightsHook(weight_path)) {
      loaded_ = true;
    } else {
      loaded_ = false;
    }
  } else {
    // 没权重文件：不虚构，保持 loaded_=false
    loaded_ = false;
  }
  // metadata 在 loaded_=false 时仍可读（便于 observation_builder 配置维度）
  return true;
}

bool PpoPolicyLoaderImpl::IsLoaded() const { return loaded_; }

const tech_1_1::PolicyMetadata& PpoPolicyLoaderImpl::Metadata() const { return metadata_; }

bool PpoPolicyLoaderImpl::Infer(const std::vector<float>& observation, std::vector<float>* action) {
  if (!loaded_ || !action) return false;
  if (observation.size() != metadata_.observation_dim) return false;
  return InferHook(observation, action);
}

bool PpoPolicyLoaderImpl::LoadPolicyWeightsHook(const std::string& /*weight_path*/) {
  // ：训练框架未批准，不加载权重、不虚构结果 → 返回 false。
  return false;
}

bool PpoPolicyLoaderImpl::InferHook(const std::vector<float>& /*observation*/,
                                    std::vector<float>* /*action*/) {
  // ：无后端推理，不伪造 action → 返回 false。
  return false;
}

}  // namespace tech_1_3
}  // namespace inspection_execution

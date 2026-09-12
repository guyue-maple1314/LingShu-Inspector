#pragma once

#include "inspection_execution_cpp/tech_1_9/microphone_synchronizer.hpp"

namespace inspection_execution {
namespace tech_1_9 {

class SpatialFilter {
 public:
  bool Filter(MultiChannelAudio* audio) const;
};

}  // namespace tech_1_9
}  // namespace inspection_execution

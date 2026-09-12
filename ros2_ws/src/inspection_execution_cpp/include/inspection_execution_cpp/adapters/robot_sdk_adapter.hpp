#pragma once

#include <string>
#include <vector>

namespace inspection_execution {

struct JointCommand {
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> torques;
};

struct RobotStateRaw {
  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;
  double battery_level{0.0};
  std::string gait;
  bool connected{false};
};

class RobotSdkAdapter {
 public:
  virtual ~RobotSdkAdapter() = default;
  virtual bool Connect() = 0;
  virtual void Disconnect() = 0;
  virtual bool IsConnected() const = 0;
  virtual bool SendJointCommand(const JointCommand& command) = 0;
  virtual bool ReadState(RobotStateRaw* state) = 0;
};

}  // namespace inspection_execution

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "inspection_execution_cpp/common/node_names.hpp"
#include "inspection_execution_cpp/common/topic_names.hpp"
#include "inspection_execution_cpp/tech_1_9/directional_beamformer.hpp"
#include "inspection_execution_cpp/tech_1_9/microphone_synchronizer.hpp"
#include "inspection_execution_cpp/tech_1_9/snr_estimator.hpp"
#include "inspection_execution_cpp/tech_1_9/spatial_filter.hpp"
#include "inspection_interfaces/msg/acoustic_diagnosis.hpp"

namespace inspection_execution {
namespace tech_1_9 {

constexpr double kPi = 3.14159265358979323846;

class Tech19Node : public rclcpp::Node {
 public:
  Tech19Node()
      : Node(node_names::kTech1_9Node) {
    this->declare_parameter<double>("beam_azimuth_deg", 0.0);
    this->declare_parameter<double>("background_noise_db", 85.0);
    publisher_ = this->create_publisher<inspection_interfaces::msg::AcousticDiagnosis>(
        topic_names::kAcousticDiagnosis, 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() { Tick(); });
    RCLCPP_INFO(this->get_logger(), "%s started (示例音频输入)",
                node_names::kTech1_9Node);
  }

 private:
  MultiChannelAudio MakeDemoAudio() {
    MultiChannelAudio audio;
    audio.sample_rate = 48000;
    for (std::size_t i = 0; i < kNumMicrophones; ++i) {
      audio.channels[i].resize(64);
      for (std::size_t n = 0; n < 64; ++n) {
        audio.channels[i][n] = std::sin(2.0 * kPi * 440.0 * static_cast<double>(n) / 48000.0) *
                                   0.5f +
                               0.01f * static_cast<float>((n + i) % 7);
      }
    }
    return audio;
  }

  void Tick() {
    MultiChannelAudio audio = MakeDemoAudio();
    audio.timestamp_ns = static_cast<std::uint64_t>(this->now().nanoseconds());
    const auto sync = synchronizer_.Synchronize(audio, audio.timestamp_ns);
    if (!sync.valid) {
      return;
    }
    const double azimuth =
        this->get_parameter("beam_azimuth_deg").as_double() * kPi / 180.0;
    const auto beam = beamformer_.Beamform(audio, azimuth);
    if (!beam.valid) {
      return;
    }
    MultiChannelAudio filtered = audio;
    spatial_filter_.Filter(&filtered);
    const double output_snr = snr_estimator_.EstimateDb(beam.mono, audio.channels[0]);

    auto msg = inspection_interfaces::msg::AcousticDiagnosis();
    msg.header.stamp = this->now();
    msg.header.frame_id = "microphone_link";  // 对齐 frames.yaml: microphone_array: microphone_link
    msg.beam_azimuth = this->get_parameter("beam_azimuth_deg").as_double();
    msg.beam_elevation = 0.0;
    msg.background_noise_db = this->get_parameter("background_noise_db").as_double();
    msg.output_snr_db = output_snr;
    msg.fault_class = "";
    msg.confidence = 0.0;
    publisher_->publish(msg);
  }

  MicrophoneSynchronizer synchronizer_;
  DirectionalBeamformer beamformer_;
  SpatialFilter spatial_filter_;
  SnrEstimator snr_estimator_;
  rclcpp::Publisher<inspection_interfaces::msg::AcousticDiagnosis>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tech_1_9
}  // namespace inspection_execution

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<inspection_execution::tech_1_9::Tech19Node>());
  rclcpp::shutdown();
  return 0;
}

"""1.9 声纹故障诊断节点：订阅诊断状态，用抽象模型分类并回填故障类别。"""

import rclpy
from rclpy.node import Node

from inspection_interfaces.msg import AcousticDiagnosis

from inspection_planning_py.common.node_names import ACOUSTIC_FAULT_DIAGNOSIS_NODE
from inspection_planning_py.common.topic_names import ACOUSTIC_DIAGNOSIS
from inspection_planning_py.tech_1_9.acoustic_metrics import AcousticMetrics
from inspection_planning_py.tech_1_9.acoustic_model_loader import FakeAcousticModel


class AcousticFaultDiagnosisNode(Node):
    def __init__(self) -> None:
        super().__init__(ACOUSTIC_FAULT_DIAGNOSIS_NODE)
        self.declare_parameter("model_path", "")
        self._model = FakeAcousticModel()
        model_path = self.get_parameter("model_path").value
        if model_path:
            self._model.load(model_path)
        self._metrics = AcousticMetrics()
        self._publisher = self.create_publisher(AcousticDiagnosis, ACOUSTIC_DIAGNOSIS, 10)
        self.create_subscription(AcousticDiagnosis, ACOUSTIC_DIAGNOSIS, self._on_diagnosis, 10)

    def _on_diagnosis(self, msg: AcousticDiagnosis) -> None:
        self._metrics.record_snr(msg.output_snr_db)
        ok, fault_class, confidence = self._model.infer([])  # 音频占位
        if not ok:
            # 无有效模型结果时不回填（不虚构故障类别）
            self.get_logger().info(
                f"snr={msg.output_snr_db:.1f}dB (no valid model result)"
            )
            return
        out = AcousticDiagnosis()
        out.header = msg.header
        out.beam_azimuth = msg.beam_azimuth
        out.beam_elevation = msg.beam_elevation
        out.background_noise_db = msg.background_noise_db
        out.output_snr_db = msg.output_snr_db
        out.fault_class = fault_class
        out.confidence = confidence
        self._publisher.publish(out)
        self.get_logger().info(
            f"fault={fault_class} confidence={confidence:.2f}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AcousticFaultDiagnosisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

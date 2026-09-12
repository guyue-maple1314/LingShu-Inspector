"""1.9 声纹故障诊断节点：订阅 C++ 侧声学状态，用抽象模型分类并回填故障类别。

边界说明：
- 本节点订阅 /acoustic_diagnosis（C++ 1.9 输出的波束方位 / SNR 状态），
  分类结果回填后仍发布到同一话题；为避免自激循环，只处理 fault_class
  为空的消息（本节点自己发布的分类结果带类别，不会再次触发推理）。
- 音频样本尚未通过 ROS 接口提供（AcousticDiagnosis 只承载方位/SNR），
  因此模型推理当前拿不到波形；未装载模型或拿不到结果时不回填，
  绝不虚构故障类别。音频输入话题确定后再接入。
"""

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

        # 防自激：已带故障类别的消息（含本节点自己发布的）不再回填
        if msg.fault_class:
            return

        # 未装载模型时不做推理（不虚构故障类别）
        if not self._model.is_loaded():
            self.get_logger().debug(
                f"snr={msg.output_snr_db:.1f}dB (no acoustic model loaded)"
            )
            return

        # 音频波形接口未定义，占位传入空样本；模型无结果时不回填
        ok, fault_class, confidence = self._model.infer([])
        if not ok:
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

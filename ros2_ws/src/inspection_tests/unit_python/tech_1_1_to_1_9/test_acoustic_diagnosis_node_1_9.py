"""1.9 声纹诊断节点的节点级逻辑单测（离线，无 ROS 依赖）。

覆盖点：
- 模型推理必须拿到波束形成后的音频样本（不能传空样本）；
- 未装载模型 / 无音频帧 / 音频过期时不推理；
- 已带故障类别的消息不再处理（防同话题自激）。
"""

import unittest
from types import SimpleNamespace

from inspection_planning_py.tech_1_9.acoustic_fault_diagnosis_node import (
    AcousticFaultDiagnosisNode,
    DEFAULT_AUDIO_MAX_AGE_SEC,
)


class FakeModel:
    """可注入的确定性模型替身，记录收到的音频。"""

    def __init__(self, loaded=True, ok=True, fault="bearing_inner", conf=0.82):
        self._loaded = loaded
        self._ok = ok
        self._fault = fault
        self._conf = conf
        self.last_audio = None

    def load(self, model_path):
        self._loaded = bool(model_path)
        return self._loaded

    def is_loaded(self):
        return self._loaded

    def infer(self, audio):
        self.last_audio = list(audio)
        return (self._ok, self._fault, self._conf)


def _audio(samples, sec=10, nanosec=0, valid=True):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
        valid=valid,
        samples=samples,
        sample_rate=48000,
        beam_azimuth=0.0,
        beam_elevation=0.0,
    )


def _diagnosis(fault_class="", sec=10, nanosec=0, snr=18.0):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
        fault_class=fault_class,
        output_snr_db=snr,
        beam_azimuth=0.0,
        beam_elevation=0.0,
        background_noise_db=85.0,
    )


class TestAudioDrivenInference(unittest.TestCase):
    def test_infer_receives_beamformed_audio(self):
        model = FakeModel()
        node = AcousticFaultDiagnosisNode(model=model)
        samples = [0.1 * i for i in range(64)]
        node._on_audio(_audio(samples))
        decision = node._diagnose(_diagnosis(), now_sec=10.0)
        self.assertEqual(decision, ("bearing_inner", 0.82))
        self.assertEqual(model.last_audio, samples)   # 不再传空样本

    def test_no_audio_means_no_inference(self):
        model = FakeModel()
        node = AcousticFaultDiagnosisNode(model=model)
        self.assertIsNone(node._diagnose(_diagnosis(), now_sec=10.0))
        self.assertIsNone(model.last_audio)

    def test_invalid_audio_frame_is_ignored(self):
        model = FakeModel()
        node = AcousticFaultDiagnosisNode(model=model)
        node._on_audio(_audio([0.1, 0.2], valid=False))
        self.assertIsNone(node._diagnose(_diagnosis(), now_sec=10.0))

    def test_stale_audio_is_rejected(self):
        model = FakeModel()
        node = AcousticFaultDiagnosisNode(model=model)
        node._on_audio(_audio([0.1, 0.2], sec=10))
        stale_time = 10.0 + DEFAULT_AUDIO_MAX_AGE_SEC + 1.0
        self.assertIsNone(node._diagnose(_diagnosis(sec=15), now_sec=stale_time))


class TestGuards(unittest.TestCase):
    def test_own_result_is_not_reprocessed(self):
        model = FakeModel()
        node = AcousticFaultDiagnosisNode(model=model)
        node._on_audio(_audio([0.1] * 8))
        # 已带故障类别的消息（含本节点自己发布的）不再推理，避免自激
        self.assertIsNone(
            node._diagnose(_diagnosis(fault_class="bearing_inner"), now_sec=10.0))
        self.assertIsNone(model.last_audio)

    def test_unloaded_model_returns_none(self):
        model = FakeModel(loaded=False)
        node = AcousticFaultDiagnosisNode(model=model)
        node._on_audio(_audio([0.1] * 8))
        self.assertIsNone(node._diagnose(_diagnosis(), now_sec=10.0))

    def test_model_without_result_returns_none(self):
        model = FakeModel(ok=False, fault="", conf=0.0)
        node = AcousticFaultDiagnosisNode(model=model)
        node._on_audio(_audio([0.1] * 8))
        self.assertIsNone(node._diagnose(_diagnosis(), now_sec=10.0))


if __name__ == "__main__":
    unittest.main()

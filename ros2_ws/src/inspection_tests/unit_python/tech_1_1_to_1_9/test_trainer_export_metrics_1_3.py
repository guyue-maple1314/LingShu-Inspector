"""tech_1_3 ppo_training_pipeline + policy_exporter + transfer_metrics 单测。

标准库 unittest。
- AbstractPpoTrainer：用 FakeTrainer 验证模板方法（早停、checkpoint 触发、manifest 写出）
- PolicyExporter：metadata 写出/读回/校验
- TransferMetricsCollector：通行率>90%门槛 + 迁移成功率96.8%门槛
"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from inspection_planning_py.tech_1_3.ppo_training_pipeline import (
    AbstractPpoTrainer, RewardWeights, TrainingChunkResult, TrainingConfig)
from inspection_planning_py.tech_1_3.policy_exporter import (
    METADATA_FILENAME, PolicyExporter, PolicyMetadataForExport)
from inspection_planning_py.tech_1_3.transfer_metrics import (
    EpisodeRecord,
    PPT_PASS_RATE_THRESHOLD,
    PPT_TRANSFER_SUCCESS_THRESHOLD,
    TransferMetricsCollector,
)


class FakeTrainer(AbstractPpoTrainer):
    """最小可测实现：按步长线性提升 pass_rate，不依赖任何训练框架。"""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.prepare_called = False
        self.checkpoints_saved = 0

    def _prepare_backend(self):
        self.prepare_called = True

    def _train_impl_chunk(self, chunk_steps: int):
        # 随 step 提升 pass_rate；训练进度保存在 self.progress
        # 系数调至 20e-6 保证 400k steps 内稳定超过 0.90 早停阈值（PPT 1.3）
        next_pass = min(1.0, 0.5 + self.progress.step * 20e-6)
        return TrainingChunkResult(
            steps_in_chunk=chunk_steps,
            episodes_in_chunk=chunk_steps // 1000,
            mean_reward=-1.0,
            pass_rate=next_pass,
            loss_policy=0.01,
            loss_value=0.5,
        )

    def _save_checkpoint(self, ckpt_dir):
        ckpt_dir.mkdir(parents=True, exist_ok=True)
        (ckpt_dir / "dummy.ckpt").write_text("x", encoding="utf-8")
        self.checkpoints_saved += 1
        return True


class TrainerPipelineTestCase(unittest.TestCase):
    def _make_config(self):
        return TrainingConfig(
            total_timesteps=400_000,
            checkpoint_interval_steps=100_000,
            early_stop_success_rate=PPT_PASS_RATE_THRESHOLD,
            observation_dim=17,
            action_dim=12,
        )

    def test_observation_or_action_zero_raises(self):
        bad = TrainingConfig(observation_dim=0, action_dim=12)
        with self.assertRaises(ValueError):
            FakeTrainer(bad)

    def test_reward_weights_missing_fields_raises(self):
        rw = RewardWeights(weights={"pose_track": 1.0})
        with self.assertRaises(ValueError):
            FakeTrainer(self._make_config(), reward_weights=rw)

    def test_early_stop_when_pass_rate_met(self):
        with tempfile.TemporaryDirectory() as tmp:
            cfg = self._make_config()
            t = FakeTrainer(cfg, output_dir=tmp)
            prog = t.train()
            self.assertTrue(t.prepare_called)
            # 早停条件：pass_rate > 0.90
            self.assertGreater(prog.pass_rate, PPT_PASS_RATE_THRESHOLD)
            self.assertLess(prog.step, cfg.total_timesteps)
            # manifest 写出可读
            mp = t.save_training_manifest()
            self.assertTrue(mp.exists())

    def test_checkpoints_created(self):
        with tempfile.TemporaryDirectory() as tmp:
            cfg = self._make_config()
            # 设阈值 > 1.0（max_pass_rate=1.0）→ 永不早停，完整跑完 400k / 100k = 4 块
            cfg.early_stop_success_rate = 1.001
            t = FakeTrainer(cfg, output_dir=tmp)
            t.train()
            self.assertGreaterEqual(t.checkpoints_saved, 4)


class PolicyExporterTestCase(unittest.TestCase):
    def test_write_and_read_metadata(self):
        with tempfile.TemporaryDirectory() as tmp:
            exporter = PolicyExporter(tmp)
            meta = PolicyMetadataForExport(
                version="1.3.0",
                observation_dim=17,
                action_dim=12,
                policy_filename="policy.onnx",
                training_config_hash="aabbccdd11223344",
                export_timestamp_utc="2026-01-01T00:00:00Z",
                terrain_ids=["grating_base_20m", "corridor_75cm"],
                sim_pass_rate=0.92,
            )
            p = exporter.write_metadata(meta)
            self.assertEqual(p.name, METADATA_FILENAME)
            loaded = exporter.read_metadata()
            self.assertIsNotNone(loaded)
            self.assertEqual(loaded.version, "1.3.0")
            self.assertEqual(loaded.observation_dim, 17)
            self.assertEqual(loaded.action_dim, 12)
            self.assertAlmostEqual(loaded.sim_pass_rate, 0.92)

    def test_export_weights_raises_not_implemented(self):
        with tempfile.TemporaryDirectory() as tmp:
            exporter = PolicyExporter(tmp)
            self.assertFalse(exporter.can_export_weights())
            with self.assertRaises(NotImplementedError):
                exporter.export_weights("any/checkpoint/dir")


class TransferMetricsTestCase(unittest.TestCase):
    def _fill_fake(self, c: TransferMetricsCollector, sim_pass_rate_goal: float,
                   transfer_goal: float):
        # 仿真 1000 条记录，按 sim_pass_rate_goal 控制通过率
        n_sim = 1000
        n_pass_sim = int(n_sim * sim_pass_rate_goal)
        for i in range(n_sim):
            c.record(EpisodeRecord(
                episode_id=f"s{i}", terrain_id="x", terrain_type="grating",
                sim_to_real=False, passed=i < n_pass_sim,
            ))
        # 实机 1000 条，按 transfer_goal 控制迁移成功
        n_real = 1000
        n_ok = int(n_real * transfer_goal)
        for i in range(n_real):
            c.record(EpisodeRecord(
                episode_id=f"r{i}", terrain_id="x", terrain_type="ramp",
                sim_to_real=True, passed=True, transfer_success=i < n_ok,
            ))

    def test_both_metrics_passed(self):
        c = TransferMetricsCollector()
        self._fill_fake(c, 0.95, 0.968)
        self.assertGreater(c.simulation_pass_rate(), PPT_PASS_RATE_THRESHOLD)
        self.assertTrue(c.pass_rate_passed())
        self.assertGreaterEqual(c.transfer_success_rate(), PPT_TRANSFER_SUCCESS_THRESHOLD)
        self.assertTrue(c.transfer_passed())

    def test_both_metrics_failed(self):
        c = TransferMetricsCollector()
        self._fill_fake(c, 0.80, 0.90)
        self.assertFalse(c.pass_rate_passed())
        self.assertFalse(c.transfer_passed())

    def test_summary_dict_keys(self):
        c = TransferMetricsCollector()
        self._fill_fake(c, 0.91, 0.97)
        s = c.summary_dict()
        for k in (
            "total_episodes",
            "simulation_episodes",
            "real_world_episodes",
            "simulation_pass_rate",
            "transfer_success_rate",
            "ppt_thresholds",
            "pass_rate_met",
            "transfer_rate_met",
        ):
            self.assertIn(k, s, f"summary 缺少键 {k}")


if __name__ == "__main__":
    unittest.main()

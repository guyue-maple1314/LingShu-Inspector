"""tech_1_3 terrain_randomizer + sim_to_real_randomization 单测。

标准库 unittest。验证随机化不越出参数硬边界（特别是 1.4 通道宽度不低于 70cm）。
"""

from __future__ import annotations

import unittest

from inspection_planning_py.tech_1_3.terrain_parameterization import (
    TerrainType, build_ppt_terrains)
from inspection_planning_py.tech_1_3.terrain_randomizer import TerrainRandomizer
from inspection_planning_py.tech_1_3.sim_to_real_randomization import (
    DomainRandomizationConfig, SimToRealRandomizer)


class TerrainRandomizerTestCase(unittest.TestCase):
    def test_seeded_reproducible(self):
        r1 = TerrainRandomizer(seed=42)
        r2 = TerrainRandomizer(seed=42)
        base = list(build_ppt_terrains().values())[0]
        a = r1.sample_one(base)
        b = r2.sample_one(base)
        self.assertEqual(a.to_dict(), b.to_dict())

    def test_narrow_corridor_width_not_below_70cm(self):
        r = TerrainRandomizer(seed=1)
        corridor = build_ppt_terrains()["corridor_75cm"]
        for _ in range(100):
            sample = r.sample_one(corridor)
            self.assertGreaterEqual(
                sample.geometry.corridor_width_m, 0.70,
                "1.4 极窄通道随机化后不得低于 70cm，否则不再是窄道场景",
            )

    def test_batch_size(self):
        r = TerrainRandomizer(seed=1)
        batch = r.sample_batch(count_per_type=3)
        self.assertEqual(len(batch), 4 * 3)  # 4 种地形 × 每类 3


class SimToRealRandomizerTestCase(unittest.TestCase):
    def test_seeded_reproducible(self):
        cfg = DomainRandomizationConfig(
            friction_range=(0.9, 1.1),
            latency_ms_range=(2.0, 5.0),
        )
        a = SimToRealRandomizer(cfg, seed=7).sample_batch(5)
        b = SimToRealRandomizer(cfg, seed=7).sample_batch(5)
        self.assertEqual([s.to_dict() for s in a], [s.to_dict() for s in b])

    def test_friction_range_respected(self):
        cfg = DomainRandomizationConfig(friction_range=(0.9, 1.1))
        r = SimToRealRandomizer(cfg, seed=2)
        for s in r.sample_batch(200):
            self.assertGreaterEqual(s.friction_multiplier, 0.9 - 1e-9)
            self.assertLessEqual(s.friction_multiplier, 1.1 + 1e-9)

    def test_latency_positive(self):
        r = SimToRealRandomizer(seed=3)
        for s in r.sample_batch(50):
            self.assertGreater(s.latency_ms, 0.0)


if __name__ == "__main__":
    unittest.main()

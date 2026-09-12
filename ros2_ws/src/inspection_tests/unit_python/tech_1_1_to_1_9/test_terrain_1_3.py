"""tech_1_3 terrain_parameterization 单元测试。

仅用标准库 unittest（不用 pytest）。
PPT 1.3 四项地形：钢格网 / 坡道 / 楼梯 / 窄通道必须齐全。
"""

from __future__ import annotations

import unittest

from inspection_planning_py.tech_1_3.terrain_parameterization import (
    TerrainType, build_ppt_terrains)


class TerrainParameterizationTestCase(unittest.TestCase):
    def setUp(self):
        self.specs = build_ppt_terrains()

    def test_ppt_terrains_count_is_4(self):
        self.assertEqual(len(self.specs), 4, "PPT 1.3 明确四种地形，数量必须为 4")

    def test_terrain_types_covered(self):
        types = {s.terrain_type for s in self.specs.values()}
        required = {
            TerrainType.GRATING,
            TerrainType.RAMP,
            TerrainType.STAIRS,
            TerrainType.NARROW_CORRIDOR,
        }
        self.assertTrue(required.issubset(types), f"缺少 PPT 要求地形：{required - types}")

    def test_grating_velocity_boundary_matches_ppt(self):
        grating = self.specs["grating_base_20m"]
        # PPT 1.5 指标：均速 ≥ 0.8 m/s
        self.assertGreaterEqual(grating.boundary.max_velocity_mps, 0.8)

    def test_narrow_corridor_width_75cm_robot_46cm(self):
        corridor = self.specs["corridor_75cm"]
        # PPT 1.4：机器人宽 46 cm，通道净宽 75 cm
        self.assertAlmostEqual(corridor.geometry.corridor_width_m, 0.75, places=3)
        self.assertAlmostEqual(corridor.boundary.robot_width_m, 0.46, places=3)
        margin_per_side = (
            corridor.geometry.corridor_width_m - corridor.boundary.robot_width_m
        ) / 2
        self.assertLess(margin_per_side, 0.20, "单侧边余量应 < 20cm（PPT 1.4 窄道基线）")

    def test_stairs_step_height_and_count_sane(self):
        stairs = self.specs["stairs_15cm_20"]
        self.assertEqual(stairs.geometry.step_count, 20)
        self.assertAlmostEqual(stairs.geometry.step_height_m, 0.15, places=3)

    def test_spec_to_dict_roundtrip(self):
        for s in self.specs.values():
            d = s.to_dict()
            self.assertIn("terrain_type", d)
            self.assertIn("geometry", d)
            self.assertIn("physics", d)
            self.assertIn("boundary", d)


if __name__ == "__main__":
    unittest.main()

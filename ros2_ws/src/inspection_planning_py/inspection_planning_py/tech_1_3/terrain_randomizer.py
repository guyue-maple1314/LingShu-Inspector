"""地形随机化：基于 TerrainSpec 生成随机地形与边界条件样本

纯离线模块，不调用实机接口。为 Sim-to-Real 域随机化提供结构化随机源。
"""

from __future__ import annotations

import copy
import random
from dataclasses import asdict
from typing import Dict, Iterable, List, Optional, Sequence

from .terrain_parameterization import TerrainSpec, TerrainType, build_ppt_terrains


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class TerrainRandomizer:
    """可种子化的地形随机发生器。

    不假定任何训练框架（不依赖 gym / isaac / gazebo）。输出纯 dict
    描述，供具体仿真适配层消费。
    """

    def __init__(self, seed: Optional[int] = None):
        self._rng = random.Random(seed)

    def reseed(self, seed: int) -> None:
        self._rng = random.Random(seed)

    # ---------- 单字段随机化（公开便于 unittest 逐项验证）----------

    def randomize_friction(self, base: float, lo: float = 0.3, hi: float = 0.9) -> float:
        return _clamp(base * self._rng.uniform(lo, hi), lo, hi)

    def randomize_geometry(
        self,
        spec: TerrainSpec,
        length_ratio: Sequence[float] = (0.8, 1.4),
        width_ratio: Sequence[float] = (0.9, 1.1),
        slope_scale: Sequence[float] = (0.7, 1.2),
    ) -> None:
        g = spec.geometry
        g.length_m = max(0.5, g.length_m * self._rng.uniform(*length_ratio))
        g.width_m = max(0.46, g.width_m * self._rng.uniform(*width_ratio))
        if spec.terrain_type == TerrainType.RAMP:
            g.slope_rad = _clamp(g.slope_rad * self._rng.uniform(*slope_scale), 0.0, 0.785)
        if spec.terrain_type == TerrainType.STAIRS:
            g.step_count = max(5, int(g.step_count * self._rng.uniform(0.6, 1.4)))
            g.step_height_m = _clamp(
                g.step_height_m * self._rng.uniform(0.8, 1.2), 0.05, 0.3
            )
        if spec.terrain_type == TerrainType.NARROW_CORRIDOR:
            # PPT 1.4 基线 75cm；随机化不低于 70cm 保证仍是窄道
            g.corridor_width_m = _clamp(
                g.corridor_width_m * self._rng.uniform(0.95, 1.1), 0.70, 1.2
            )
        if spec.terrain_type == TerrainType.GRATING:
            g.grating_bar_spacing_m = _clamp(
                g.grating_bar_spacing_m * self._rng.uniform(0.6, 1.5), 0.01, 0.08
            )

    def randomize_boundary(
        self,
        spec: TerrainSpec,
        velocity_ratio: Sequence[float] = (0.6, 1.1),
        yaw_perturb_rad: Sequence[float] = (-0.087, 0.087),  # ±5°
    ) -> None:
        b = spec.boundary
        b.max_velocity_mps = max(0.1, b.max_velocity_mps * self._rng.uniform(*velocity_ratio))
        dyaw = self._rng.uniform(*yaw_perturb_rad)
        for pose in (b.entry_pose, b.exit_pose):
            pose["yaw"] = float(pose.get("yaw", 0.0)) + dyaw

    # ---------- 对外采样接口 ----------

    def sample_one(self, base_spec: TerrainSpec) -> TerrainSpec:
        spec = copy.deepcopy(base_spec)
        spec.physics.friction_coeff = self.randomize_friction(spec.physics.friction_coeff)
        self.randomize_geometry(spec)
        self.randomize_boundary(spec)
        # 用 rng 生成 16 位后缀，保证相同 seed 可重现（不能用 id(spec)）
        suffix = self._rng.randrange(0, 0x10000)
        spec.terrain_id = f"{base_spec.terrain_id}_rnd_{suffix:04x}"
        return spec

    def sample_batch(
        self,
        base_specs: Optional[Iterable[TerrainSpec]] = None,
        count_per_type: int = 10,
    ) -> List[TerrainSpec]:
        """按地形类型批量采样；默认使用 PPT 四种典型地形作为基准集。"""
        if base_specs is None:
            base_specs = list(build_ppt_terrains().values())
        results: List[TerrainSpec] = []
        for base in base_specs:
            for _ in range(count_per_type):
                results.append(self.sample_one(base))
        return results

    def sample_batch_as_dicts(self, **kwargs) -> List[dict]:
        return [s.to_dict() for s in self.sample_batch(**kwargs)]

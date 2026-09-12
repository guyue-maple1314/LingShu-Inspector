"""Sim-to-Real 域随机化：摩擦 / 负载 / 噪声 / 延时

纯离线模块，不调用实机接口。为 PPT 1.3 的训练域随机化提供可复现、
可统计、可序列化的随机源。
"""

from __future__ import annotations

import random
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Sequence


@dataclass
class DomainRandomizationConfig:
    """四类随机化的范围（PPT 1.3 明确项）。"""

    # 摩擦系数（已含于 terrain_randomizer；这里做执行层叠加）
    friction_range: Sequence[float] = (0.85, 1.15)  # 与地形摩擦相乘的倍率
    # 机身/负载质量倍率（考虑巡检挂载不同）
    payload_mass_ratio_range: Sequence[float] = (1.0, 1.25)
    # 关节执行噪声（标准差倍率）
    joint_noise_std_range: Sequence[float] = (0.0, 0.03)
    # 控制回路延迟 / 观测延迟（毫秒）
    latency_ms_range: Sequence[float] = (1.0, 20.0)


@dataclass
class DomainRandomizationSample:
    """单次采样结果。"""
    friction_multiplier: float = 1.0
    payload_mass_ratio: float = 1.0
    joint_noise_std: float = 0.0
    latency_ms: float = 5.0
    seed: int = 0

    def to_dict(self) -> Dict[str, float]:
        d = asdict(self)
        return {k: float(v) for k, v in d.items()}


class SimToRealRandomizer:
    """Seedable 域随机化发生器。输出纯 dict/数值，不依赖任何仿真框架。"""

    def __init__(
        self,
        config: Optional[DomainRandomizationConfig] = None,
        seed: Optional[int] = None,
    ) -> None:
        self.config = config or DomainRandomizationConfig()
        self._seed = seed if seed is not None else 0xC0FFEE
        self._rng = random.Random(self._seed)

    def reseed(self, seed: int) -> None:
        self._seed = seed
        self._rng = random.Random(seed)

    def sample_one(self, seed: Optional[int] = None) -> DomainRandomizationSample:
        rng = self._rng if seed is None else random.Random(seed)
        c = self.config
        return DomainRandomizationSample(
            friction_multiplier=_rng_uniform(rng, c.friction_range),
            payload_mass_ratio=_rng_uniform(rng, c.payload_mass_ratio_range),
            joint_noise_std=_rng_uniform(rng, c.joint_noise_std_range),
            latency_ms=_rng_uniform(rng, c.latency_ms_range),
            seed=self._seed if seed is None else seed,
        )

    def sample_batch(self, n: int) -> List[DomainRandomizationSample]:
        return [self.sample_one(seed=self._seed + i) for i in range(n)]

    def sample_batch_as_dicts(self, n: int) -> List[Dict[str, float]]:
        return [s.to_dict() for s in self.sample_batch(n)]


def _rng_uniform(rng: random.Random, rng_range: Sequence[float]) -> float:
    lo, hi = float(rng_range[0]), float(rng_range[1])
    if lo > hi:
        lo, hi = hi, lo
    return rng.uniform(lo, hi)

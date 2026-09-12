"""地形参数化（钢格网 / 坡道 / 楼梯 / 窄通道 / 平地）

纯离线模块，不调用实机接口。仅描述 PPT 指定地形的几何、物理参数与边界条件。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional


class TerrainType(str, Enum):
    FLAT = "flat"
    GRATING = "grating"          # 钢格网
    RAMP = "ramp"                # 坡道
    STAIRS = "stairs"            # 楼梯
    NARROW_CORRIDOR = "narrow_corridor"  # 窄通道


@dataclass
class TerrainGeometry:
    """几何描述；单位使用 SI：米 / 弧度。"""
    length_m: float = 0.0
    width_m: float = 0.0
    height_m: float = 0.0
    slope_rad: float = 0.0          # 坡道：俯仰角；楼梯：单阶坡度
    step_count: int = 0             # 楼梯：阶数
    step_height_m: float = 0.0      # 楼梯：单阶高度
    corridor_width_m: float = 0.0   # 窄通道：净宽（PPT 指定 75cm=0.75m）
    grating_bar_spacing_m: float = 0.0  # 钢格网：栅格间距


@dataclass
class TerrainPhysics:
    """物理参数，Sim-to-Real 随机化基于这些字段采样。"""
    friction_coeff: float = 0.8
    restitution: float = 0.0
    stiffness: float = 1e6
    damping: float = 1e3


@dataclass
class TerrainBoundary:
    """边界条件：进入姿态、出口目标、速度/步态约束。"""
    entry_pose: Dict[str, float] = field(default_factory=lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0})
    exit_pose: Dict[str, float] = field(default_factory=lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0})
    max_velocity_mps: float = 1.0   # PPT 1.5 指定 0.8 m/s 以上
    allowed_gaits: List[str] = field(default_factory=lambda: ["trot", "walk"])
    robot_width_m: float = 0.46      # PPT 1.4 指定机器人宽 46 cm


@dataclass
class TerrainSpec:
    """单一地形的完整参数描述。"""
    terrain_id: str
    terrain_type: TerrainType
    geometry: TerrainGeometry = field(default_factory=TerrainGeometry)
    physics: TerrainPhysics = field(default_factory=TerrainPhysics)
    boundary: TerrainBoundary = field(default_factory=TerrainBoundary)
    description: Optional[str] = None

    def to_dict(self) -> dict:
        return {
            "terrain_id": self.terrain_id,
            "terrain_type": self.terrain_type.value,
            "geometry": self.geometry.__dict__,
            "physics": self.physics.__dict__,
            "boundary": self.boundary.__dict__,
            "description": self.description,
        }


def build_ppt_terrains() -> Dict[str, TerrainSpec]:
    """构建 PPT 指定的四个典型地形基准参数。

    训练前可按 terrain_randomizer 对返回值做随机化采样，不要直接把
    本函数参数作为唯一训练集（避免过拟合）。
    """
    specs: Dict[str, TerrainSpec] = {}

    # 1.5 钢格网：长 2000 m 巡检目标（简化基准用 20 m 段）
    grating = TerrainSpec(
        terrain_id="grating_base_20m",
        terrain_type=TerrainType.GRATING,
        geometry=TerrainGeometry(length_m=20.0, width_m=1.5, grating_bar_spacing_m=0.03),
        physics=TerrainPhysics(friction_coeff=0.6, stiffness=5e5),
        boundary=TerrainBoundary(
            max_velocity_mps=0.8,
            allowed_gaits=["trot"],
            exit_pose={"x": 20.0, "y": 0.0, "yaw": 0.0},
        ),
        description="1.5 钢格网基准段，PPT 指标均速≥0.8 m/s，单次 2 km",
    )
    specs[grating.terrain_id] = grating

    # 1.3 坡道：坡度 15°
    ramp = TerrainSpec(
        terrain_id="ramp_15deg",
        terrain_type=TerrainType.RAMP,
        geometry=TerrainGeometry(length_m=5.0, width_m=1.5, slope_rad=0.2618),
        boundary=TerrainBoundary(max_velocity_mps=0.5, allowed_gaits=["trot", "walk"]),
        description="1.3 坡道场景，姿态跟踪奖励关键场景",
    )
    specs[ramp.terrain_id] = ramp

    # 1.3 楼梯：每阶 15cm，20 阶
    stairs = TerrainSpec(
        terrain_id="stairs_15cm_20",
        terrain_type=TerrainType.STAIRS,
        geometry=TerrainGeometry(step_count=20, step_height_m=0.15, length_m=6.0, width_m=1.5),
        boundary=TerrainBoundary(max_velocity_mps=0.3, allowed_gaits=["walk"]),
        description="1.3 楼梯场景，接触估计与速度跟踪奖励关键场景",
    )
    specs[stairs.terrain_id] = stairs

    # 1.4 极窄通道：75 cm 净宽，机器人宽 46 cm（余量仅 14.5 cm / 侧）
    narrow = TerrainSpec(
        terrain_id="corridor_75cm",
        terrain_type=TerrainType.NARROW_CORRIDOR,
        geometry=TerrainGeometry(length_m=10.0, corridor_width_m=0.75),
        boundary=TerrainBoundary(
            max_velocity_mps=0.4,
            robot_width_m=0.46,
            allowed_gaits=["trot"],
            exit_pose={"x": 10.0, "y": 0.0, "yaw": 0.0},
        ),
        description="1.4 极窄通道基准（75cm / 机器人 46cm），侧向余量 < 15cm",
    )
    specs[narrow.terrain_id] = narrow

    return specs

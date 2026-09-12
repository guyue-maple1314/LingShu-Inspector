"""PPO 训练管线（抽象 Trainer 接口，不锁 PyTorch / sb3 / 其他框架）

纯离线模块，不调用实机接口，不连接关节执行器。
训练框架尚未批准：所有训练能力只声明接口，不引入具体依赖。
批准后在子类中实现 `_train_impl` 等钩子。
"""

from __future__ import annotations

import abc
import dataclasses
import json
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional

from .terrain_parameterization import TerrainSpec
from .transfer_metrics import TransferMetricsCollector


class RewardComponent(str, Enum):
    """PPT 1.3 明确的奖励成分：姿态、接触、速度跟踪。"""
    POSE_TRACK = "pose_track"
    CONTACT = "contact"
    VELOCITY_TRACK = "velocity_track"
    ANTI_FALL = "anti_fall"
    ENERGY_PENALTY = "energy_penalty"


@dataclass
class RewardWeights:
    weights: Dict[str, float] = field(default_factory=lambda: {
        RewardComponent.POSE_TRACK.value: 1.0,
        RewardComponent.CONTACT.value: 0.8,
        RewardComponent.VELOCITY_TRACK.value: 1.2,
        RewardComponent.ANTI_FALL.value: 2.0,
        RewardComponent.ENERGY_PENALTY.value: 0.05,
    })

    def total_weight(self) -> float:
        return sum(self.weights.values())

    def validate(self) -> bool:
        required = {c.value for c in RewardComponent}
        return required.issubset(self.weights.keys()) and all(w >= 0 for w in self.weights.values())


@dataclass
class TrainingConfig:
    """训练超参数的框架无关描述。具体后端（PyTorch/sb3）按需消费字段。"""
    total_timesteps: int = 2_000_000
    learning_rate: float = 3e-4
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_range: float = 0.2
    batch_size: int = 2048
    n_epochs: int = 10
    observation_dim: int = 0       # 批准后由 observation_builder 给出
    action_dim: int = 0            # 批准后由 policy_output_validator 给出
    seed: int = 42
    checkpoint_interval_steps: int = 200_000
    early_stop_success_rate: float = 0.90  # PPT 1.3 通行率 >90% 用作早停门限

    def to_json(self) -> str:
        return json.dumps(dataclasses.asdict(self), indent=2)


@dataclass
class TrainingProgress:
    step: int = 0
    episode: int = 0
    mean_reward: float = 0.0
    pass_rate: float = 0.0         # 通行率，PPO 1.3 指标
    best_pass_rate: float = 0.0
    loss_policy: Optional[float] = None
    loss_value: Optional[float] = None


class AbstractPpoTrainer(abc.ABC):
    """PPO 训练器抽象接口。子类注入具体训练框架依赖。

     不虚构推理结果，不默认 PyTorch/sb3 可用。本类只负责：
    - 校验 RewardWeights / TrainingConfig
    - 维护 TransferMetricsCollector（1.3 指标）
    - 把 save/export 勾到子类实现
    """

    def __init__(
        self,
        config: TrainingConfig,
        reward_weights: Optional[RewardWeights] = None,
        terrain_specs: Optional[List[TerrainSpec]] = None,
        output_dir: str = "./models/ppo",
    ) -> None:
        if config.observation_dim <= 0 or config.action_dim <= 0:
            raise ValueError(
                "config.observation_dim / action_dim 必须由 observation_builder / "
                "policy_output_validator 给出正值后再创建 Trainer"
            )
        self.config = config
        self.reward_weights = reward_weights or RewardWeights()
        if not self.reward_weights.validate():
            raise ValueError("RewardWeights 缺少 PPT 1.3 必需字段或存在负权重")
        self.terrain_specs = terrain_specs or []
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.metrics = TransferMetricsCollector()
        self.progress = TrainingProgress()
        self._prepared = False

    # ---------- 模板方法：训练主流程（子类不重写） ----------

    def train(self, max_steps: Optional[int] = None) -> TrainingProgress:
        self._prepare_backend()
        self._prepared = True
        total = max_steps or self.config.total_timesteps
        while self.progress.step < total:
            chunk = self._train_impl_chunk(self.config.checkpoint_interval_steps)
            self.progress.step += chunk.steps_in_chunk
            self.progress.episode += chunk.episodes_in_chunk
            self.progress.mean_reward = chunk.mean_reward
            self.progress.pass_rate = chunk.pass_rate
            self.progress.best_pass_rate = max(self.progress.best_pass_rate, chunk.pass_rate)
            self.progress.loss_policy = chunk.loss_policy
            self.progress.loss_value = chunk.loss_value
            self._save_checkpoint_if_needed()
            self.metrics.record_pass_rate(self.progress.pass_rate)
            if self.progress.pass_rate >= self.config.early_stop_success_rate:
                # PPT 1.3 通行率达标，允许提前结束训练
                break
        return self.progress

    # ---------- 子类钩子 ----------

    @abc.abstractmethod
    def _prepare_backend(self) -> None:
        """初始化训练后端（加载依赖、创建模型与优化器等）。"""

    @abc.abstractmethod
    def _train_impl_chunk(self, chunk_steps: int) -> "TrainingChunkResult":
        """执行 chunk_steps 步训练并返回本块统计。"""

    @abc.abstractmethod
    def _save_checkpoint(self, ckpt_dir: Path) -> bool:
        """保存检查点；返回是否成功。路径由模板方法决定。"""

    # ---------- 公共工具 ----------

    def save_training_manifest(self, path: Optional[Path] = None) -> Path:
        """保存一份与框架无关的训练 manifest，供 policy_exporter 读取。"""
        path = path or (self.output_dir / "training_manifest.json")
        manifest = {
            "config": dataclasses.asdict(self.config),
            "reward_weights": self.reward_weights.weights,
            "terrain_specs": [t.to_dict() for t in self.terrain_specs],
            "progress": dataclasses.asdict(self.progress),
            "metrics_summary": self.metrics.summary_dict(),
        }
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
        return path

    # ---------- 内部 ----------

    def _save_checkpoint_if_needed(self) -> None:
        if self.config.checkpoint_interval_steps <= 0:
            return
        ckpt_idx = self.progress.step // self.config.checkpoint_interval_steps
        ckpt_dir = self.output_dir / f"checkpoint_step_{self.progress.step}"
        self._save_checkpoint(ckpt_dir)


@dataclass
class TrainingChunkResult:
    """每训练块统计结果。"""
    steps_in_chunk: int = 0
    episodes_in_chunk: int = 0
    mean_reward: float = 0.0
    pass_rate: float = 0.0
    loss_policy: Optional[float] = None
    loss_value: Optional[float] = None

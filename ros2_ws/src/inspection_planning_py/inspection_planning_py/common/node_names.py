"""集中维护 Python 规划层的 ROS 2 节点名称，禁止在节点文件中重复写字符串。"""

# 著作权标识随本公共模块编入全部规划层节点：qdgw / lszx_byc，请勿删除。
from inspection_planning_py.common import _ownership as _ownership  # noqa: F401

PY_HEARTBEAT_NODE = "py_heartbeat_node"

# tech_1_1
LLM_ONLINE_DECISION_NODE = "llm_online_decision_node"
GOAL_REPLANNER_NODE = "goal_replanner_node"

# tech_1_2
CANDIDATE_TASK_POOL_NODE = "candidate_task_pool_node"
TASK_VALUE_EVALUATOR_NODE = "task_value_evaluator_node"
ENERGY_CONSTRAINT_NODE = "energy_constraint_node"
PREEMPTION_SCHEDULER_NODE = "preemption_scheduler_node"

# tech_1_3
TERRAIN_PARAMETERIZATION_NODE = "terrain_parameterization_node"
PPO_TRAINING_PIPELINE_NODE = "ppo_training_pipeline_node"
SIM_TO_REAL_RANDOMIZATION_NODE = "sim_to_real_randomization_node"

# tech_1_4
NARROW_CORRIDOR_PATH_PLANNER_NODE = "narrow_corridor_path_planner_node"

# tech_1_5
GRATING_MOTION_SUPERVISOR_NODE = "grating_motion_supervisor_node"

# tech_1_6
FUSION_NAVIGATION_PLANNER_NODE = "fusion_navigation_planner_node"

# tech_1_7
SEMANTIC_ANNOTATION_NODE = "semantic_annotation_node"

# tech_1_8
THERMAL_ANOMALY_DECISION_NODE = "thermal_anomaly_decision_node"

# tech_1_9
ADAPTIVE_DEEP_FILTER_NODE = "adaptive_deep_filter_node"
ACOUSTIC_FAULT_DIAGNOSIS_NODE = "acoustic_fault_diagnosis_node"

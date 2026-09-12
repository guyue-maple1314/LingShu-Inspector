# 接口说明

Topic 用连续状态，Service 用短时查询/校验，Action 用可抢占长任务。

## Topic

| 名称 | 消息类型 | 发布者 | 订阅者 | 频率/触发 | QoS | 技术 |
|---|---|---|---|---|---|---|
| `/robot_state` | `RobotState` | C++ 本体/状态 | Python、JS | 周期 | Sensor Data | 1.1/1.2 |
| `/operator_instruction` | `OperatorInstruction` | JS 指令入口 | Python、C++ | 事件 | Reliable | 1.1 |
| `/inspection_alert` | `InspectionAlert` | C++ 传感器/诊断 | Python、JS | 事件 | Reliable | 1.1/1.7/1.8/1.9 |
| `/structured_goal` | `StructuredGoal` | Python 大模型 | C++ 行为树 | 事件 | Reliable | 1.1 |
| `/task` | `Task` | Python 任务池 | C++、JS | 事件 | Reliable | 1.2 |
| `/energy_constraint` | `EnergyConstraint` | Python 能耗评估 | C++、JS | 事件 | Reliable | 1.2 |
| `/task_decision` | `TaskDecision` | Python 调度器 | C++、JS | 事件 | Reliable | 1.2 |
| `/terrain_observation` | `TerrainObservation` | C++ 感知 | Python、C++ | 周期 | Sensor Data | 1.3 |
| `/grating_status` | `GratingStatus` | C++ 1.5 抑振 | Python、JS | 10 Hz | Sensor Data | 1.5 |
| `/corridor_state` | `CorridorState` | C++ 融合 | Python、JS | 周期 | Sensor Data | 1.4 |
| `/fusion_pose` | `FusionPose` | C++ 定位 | Python、JS | 周期 | Sensor Data | 1.6 |
| `/semantic_alarm` | `SemanticAlarm` | C++ 定位 | JS | 事件 | Reliable | 1.7 |
| `/thermal_measurement` | `ThermalMeasurement` | C++ 补偿 | Python、JS | 30 fps | Sensor Data | 1.8 |
| `/acoustic_diagnosis` | `AcousticDiagnosis` | C++ 1.9 状态 / Python 分类回填 | Python、JS | 事件 | Reliable | 1.9 |
| `/acoustic_mono` | `AcousticFrame` | C++ 1.9 波束形成 | Python 1.9 模型 | 10 Hz | Sensor Data | 1.9 |

原始传感器 Topic（C++ 内部）：`/imu`(1000 Hz)、`/foot_force`(500 Hz)、`/lidar_scan`、`/image`、`/thermal_image`、`/audio_multi_channel`。

### 1.2 安全电量字段（自动返航）

| 消息 | 字段 | 含义 |
|---|---|---|
| `Task` | `estimated_consumption` | 预计任务耗电（%），用于新任务准入 |
| `EnergyConstraint` | `estimated_return_consumption` | 预计返航耗电（%） |
| `EnergyConstraint` | `base_safe_soc_percent` | 固定安全电量基准，默认 30% |
| `EnergyConstraint` | `safe_soc` | 动态安全阈值 `max(30%, 预计返航耗电 + 10 个百分点)` |
| `InspectionAlert` | `pose_valid` | 检测位姿是否有效；`false` 时 1.7 不做位置匹配，保持未定位 |
| `GratingStatus` | `valid` | 是否已有实测数据；`false` 时异常率/速度/里程无意义，不得据此评估 |

`SOC <= safe_soc` 时调度器输出 `TaskDecision.decision = CHARGE`：保存当前任务进度、停止启动新的普通巡检/采集任务并返航充电。30% 是调度层临时阈值，不是 BMS 硬件欠压保护值。

## Service

| 名称 | 类型 | 作用 |
|---|---|---|
| `/validate_goal` | `ValidateGoal` | C++ 行为树执行前校验结构化目标与安全约束 |
| `/resume_task` | `ResumeTask` | 从已保存进度恢复被抢占任务 |

## Action

| 名称 | 类型 | 作用 |
|---|---|---|
| `/execute_task` | `ExecuteTask` | 巡检/异常/采集长任务，含反馈、取消、抢占 |
| `/navigate_goal` | `NavigateGoal` | 动态目标与重规划路径导航（服务端在 `tech_1_6_node`：接受目标→航点执行→反馈位姿与状态→到达/超时结束） |

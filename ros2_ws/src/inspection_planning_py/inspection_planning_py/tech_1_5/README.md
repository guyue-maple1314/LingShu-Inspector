# tech_1_5 钢格网抑振（Python 任务级监督侧）

- grating_motion_supervisor_node：提供钢格网任务级运动目标
- grating_metrics：步态异常率 / 均速 / 单次里程

订阅 `/grating_status`（GratingStatus）取 C++ 侧实测值；`valid=false`（无实测数据）
时不做阈值判定，不虚构速度 / 距离 / 异常率。

不参与 500/1000 Hz MPC 控制。

# tech_1_5 钢格网地面主动抑振（C++，500/1000 Hz 高频闭环）

- foot_contact_estimator：足端力接触判断
- vibration_estimator：足力 + IMU 异常振动估计
- mpc_vibration_controller：MPC 修正足端轨迹 / 关节扭矩
- grating_metrics_recorder：记录步态异常 / 速度 / 里程

状态与实测指标经 `/grating_status`（GratingStatus）以 10 Hz 输出；
未注入适配器时 `valid=false`，上游不得据此评估指标。

禁止 Python / HMI 进入本闭环。

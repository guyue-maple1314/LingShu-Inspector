# 技术 1.8 验收场景（抗振抑扰的动态红外精准测温）

## S1：时间同步（IMU + 红外均有效）
- **输入**：1000Hz IMU 样本 + 30fps 红外帧（时间戳对齐容差 5ms）
- **预期**：`TrySync` 返回有效 packet，含红外帧 + 对齐 IMU
- **验证**：`packet.has_value() == true`

## S2：时间同步（IMU 无效，不发布）
- **输入**：IMU 适配器未注入或 `ReadSample` 失败
- **预期**：`TrySync` 返回 `nullopt`，节点不发布
- **红线**：IMU 无效时不虚构测温结果
- **验证**：`packet.has_value() == false`

## S3：稳像补偿（IMU 角速度有效）
- **输入**：同步 packet + 非零 IMU 角速度
- **预期**：稳像后 `valid == true`，位移补偿量反映角速度
- **验证**：`stabilized.valid == true`

## S4：稳像补偿（IMU/红外无效）
- **输入**：IMU 角速度无效或红外帧无效
- **预期**：`valid == false`，`error_state` 非空
- **红线**：稳像数据无效时不输出虚假补偿温度
- **验证**：`stabilized.valid == false`

## S5：辐射率补偿（物理辐射率）
- **输入**：raw 温度 25℃，辐射率 0.95
- **预期**：`compensated = raw / 0.95^0.25`，`valid == true`
- **验证**：`emissivity_out.valid == true`，补偿温度 > raw

## S6：辐射率补偿（非物理辐射率）
- **输入**：辐射率 0.0 或负值
- **预期**：`valid == false`，`error_state == "invalid"`
- **红线**：非物理辐射率不虚构补偿
- **验证**：`emissivity_out.valid == false`

## S7：角度距离补偿（正常范围）
- **输入**：角度 30°，距离 3m，修正系数 1.0
- **预期**：`compensated = temp / cos(30°) * 1.0`，`valid == true`
- **验证**：`angle_out.valid == true`

## S8：角度距离补偿（cos 防发散）
- **输入**：角度接近 90°（cos→0）
- **预期**：`cos` 下限 0.01 生效，补偿不发散
- **验证**：补偿值有界

## S9：范围校验（角度超 ±60°）
- **输入**：角度 70°
- **预期**：`in_range == false`，`error_state == "angle_out_of_range"`
- **红线**：超范围不按范围内精度发布
- **验证**：`validated.in_range == false`

## S10：范围校验（距离超 1–5m）
- **输入**：距离 6m
- **预期**：`in_range == false`，`error_state == "distance_out_of_range"`
- **验证**：`validated.in_range == false`

## S11：范围校验（辐射率超 (0,1]）
- **输入**：辐射率 1.2
- **预期**：`in_range == false`，`error_state == "emissivity_out_of_range"`
- **验证**：`validated.in_range == false`

## S12：范围校验（范围内全通过）
- **输入**：角度 45°，距离 2m，修正系数 1.02，辐射率 0.95
- **预期**：`in_range == true`，`error_state == "ok"`
- **验证**：`validated.in_range == true`

## S13：节点发布（范围内）
- **输入**：有效 IMU + 红外帧 + 全范围内参数
- **预期**：发布 ThermalMeasurement，`error_state == "ok"`，`compensated_temperature` 非零
- **验证**：`msg.error_state == "ok"`

## S14：节点发布（超范围仍发布但标记）
- **输入**：角度 70°（超范围）
- **预期**：发布 ThermalMeasurement，`error_state == "angle_out_of_range"`，`compensated_temperature` 为补偿值但 `in_range=false`
- **红线**：超范围仍发布数据但明确标记，不按范围内精度发布
- **验证**：`msg.error_state == "angle_out_of_range"`

## S15：PPT 范围内偏差通过（≤0.2℃）
- **输入**：10 次范围内静态采样，最大偏差 0.1℃
- **预期**：`static_pass == true`（0.1 ≤ 0.2）

## S16：PPT 范围内偏差失败（>0.2℃）
- **输入**：偏差 0.5℃ > 0.2℃
- **预期**：`static_pass == false`

## S17：PPT 动态精度通过（≤0.5℃）
- **输入**：10 次范围内动态采样，最大偏差 0.3℃
- **预期**：`dynamic_pass == true`（0.3 ≤ 0.5）

## S18：PPT 动态精度失败（>0.5℃）
- **输入**：偏差 1.0℃ > 0.5℃
- **预期**：`dynamic_pass == false`

## S19：PPT 效率提升通过（≥10 倍）
- **输入**：处理 300fps，基线 30fps → 10x
- **预期**：`efficiency_pass == true`

## S20：PPT 效率提升失败（<10 倍）
- **输入**：处理 150fps，基线 30fps → 5x
- **预期**：`efficiency_pass == false`

## S21：红线 - 超范围样本不计入精度统计
- **输入**：1 个范围内样本（偏差 0.05℃）+ 1 个超范围样本（偏差 75℃）
- **预期**：`out_of_range_count == 1`，`static_deviation_max` 仅为 0.05℃
- **红线**：超范围样本不污染精度统计
- **验证**：`result.static_deviation_max_c <= 0.2`

## S22：异常高温判定（范围内 >80℃）
- **输入**：范围内补偿温度 90℃
- **预期**：发布 InspectionAlert（`alert_type == "thermal_anomaly"`）
- **验证**：异常高温阈值 `ANOMALY_HIGH_TEMP_C == 80.0`

## S23：异常高温判定（超范围不判定，红线）
- **输入**：超范围样本温度 200℃（`error_state != "ok"`）
- **预期**：不做异常高温判定（记录但不发 alert）
- **红线**：超范围不按范围内精度发布，也不触发异常高温
- **验证**：metrics 记录 `out_of_range=1`

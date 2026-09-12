# 技术 1.7 验收场景（高精度时空融合语义地图构建与定位）

## S1：点云转换（位姿有效）
- **输入**：点云 + 有效位姿（非 lost）
- **预期**：点云转换到地图坐标系，`transformed_cloud` 非空，`valid == true`
- **验证**：`transformer.has_valid_transform() == true`

## S2：点云转换（位姿 lost，不输出）
- **输入**：位姿 lost（`pose_valid == false`）
- **预期**：不转换点云，`valid == false`
- **红线**：位姿丢失时不输出虚假点云
- **验证**：`transformer.has_valid_transform() == false`

## S3：栅格赋权（有数据源）
- **输入**：转换后点云 + BIM 要素
- **预期**：栅格被赋权，`valid == true`，权重落在 [0,1]
- **验证**：`grid.valid() == true`，权重区间合法

## S4：栅格赋权（无数据源）
- **输入**：无点云、无 BIM 要素
- **预期**：`valid == false`，不伪造栅格
- **红线**：无数据源时不输出虚假栅格
- **验证**：`grid.valid() == false`

## S5：BIM-SLAM 关联（30cm 半径内匹配）
- **输入**：SLAM 位姿 (5.000, 3.000)，BIM 要素 (5.020, 3.010)（距离 ≈2.24cm）
- **预期**：匹配成功，`localized == true`，`coordinate_source == "bim_id+slam_id"`
- **验证**：`matcher.last_confidence() > 0.5`

## S6：BIM-SLAM 关联（超出 30cm 半径，未定位）
- **输入**：SLAM 位姿 (5.000, 3.000)，最近 BIM 要素 (6.000, 3.000)（距离 100cm）
- **预期**：`localized == false`，`coordinate_source == "unlocalized"`
- **红线**：无法对应时保持「未定位」，不伪造物理监测点
- **验证**：`alarm.localized == false`

## S7：位置置信度（强匹配）
- **输入**：距离 < 5cm 的强匹配
- **预期**：`confidence ≥ 0.9`
- **验证**：`alarm.confidence >= 0.9`

## S8：位置置信度（弱匹配）
- **输入**：距离 20cm（在 30cm 半径内但偏离）
- **预期**：`confidence` 在 (0.1, 0.9) 之间
- **验证**：位置结果仍带 confidence，不丢失来源标记

## S9：SemanticAlarm 发布（带来源）
- **输入**：检测事件 + 成功关联
- **预期**：发布 SemanticAlarm，含 `bim_id` / `slam_id` / `confidence`
- **验证**：`alarm.bim_id != ""`，`alarm.slam_id != ""`

## S10：BIM 抽象接口（FakeBimLoader 不虚构）
- **输入**：未注入具体 BIM 后端，使用 FakeBimLoader
- **预期**：仅输出确定性 BIM 要素，不虚构额外要素
- **红线**：BIM 格式未获批，不锁具体库，不虚构 BIM 数据
- **验证**：`bim_loader.has_element(id)` 返回确定性结果

## S11：PPT 语义映射精度通过（≤5cm）
- **输入**：10 次匹配采样，最大误差 4cm
- **预期**：`mapping_accuracy_pass == true`（4cm < 5cm）

## S12：PPT 语义映射精度失败（>5cm）
- **输入**：匹配误差 8cm > 5cm
- **预期**：`mapping_accuracy_pass == false`

## S13：PPT 位置报告准确率通过（≥98%）
- **输入**：100 次位置报告，2 次未定位（准确率 98%）
- **预期**：`location_report_accuracy_pass == true`（98% ≥ 98%）

## S14：PPT 位置报告准确率失败（<98%）
- **输入**：100 次位置报告，5 次未定位（准确率 95%）
- **预期**：`location_report_accuracy_pass == false`（95% < 98%）

## S15：退化级联（位姿 lost 时不发布报警）
- **输入**：位姿 lost + 检测事件
- **预期**：不发布 SemanticAlarm（避免基于 lost 位姿输出错误位置）
- **验证**：`alarm.localized == false`，且不伪造位置

## S16：BIM-SLAM 关联半径边界
- **输入**：距离恰好 30cm
- **预期**：`localized == true`（边界内含），`confidence` 较低但仍有效
- **验证**：`alarm.localized == true`

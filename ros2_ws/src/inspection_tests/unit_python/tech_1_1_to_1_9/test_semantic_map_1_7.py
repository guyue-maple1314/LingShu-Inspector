"""1.7 语义地图构建与定位 Python 单元测试。

覆盖：
- bim_loader：抽象接口 + FakeBimLoader 不虚构
- bim_slam_alignment：30cm 半径关联 + BIM 缺失时全未关联
- semantic_location_metrics：5cm 映射 + 98% 位置报告准确率
- semantic_annotation_node：离线逻辑自检

使用 unittest（不用 pytest，红线）。
"""

import unittest

from inspection_planning_py.tech_1_7.bim_loader import (
    AbstractBimLoader,
    BimElement,
    FakeBimLoader,
)
from inspection_planning_py.tech_1_7.bim_slam_alignment import (
    BimSlamAlignment,
    SlamLandmark,
)
from inspection_planning_py.tech_1_7.semantic_location_metrics import (
    LocationMappingSample,
    SemanticLocationMetrics,
    PPT_MAPPING_ACCURACY_M,
    PPT_LOCATION_REPORT_ACCURACY,
)


class TestBimLoader(unittest.TestCase):
    """BIM 装载抽象接口。"""

    def test_fake_loader_not_initialized_returns_empty(self):
        """未初始化时 load_all 返回空（不虚构）。"""
        loader = FakeBimLoader()
        self.assertFalse(loader.is_ready())
        self.assertEqual(loader.load_all(), [])

    def test_fake_loader_initialized_empty_returns_empty(self):
        """初始化但未注入 BIM 元素 → 返回空（不虚构）。"""
        loader = FakeBimLoader()
        self.assertTrue(loader.initialize())
        self.assertTrue(loader.is_ready())
        self.assertEqual(loader.load_all(), [])

    def test_fake_loader_returns_only_injected_elements(self):
        """仅返回已注入的 BIM 元素。"""
        loader = FakeBimLoader()
        loader.initialize()
        elements = [
            BimElement(id="b1", center_x=1.0, center_y=2.0, prior_weight=0.8),
            BimElement(id="b2", center_x=3.0, center_y=4.0, prior_weight=0.6),
        ]
        loader.set_elements(elements)
        loaded = loader.load_all()
        self.assertEqual(len(loaded), 2)
        self.assertEqual(loaded[0].id, "b1")
        self.assertEqual(loaded[1].id, "b2")

    def test_find_by_id_returns_none_if_not_exist(self):
        loader = FakeBimLoader()
        loader.initialize()
        loader.set_elements([BimElement(id="b1")])
        self.assertIsNone(loader.find_by_id("nonexistent"))
        self.assertEqual(loader.find_by_id("b1").id, "b1")

    def test_abstract_cannot_instantiate(self):
        """抽象接口不能直接实例化。"""
        with self.assertRaises(TypeError):
            AbstractBimLoader()  # type: ignore[abstract]


class TestBimSlamAlignment(unittest.TestCase):
    """BIM-SLAM 关联决策。"""

    def test_no_bim_all_unassociated(self):
        """BIM 缺失时全部未关联（不虚构对应关系）。"""
        aligner = BimSlamAlignment(association_radius_m=0.30)
        landmarks = [SlamLandmark(id="s1", x=1.0, y=2.0)]
        results = aligner.associate([], landmarks)
        self.assertEqual(len(results), 1)
        self.assertFalse(results[0].associated)
        self.assertEqual(results[0].bim_id, "")
        self.assertEqual(results[0].coordinate_source, "slam")

    def test_close_bim_associated(self):
        """30cm 内的 BIM 构件被关联。"""
        aligner = BimSlamAlignment(association_radius_m=0.30)
        bim = [BimElement(id="b1", center_x=1.0, center_y=2.0)]
        landmarks = [SlamLandmark(id="s1", x=1.1, y=2.05)]  # 距离 ~0.11m
        results = aligner.associate(bim, landmarks)
        self.assertTrue(results[0].associated)
        self.assertEqual(results[0].bim_id, "b1")
        self.assertEqual(results[0].coordinate_source, "bim")

    def test_far_bim_not_associated(self):
        """超出半径的 BIM 不被关联（不虚构）。"""
        aligner = BimSlamAlignment(association_radius_m=0.30)
        bim = [BimElement(id="b1", center_x=1.0, center_y=2.0)]
        landmarks = [SlamLandmark(id="s1", x=2.0, y=2.0)]  # 距离 1.0m
        results = aligner.associate(bim, landmarks)
        self.assertFalse(results[0].associated)
        self.assertEqual(results[0].bim_id, "")

    def test_summary_text(self):
        aligner = BimSlamAlignment()
        results = aligner.associate(
            [BimElement(id="b1", center_x=0.0)],
            [SlamLandmark(id="s1", x=0.0)],
        )
        summary = aligner.summary(results)
        self.assertIn("associated", summary)


class TestSemanticLocationMetrics(unittest.TestCase):
    """语义位置指标评估。"""

    def test_empty_run(self):
        metrics = SemanticLocationMetrics()
        metrics.start_run("empty")
        result = metrics.finish_run()
        self.assertEqual(result.sample_count, 0)
        self.assertFalse(result.all_pass)

    def test_all_localized_within_5cm_pass(self):
        """全部正确定位 + 误差 ≤5cm → PASS。"""
        metrics = SemanticLocationMetrics()
        metrics.start_run("perfect")
        for i in range(10):
            metrics.record_sample(LocationMappingSample(
                event_id=f"e{i}",
                matched_x=1.0,
                matched_y=2.0,
                ref_x=1.02,
                ref_y=2.01,  # 误差 ~0.022m < 5cm
                localized=True,
                ref_localized=True,
            ))
        result = metrics.finish_run()
        self.assertEqual(result.sample_count, 10)
        self.assertEqual(result.localized_count, 10)
        self.assertLessEqual(result.mapping_error_max_m, PPT_MAPPING_ACCURACY_M)
        self.assertGreaterEqual(result.location_report_accuracy, PPT_LOCATION_REPORT_ACCURACY)
        self.assertTrue(result.mapping_accuracy_pass)
        self.assertTrue(result.report_accuracy_pass)
        self.assertTrue(result.all_pass)

    def test_unlocalized_when_ref_unlocalized_is_correct(self):
        """真值不应定位且实际未定位 → 正确报告。"""
        metrics = SemanticLocationMetrics()
        metrics.start_run("mixed")
        # 真值应定位 + 实际定位 → 正确
        metrics.record_sample(LocationMappingSample(
            event_id="e1", localized=True, ref_localized=True,
            matched_x=1.0, matched_y=1.0, ref_x=1.0, ref_y=1.0,
        ))
        # 真值不应定位 + 实际未定位 → 正确
        metrics.record_sample(LocationMappingSample(
            event_id="e2", localized=False, ref_localized=False,
        ))
        result = metrics.finish_run()
        self.assertEqual(result.location_report_accuracy, 1.0)
        self.assertTrue(result.report_accuracy_pass)

    def test_false_localized_lowers_accuracy(self):
        """真值不应定位但实际误定位 → 准确率下降。"""
        metrics = SemanticLocationMetrics()
        metrics.start_run("false_positive")
        # 真值应定位 + 实际定位 → 正确
        metrics.record_sample(LocationMappingSample(
            event_id="e1", localized=True, ref_localized=True,
            matched_x=1.0, ref_x=1.0,
        ))
        # 真值不应定位但误定位 → 错误
        metrics.record_sample(LocationMappingSample(
            event_id="e2", localized=True, ref_localized=False,
        ))
        result = metrics.finish_run()
        self.assertLess(result.location_report_accuracy, 1.0)
        self.assertFalse(result.report_accuracy_pass)

    def test_mapping_error_exceeds_5cm_fails(self):
        """映射误差 >5cm → 失败。"""
        metrics = SemanticLocationMetrics()
        metrics.start_run("large_error")
        metrics.record_sample(LocationMappingSample(
            event_id="e1", localized=True, ref_localized=True,
            matched_x=0.0, matched_y=0.0, ref_x=0.1, ref_y=0.0,  # 误差 10cm
        ))
        result = metrics.finish_run()
        self.assertGreater(result.mapping_error_max_m, PPT_MAPPING_ACCURACY_M)
        self.assertFalse(result.mapping_accuracy_pass)

    def test_summary_text_aggregates_runs(self):
        metrics = SemanticLocationMetrics()
        metrics.start_run("r1")
        metrics.record_sample(LocationMappingSample(
            event_id="e1", localized=True, ref_localized=True,
            matched_x=1.0, ref_x=1.0,
        ))
        metrics.finish_run()
        summary = metrics.summary_text()
        self.assertIn("r1", summary)
        self.assertIn("passed PPT thresholds", summary)


class TestSemanticAnnotationNodeOffline(unittest.TestCase):
    """语义标注节点离线逻辑自检（无 ROS 环境）。"""

    def test_offline_main_runs_without_rclpy(self):
        """无 ROS 环境下 main() 应执行离线自检不抛异常。"""
        from inspection_planning_py.tech_1_7 import semantic_annotation_node
        # main() 内部会检测 _HAS_RCLPY 并走离线分支
        semantic_annotation_node.main()

    def test_node_with_fake_loader_does_not_fabricate(self):
        """FakeBimLoader 未注入 BIM 时 load_all 返回空。"""
        loader = FakeBimLoader()
        loader.initialize()
        self.assertEqual(loader.load_all(), [])


if __name__ == "__main__":
    unittest.main()

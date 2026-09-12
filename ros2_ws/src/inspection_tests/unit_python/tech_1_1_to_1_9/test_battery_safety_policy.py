"""安全电量策略单测（1.2）。

覆盖设计方案 §6.2 的全部规则：
- 30% 固定基准（含等号边界）
- safe_soc = max(30%, 预计返航耗电 + 10pp) 的两个文档示例
- SOC <= safe_soc 时返航、保存进度、禁止新任务
- 新任务准入 current - task - return >= 10pp
- decide() 与抢占调度的联动
"""

import unittest

from inspection_planning_py.tech_1_2.battery_safety_policy import (
    DEFAULT_BASE_SAFE_SOC_PERCENT,
    DEFAULT_RETURN_RESERVE_PERCENT_POINTS,
    POLICY_STATUS,
    BatterySafetyPolicy,
)
from inspection_planning_py.tech_1_2.preemption_scheduler import decide


def _task(tid, benefit, risk, urgency, consumption=0.0):
    return {
        "task_id": tid,
        "benefit": benefit,
        "risk": risk,
        "urgency": urgency,
        "estimated_consumption": consumption,
    }


class TestDefaults(unittest.TestCase):
    def test_defaults_match_yaml(self):
        policy = BatterySafetyPolicy()
        self.assertEqual(policy.base_safe_soc_percent, 30.0)
        self.assertEqual(policy.return_reserve_percent_points, 10.0)
        self.assertEqual(DEFAULT_BASE_SAFE_SOC_PERCENT, 30.0)
        self.assertEqual(DEFAULT_RETURN_RESERVE_PERCENT_POINTS, 10.0)
        self.assertEqual(POLICY_STATUS, "provisional_until_real_robot_calibration")

    def test_invalid_params_rejected(self):
        with self.assertRaises(ValueError):
            BatterySafetyPolicy(base_safe_soc_percent=-1.0)
        with self.assertRaises(ValueError):
            BatterySafetyPolicy(base_safe_soc_percent=101.0)
        with self.assertRaises(ValueError):
            BatterySafetyPolicy(return_reserve_percent_points=-0.1)


class TestDynamicSafeSoc(unittest.TestCase):
    def setUp(self):
        self.policy = BatterySafetyPolicy()

    def test_example_return_12(self):
        # 文档示例：预计返航耗电 12% → max(30, 12+10)=30
        self.assertEqual(self.policy.safe_soc(12.0), 30.0)

    def test_example_return_25(self):
        # 文档示例：预计返航耗电 25% → max(30, 25+10)=35
        self.assertEqual(self.policy.safe_soc(25.0), 35.0)

    def test_zero_return_uses_base(self):
        self.assertEqual(self.policy.safe_soc(0.0), 30.0)


class TestReturnBoundary(unittest.TestCase):
    def setUp(self):
        self.policy = BatterySafetyPolicy()

    def test_soc_equal_30_must_return(self):
        # 等号边界：30% <= 30% 必须返航
        self.assertTrue(self.policy.must_return_to_charge(30.0, 0.0))

    def test_soc_just_above_30_stays(self):
        self.assertFalse(self.policy.must_return_to_charge(30.01, 0.0))

    def test_dynamic_threshold_triggers(self):
        # safe_soc=35：32% 虽高于 30 基准仍须返航
        d = self.policy.evaluate(32.0, 0.0, 25.0)
        self.assertTrue(d.must_return)
        self.assertEqual(d.safe_soc, 35.0)
        self.assertTrue(d.save_progress)
        self.assertFalse(d.admit_new_task)

    def test_high_soc_no_return(self):
        d = self.policy.evaluate(80.0, 0.0, 25.0)
        self.assertFalse(d.must_return)
        self.assertFalse(d.save_progress)


class TestTaskAdmission(unittest.TestCase):
    def setUp(self):
        self.policy = BatterySafetyPolicy()

    def test_margin_15_admitted(self):
        # 50 - 10 - 25 = 15 >= 10
        self.assertTrue(self.policy.can_admit_new_task(50.0, 10.0, 25.0))

    def test_margin_9_rejected(self):
        # 50 - 16 - 25 = 9 < 10
        self.assertFalse(self.policy.can_admit_new_task(50.0, 16.0, 25.0))

    def test_margin_exactly_10_admitted(self):
        # 等号边界：45 - 10 - 25 = 10
        self.assertTrue(self.policy.can_admit_new_task(45.0, 10.0, 25.0))

    def test_evaluate_above_safe_but_insufficient_margin(self):
        # 35% > safe_soc(35? return=25→safe 35, 35<=35 会返航)；
        # 取 SOC=36：不返航，但 36-20-25 < 0 → 不准入
        d = self.policy.evaluate(36.0, 20.0, 25.0)
        self.assertFalse(d.must_return)
        self.assertFalse(d.admit_new_task)


class TestCustomPolicy(unittest.TestCase):
    def test_custom_base_and_reserve(self):
        policy = BatterySafetyPolicy(
            base_safe_soc_percent=25.0, return_reserve_percent_points=8.0
        )
        self.assertEqual(policy.safe_soc(10.0), 25.0)
        self.assertEqual(policy.safe_soc(20.0), 28.0)


class TestSchedulerIntegration(unittest.TestCase):
    def test_charge_at_30_boundary(self):
        d = decide([_task("a", 10, 0, 0)], "a", battery_percent=30.0,
                   return_distance_m=0.0)
        self.assertEqual(d.decision, "CHARGE")

    def test_charge_legacy_low_battery_still_works(self):
        d = decide([_task("a", 10, 0, 0)], "a", battery_percent=2.0,
                   return_distance_m=100.0)
        self.assertEqual(d.decision, "CHARGE")

    def test_dynamic_threshold_charge_with_return_soc(self):
        # 返航预计耗电 25% → safe_soc=35，SOC=33 → CHARGE
        d = decide([_task("a", 10, 0, 0)], "a", battery_percent=33.0,
                   return_distance_m=0.0, estimated_return_soc=25.0)
        self.assertEqual(d.decision, "CHARGE")

    def test_new_task_blocked_wait_when_margin_insufficient(self):
        # 无在执行任务；36% 高于 safe(30)，但任务耗电 20+返航 20 后余量 -4 → WAIT
        d = decide([_task("a", 10, 0, 0, consumption=20.0)], None,
                   battery_percent=36.0, return_distance_m=0.0,
                   estimated_return_soc=20.0)
        self.assertEqual(d.decision, "WAIT")
        self.assertIsNone(d.current_task_id)

    def test_new_task_executes_when_margin_ok(self):
        d = decide([_task("a", 10, 0, 0, consumption=5.0)], None,
                   battery_percent=80.0, return_distance_m=0.0,
                   estimated_return_soc=20.0)
        self.assertEqual(d.decision, "EXECUTE")
        self.assertEqual(d.current_task_id, "a")

    def test_preemption_blocked_keeps_current(self):
        # 当前任务 a（低分），高价值 b 到达，但准入不足 → 保持 a，不抢占
        d = decide(
            [_task("a", 1, 0, 0, consumption=0.0),
             _task("b", 10, 0, 0, consumption=25.0)],
            "a", battery_percent=36.0, return_distance_m=0.0,
            estimated_return_soc=20.0,
        )
        self.assertEqual(d.decision, "EXECUTE")
        self.assertEqual(d.current_task_id, "a")

    def test_preemption_allowed_when_margin_ok(self):
        d = decide(
            [_task("a", 1, 0, 0), _task("b", 10, 0, 0, consumption=5.0)],
            "a", battery_percent=80.0, return_distance_m=0.0,
            estimated_return_soc=20.0,
        )
        self.assertEqual(d.decision, "PREEMPT")
        self.assertEqual(d.current_task_id, "b")


if __name__ == "__main__":
    unittest.main()

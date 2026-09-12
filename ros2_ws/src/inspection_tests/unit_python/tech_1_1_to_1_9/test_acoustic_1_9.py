import unittest

from inspection_planning_py.tech_1_9.acoustic_metrics import AcousticMetrics
from inspection_planning_py.tech_1_9.acoustic_model_loader import FakeAcousticModel


class TestAcousticMetrics(unittest.TestCase):
    def test_snr_target(self):
        metrics = AcousticMetrics()
        metrics.record_snr(16.0)
        metrics.record_snr(18.0)
        self.assertEqual(metrics.mean_snr_db(), 17.0)
        self.assertTrue(metrics.snr_in_target_range())

    def test_accuracy(self):
        metrics = AcousticMetrics()
        metrics.record_prediction("bearing", "bearing")
        metrics.record_prediction("gear", "bearing")
        metrics.record_prediction("bearing", "bearing")
        metrics.record_prediction("bearing", "bearing")
        self.assertAlmostEqual(metrics.accuracy(), 0.75)
        self.assertTrue(metrics.accuracy_passed())


class TestFakeAcousticModel(unittest.TestCase):
    def test_not_fabricate(self):
        model = FakeAcousticModel()
        self.assertFalse(model.is_loaded())
        ok, category, _ = model.infer([1.0, 2.0])
        self.assertFalse(ok)
        self.assertEqual(category, "")

    def test_load(self):
        model = FakeAcousticModel()
        self.assertTrue(model.load("/path/to/model"))
        self.assertTrue(model.is_loaded())


if __name__ == "__main__":
    unittest.main()

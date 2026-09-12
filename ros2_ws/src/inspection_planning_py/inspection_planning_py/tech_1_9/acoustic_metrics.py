"""1.9 统计 85dB 下的 SNR 与 75% 准确率。"""

from typing import List

PPT_TARGET_SNR_MIN_DB = 15.0
PPT_TARGET_SNR_MAX_DB = 20.0
PPT_ACCURACY_THRESHOLD = 0.75
PPT_BACKGROUND_NOISE_DB = 85.0


class AcousticMetrics:
    def __init__(self) -> None:
        self._snr_samples: List[float] = []
        self._predictions = 0
        self._correct = 0

    def record_snr(self, snr_db: float) -> None:
        self._snr_samples.append(float(snr_db))

    def record_prediction(self, predicted: str, ground_truth: str) -> None:
        self._predictions += 1
        if predicted == ground_truth:
            self._correct += 1

    def mean_snr_db(self) -> float:
        if not self._snr_samples:
            return 0.0
        return sum(self._snr_samples) / len(self._snr_samples)

    def snr_in_target_range(self) -> bool:
        mean = self.mean_snr_db()
        return PPT_TARGET_SNR_MIN_DB <= mean <= PPT_TARGET_SNR_MAX_DB

    def accuracy(self) -> float:
        if not self._predictions:
            return 0.0
        return self._correct / self._predictions

    def accuracy_passed(self) -> bool:
        return self.accuracy() >= PPT_ACCURACY_THRESHOLD

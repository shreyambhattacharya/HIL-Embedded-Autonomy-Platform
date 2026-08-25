import sys
import unittest
from pathlib import Path


TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))

from run_hardware_validation import validation_decision  # noqa: E402
from run_timing_characterization import counter_delta, timing_counter_delta, uint32_delta  # noqa: E402


class TimingToolTests(unittest.TestCase):
    def test_uint32_delta_wraps(self):
        self.assertEqual(uint32_delta(1, 0xFFFFFFFF), 2)
        self.assertEqual(uint32_delta(12, 12), 0)

    def test_counter_delta_keeps_absolute_values_separate(self):
        baseline = {"rx_decode": 2, "rx_gaps": 7}
        final = {"rx_decode": 2, "rx_gaps": 9}
        delta = counter_delta(final, baseline)
        self.assertEqual(delta["rx_decode"], 0)
        self.assertEqual(delta["rx_gaps"], 2)

    def test_timing_deadline_delta_wraps(self):
        baseline = {"deadline_misses": 0xFFFFFFFF}
        final = {"deadline_misses": 0}
        self.assertEqual(timing_counter_delta(final, baseline)["deadline_misses"], 1)


class ValidationDecisionTests(unittest.TestCase):
    def test_required_pass_ignores_experimental_failure(self):
        cases = [
            {"required": True, "status": "complete"},
            {"required": False, "status": "fail"},
        ]
        self.assertEqual(validation_decision(cases), "PASS")

    def test_required_failure_is_fail(self):
        self.assertEqual(validation_decision([{"required": True, "status": "fail"}]), "FAIL")

    def test_required_not_run_is_not_run(self):
        self.assertEqual(validation_decision([{"required": True, "status": "not_run"}]), "NOT_RUN")

    def test_empty_or_partial_required_set_is_not_run(self):
        self.assertEqual(validation_decision([]), "NOT_RUN")
        self.assertEqual(validation_decision([
            {"required": True, "status": "complete"},
            {"required": True, "status": "not_run"},
        ]), "NOT_RUN")


if __name__ == "__main__":
    unittest.main()


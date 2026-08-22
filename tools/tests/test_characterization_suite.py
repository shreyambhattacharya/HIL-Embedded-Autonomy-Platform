#!/usr/bin/env python3
"""Unit tests for pure repeatability analysis and schema handling."""

import importlib.util
from pathlib import Path
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "run_characterization_suite.py"
SPEC = importlib.util.spec_from_file_location("characterization_suite", MODULE_PATH)
SUITE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SUITE)


def valid_document(result="PASS"):
    return {
        "schema_version": 1,
        "run_metadata": {},
        "topics": {},
        "controller_timing": {},
        "controller_budget": {},
        "response_latency_ms": {},
        "stationary": {},
        "result": result,
        "failures": [],
    }


class PercentileTests(unittest.TestCase):
    def test_nearest_rank_is_deterministic(self):
        values = list(range(1, 11))
        self.assertEqual(SUITE.percentile(values, 0.5), 5)
        self.assertEqual(SUITE.percentile(values, 0.95), 10)

    def test_empty_percentile_is_none(self):
        self.assertIsNone(SUITE.percentile([], 0.95))


class AggregationTests(unittest.TestCase):
    def test_aggregate_contains_across_run_statistics(self):
        summary = SUITE.aggregate_values([1.0, 2.0, 3.0, 4.0])
        self.assertEqual(summary["samples"], 4)
        self.assertEqual(summary["mean"], 2.5)
        self.assertEqual(summary["median"], 2.5)
        self.assertAlmostEqual(summary["standard_deviation"], 1.2909944487)
        self.assertEqual(summary["minimum"], 1.0)
        self.assertEqual(summary["maximum"], 4.0)
        self.assertEqual(summary["p95_across_runs"], 4.0)

    def test_missing_and_nonfinite_values_are_excluded(self):
        summary = SUITE.aggregate_values([None, float("nan"), 3.0])
        self.assertEqual(summary["samples"], 1)
        self.assertEqual(summary["mean"], 3.0)
        self.assertEqual(summary["standard_deviation"], 0.0)


class SchemaTests(unittest.TestCase):
    def test_valid_schema_is_accepted(self):
        document = valid_document()
        self.assertIs(SUITE.validate_run_document(document), document)

    def test_missing_schema_field_is_rejected(self):
        document = valid_document()
        del document["controller_timing"]
        with self.assertRaisesRegex(ValueError, "controller_timing"):
            SUITE.validate_run_document(document)

    def test_unknown_result_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "result"):
            SUITE.validate_run_document(valid_document("UNKNOWN"))


class ResultInterpretationTests(unittest.TestCase):
    def test_pass_requires_zero_process_status(self):
        self.assertEqual(SUITE.classify_run(valid_document("PASS"), 0), "passed")
        self.assertEqual(SUITE.classify_run(valid_document("PASS"), 1), "invalid")

    def test_regression_failure_is_preserved(self):
        self.assertEqual(SUITE.classify_run(valid_document("FAIL"), 1), "failed")

    def test_invalid_run_is_preserved(self):
        self.assertEqual(SUITE.classify_run(valid_document("INVALID"), 1), "invalid")


if __name__ == "__main__":
    unittest.main()

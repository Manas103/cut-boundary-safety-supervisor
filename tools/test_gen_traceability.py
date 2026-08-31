#!/usr/bin/env python3
"""Black-box tests for tools/gen_traceability.py itself (REQ-021): runs the
real script as a subprocess against small fixture requirements/trace files
and checks its exit code and generated markdown, the same discipline
flight-software-test-harness's tests/test_gen_release_manifest.py uses for
its own tool. Plain stdlib unittest, no pytest dependency, run standalone:

    python3 tools/test_gen_traceability.py -v
"""
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parent / "gen_traceability.py"

REQUIREMENTS_CSV = """id,description,component
REQ-A,"first requirement",Comp
REQ-B,"second requirement",Comp
"""


class GenTraceabilityTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.requirements_path = self.dir / "requirements.csv"
        self.requirements_path.write_text(REQUIREMENTS_CSV)
        self.out_path = self.dir / "matrix.md"

    def tearDown(self):
        self.tmp.cleanup()

    def run_generator(self, trace_paths):
        args = [sys.executable, str(SCRIPT)]
        for t in trace_paths:
            args += ["--trace", str(t)]
        args += ["--requirements", str(self.requirements_path), "--out", str(self.out_path)]
        return subprocess.run(args, capture_output=True, text=True)

    def write_trace(self, name, records):
        p = self.dir / name
        p.write_text(json.dumps(records))
        return p

    def test_all_covered_and_passing_exits_zero(self):
        trace = self.write_trace("trace.json", [
            {"test": "t1", "passed": True, "requirements": ["REQ-A"]},
            {"test": "t2", "passed": True, "requirements": ["REQ-B"]},
        ])
        result = self.run_generator([trace])
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        content = self.out_path.read_text()
        self.assertIn("REQ-A", content)
        self.assertIn("None, every requirement has at least one passing covering test.", content)

    def test_one_uncovered_requirement_exits_nonzero(self):
        trace = self.write_trace("trace.json", [
            {"test": "t1", "passed": True, "requirements": ["REQ-A"]},
        ])
        result = self.run_generator([trace])
        self.assertEqual(result.returncode, 1)
        self.assertIn("REQ-B", result.stdout)
        content = self.out_path.read_text()
        self.assertIn("- REQ-B", content)

    def test_covering_test_failed_exits_nonzero(self):
        trace = self.write_trace("trace.json", [
            {"test": "t1", "passed": True, "requirements": ["REQ-A"]},
            {"test": "t2", "passed": False, "requirements": ["REQ-B"]},
        ])
        result = self.run_generator([trace])
        self.assertEqual(result.returncode, 1)
        content = self.out_path.read_text()
        self.assertIn("| REQ-B |", content)
        self.assertIn("FAIL", content)

    def test_merges_multiple_trace_files(self):
        trace1 = self.write_trace("trace1.json", [
            {"test": "gtest.t1", "passed": True, "requirements": ["REQ-A"]},
        ])
        trace2 = self.write_trace("trace2.json", [
            {"test": "e2e.scenario", "passed": True, "requirements": ["REQ-B"]},
        ])
        result = self.run_generator([trace1, trace2])
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("2 trace file(s)", result.stdout)

    def test_untraced_test_is_reported_but_does_not_fail_the_gate(self):
        trace = self.write_trace("trace.json", [
            {"test": "t1", "passed": True, "requirements": ["REQ-A"]},
            {"test": "t2", "passed": True, "requirements": ["REQ-B"]},
            {"test": "sanity_check", "passed": True, "requirements": []},
        ])
        result = self.run_generator([trace])
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        content = self.out_path.read_text()
        self.assertIn("- sanity_check", content)


if __name__ == "__main__":
    unittest.main()

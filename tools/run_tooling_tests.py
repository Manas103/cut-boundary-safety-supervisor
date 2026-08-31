#!/usr/bin/env python3
"""Runs tools/test_gen_traceability.py and, on success, records REQ-021 as
covered in a trace JSON in the same schema the GoogleTest listener and the
end-to-end drivers use, so tools/gen_traceability.py's own merge step can
pick it up. This is the meta-test for the traceability tool itself; nothing
else in this repository exercises REQ-021.

Usage:
    python3 tools/run_tooling_tests.py --out build/tooling_trace.json
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    result = subprocess.run(
        [sys.executable, str(HERE / "test_gen_traceability.py"), "-v"],
        capture_output=True, text=True,
    )
    print(result.stdout)
    print(result.stderr, file=sys.stderr)
    passed = result.returncode == 0

    trace = [{"test": "tools.test_gen_traceability", "passed": passed, "requirements": ["REQ-021"]}]
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(trace, indent=2))

    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""End-to-end driver: 12 fault-injection cases (4 tracker dropout, 4 stale
pose, 4 latency spike), each run through a fresh commander/supervisor/
actuator trio, measuring real wall-clock time from fault-condition onset to
the supervisor issuing a hold command (REQ-011, REQ-012, REQ-013, REQ-014,
REQ-016).

What "fault condition onset" means, precisely, per case (see README
"Findings" / "What each metric measures" for the full rationale): the
earliest instant the supervisor could have known about the fault, not the
instant the fault was injected. For dropout that is the moment silence
first crosses the configured dropout window (not the moment the last pose
was sent); for stale/latency it is the moment the offending pose message is
fully received (that is the earliest instant its age is knowable at all).
The supervisor computes and logs this exactly (fault_onset_wall_ms,
hold_issued_wall_ms, hold_ms) using its own single clock domain
(std::chrono::steady_clock in the same process), so this driver reads the
number the supervisor itself measured rather than re-deriving it from
cross-process timestamps.

Usage:
    python3 driver/run_fault_injection.py --build-dir build \
        --boundary config/boundary.txt --out-dir build/e2e_fault --docs-dir docs
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import start_with_port, start_plain, wait_clean, read_all_stderr  # noqa: E402

DROPOUT_MS = 50.0
STALE_MS = 80.0
LATENCY_MS = 15.0
HOLD_BOUND_MS = 20.0

CASES = (
    [("dropout", f"dropout_{n}", n) for n in (60, 90, 150, 300)]
    + [("stale", f"stale_{n}", n) for n in (90, 120, 200, 500)]
    + [("latency", f"latency_{n}", n) for n in (20, 30, 45, 60)]
)
assert len(CASES) == 12


def build_scenario(kind: str, param: int) -> str:
    if kind == "dropout":
        return f"POSE 50 30 20\nDROPOUT_MS {param}\nSHUTDOWN\n"
    if kind == "stale":
        return f"POSE 50 30 20\nPOSE 50 30 20 STALE_MS={param}\nSHUTDOWN\n"
    if kind == "latency":
        return f"POSE 50 30 20\nPOSE 50 30 20 DELAY_MS={param}\nSHUTDOWN\n"
    raise ValueError(kind)


def parse_decisions(path: Path):
    records = []
    for line in path.read_text().splitlines():
        if not line.startswith("DECISION"):
            continue
        fields = {}
        for tok in line.split(" ")[1:]:
            if "=" not in tok:
                continue
            k, v = tok.split("=", 1)
            fields[k] = v
        records.append(fields)
    return records


def run_one_case(build_dir: Path, boundary: Path, out_dir: Path, name: str, kind: str, param: int):
    case_dir = out_dir / name
    case_dir.mkdir(parents=True, exist_ok=True)
    scenario_path = case_dir / "scenario.txt"
    scenario_path.write_text(build_scenario(kind, param))

    actuator_log = case_dir / "actuator.log"
    decisions_log = case_dir / "decisions.log"
    commander_log = case_dir / "commander.log"

    actuator = start_with_port([str(build_dir / "actuator"), f"--log={actuator_log}"], cwd=build_dir, name="actuator")
    supervisor = start_with_port(
        [
            str(build_dir / "supervisor"),
            f"--actuator-port={actuator.port}",
            f"--boundary={boundary}",
            f"--decisions-log={decisions_log}",
            f"--stale-ms={STALE_MS}",
            f"--latency-ms={LATENCY_MS}",
            f"--dropout-ms={DROPOUT_MS}",
            "--poll-ms=1",
        ],
        cwd=build_dir,
        name="supervisor",
    )
    commander = start_plain(
        [str(build_dir / "commander"), f"--supervisor-port={supervisor.port}", f"--scenario={scenario_path}",
         f"--log={commander_log}"],
        cwd=build_dir,
        name="commander",
    )

    # A dropout case's commander deliberately sleeps `param` ms before
    # sending SHUTDOWN; every other process must still be waited for with a
    # margin above that.
    timeout = max(15.0, param / 1000.0 + 10.0)
    rc = [wait_clean(commander, timeout), wait_clean(supervisor, timeout), wait_clean(actuator, timeout)]

    stderrs = {s.name: read_all_stderr(s) for s in (commander, supervisor, actuator)}

    records = parse_decisions(decisions_log)
    fault_req = {"dropout": "REQ-011", "stale": "REQ-012", "latency": "REQ-013"}[kind]
    fault_record = next((r for r in records if r.get("req") == fault_req), None)

    result = {
        "name": name,
        "kind": kind,
        "param_ms": param,
        "return_codes": rc,
        "fault_detected": fault_record is not None,
        "hold_ms": float(fault_record["hold_ms"]) if fault_record else None,
        "requirement_id": fault_req,
    }
    if any(c != 0 for c in rc):
        result["stderr"] = stderrs
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build-dir", required=True)
    ap.add_argument("--boundary", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--docs-dir", required=True)
    args = ap.parse_args()

    build_dir = Path(args.build_dir).resolve()
    boundary = Path(args.boundary).resolve()
    out_dir = Path(args.out_dir).resolve()
    docs_dir = Path(args.docs_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    docs_dir.mkdir(parents=True, exist_ok=True)

    report = []
    results = []
    for kind, name, param in CASES:
        r = run_one_case(build_dir, boundary, out_dir, name, kind, param)
        results.append(r)
        line = (
            f"{name:14s} kind={kind:8s} param_ms={param:5d} "
            f"detected={r['fault_detected']!s:5s} hold_ms={r['hold_ms']}"
        )
        print(line)
        report.append(line)

    hold_times = [r["hold_ms"] for r in results if r["hold_ms"] is not None]
    all_detected = all(r["fault_detected"] for r in results)
    all_under_bound = all(h < HOLD_BOUND_MS for h in hold_times) and len(hold_times) == 12

    summary_lines = [
        "",
        f"cases: {len(results)}",
        f"all faults detected: {all_detected}",
        f"hold times (ms): {hold_times}",
        f"max hold_ms: {max(hold_times) if hold_times else None}",
        f"median hold_ms: {statistics.median(hold_times) if hold_times else None}",
        f"all 12 under {HOLD_BOUND_MS} ms: {all_under_bound}",
    ]
    for line in summary_lines:
        print(line)
    report.extend(summary_lines)

    (docs_dir / "fault_injection_timing.txt").write_text("\n".join(report) + "\n")

    ok = all_detected and all_under_bound
    trace = [
        {
            "test": "e2e.fault_injection_12_cases",
            "passed": ok,
            "requirements": ["REQ-001", "REQ-011", "REQ-012", "REQ-013", "REQ-014", "REQ-016"],
        }
    ]
    (out_dir / "e2e_fault_trace.json").write_text(json.dumps(trace, indent=2))
    (out_dir / "fault_summary.json").write_text(json.dumps({
        "results": results,
        "max_hold_ms": max(hold_times) if hold_times else None,
        "median_hold_ms": statistics.median(hold_times) if hold_times else None,
        "all_detected": all_detected,
        "all_under_bound": all_under_bound,
    }, indent=2))

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

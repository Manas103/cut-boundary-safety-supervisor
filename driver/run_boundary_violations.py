#!/usr/bin/env python3
"""End-to-end driver: launches the real commander/supervisor/actuator
processes and streams 40 deliberately boundary-violating poses interleaved
with 10 safe poses and one deliberately malformed line, through the live
two-process IPC path (REQ-001, REQ-003, REQ-004, REQ-005, REQ-006, REQ-008,
REQ-010, REQ-019, REQ-020).

Usage:
    python3 driver/run_boundary_violations.py --build-dir build \
        --boundary config/boundary.txt --out-dir build/e2e_boundary \
        --docs-dir docs

Exit code is non-zero if any of the checked properties does not hold, so
this can be wired into a real CI gate rather than being a report generator
only.
"""
from __future__ import annotations

import argparse
import json
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import start_with_port, start_plain, wait_clean, read_all_stderr  # noqa: E402


def build_scenario():
    """Returns (lines, expected) where expected maps seq -> dict describing
    what the supervisor should have decided. Faces and depths are chosen to
    vary approach direction (which face) and depth (how far past it) per
    the six axis-aligned faces of the shipped envelope, plus the
    non-axis-aligned cut_plane face."""
    entries = []  # (kind, face_or_none, x, y, z)

    face_generators = {
        "x_min": lambda depth, i: (-depth, 10 + i * 8, 8 + i * 3),
        "x_max": lambda depth, i: (100 + depth, 10 + i * 8, 8 + i * 3),
        "y_min": lambda depth, i: (10 + i * 8, -depth, 8 + i * 3),
        "y_max": lambda depth, i: (10 + i * 8, 60 + depth, 8 + i * 3),
        "z_min": lambda depth, i: (10 + i * 8, 8 + i * 3, -depth),
        "z_max": lambda depth, i: (10 + i * 8, 8 + i * 3, 40 + depth),
    }
    depths = [2, 5, 10, 20, 40, 80]  # varied intrusion depth, shallow to gross
    for face, gen in face_generators.items():
        for i, depth in enumerate(depths):
            x, y, z = gen(depth, i)
            entries.append(("violation", face, x, y, z))

    # cut_plane: inside the box on x/y/z but x+y+z > 160 (0.5*sum > 80).
    cut_plane_points = [
        (100, 60, 5),   # sum 165, margin 2.5
        (100, 60, 20),  # sum 180, margin 10
        (90, 60, 40),   # sum 190, margin 15
        (100, 60, 40),  # sum 200, margin 20 (far corner)
    ]
    for (x, y, z) in cut_plane_points:
        entries.append(("violation", "cut_plane", x, y, z))

    assert sum(1 for e in entries if e[0] == "violation") == 40

    safe_points = [
        (50, 30, 20), (10, 10, 10), (90, 5, 5), (5, 55, 5), (5, 5, 35),
        (60, 40, 25), (20, 20, 20), (80, 50, 10), (30, 45, 30), (95, 55, 5),
    ]
    for (x, y, z) in safe_points:
        entries.append(("safe", None, x, y, z))

    rng = random.Random(42)
    rng.shuffle(entries)

    lines = []
    expected = {}
    seq = 0
    malformed_after = len(entries) // 2
    for idx, (kind, face, x, y, z) in enumerate(entries):
        seq += 1
        lines.append(f"POSE {x} {y} {z}")
        expected[seq] = {"kind": kind, "face": face}
        if idx == malformed_after:
            lines.append("RAW THIS_IS_NOT_A_VALID_MESSAGE 1 2 3")
    lines.append("SHUTDOWN")
    return lines, expected


def parse_decisions_log(path: Path):
    decisions = {}
    malformed_seen = False
    for line in path.read_text().splitlines():
        if not line.startswith("DECISION"):
            continue
        fields = {}
        for tok in line.split(" ")[1:]:
            if "=" not in tok:
                continue
            k, v = tok.split("=", 1)
            fields[k] = v
        if fields.get("tag") == "malformed":
            malformed_seen = True
            continue
        seq = int(fields["seq"])
        decisions[seq] = fields
    return decisions, malformed_seen


def parse_actuator_log(path: Path):
    forwarded_seqs = set()
    hold_seqs = set()
    for line in path.read_text().splitlines():
        if line.startswith("RECV FORWARD"):
            for tok in line.split(" "):
                if tok.startswith("seq="):
                    forwarded_seqs.add(int(tok.split("=", 1)[1]))
        elif line.startswith("RECV HOLD"):
            for tok in line.split(" "):
                if tok.startswith("seq="):
                    s = tok.split("=", 1)[1]
                    if s != "-1":
                        hold_seqs.add(int(s))
    return forwarded_seqs, hold_seqs


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

    lines, expected = build_scenario()
    scenario_path = out_dir / "scenario_boundary.txt"
    scenario_path.write_text("\n".join(lines) + "\n")

    actuator_log = out_dir / "actuator_boundary.log"
    decisions_log = out_dir / "supervisor_boundary_decisions.log"
    commander_log = out_dir / "commander_boundary.log"

    report_lines = []

    def emit(s: str):
        print(s)
        report_lines.append(s)

    emit("== boundary violation end-to-end scenario ==")
    emit(f"scenario file: {scenario_path}")
    emit(f"poses: {len(expected)} (40 violations, 10 safe), plus 1 malformed line")

    actuator = start_with_port(
        [str(build_dir / "actuator"), f"--log={actuator_log}"], cwd=build_dir, name="actuator"
    )
    supervisor = start_with_port(
        [
            str(build_dir / "supervisor"),
            f"--actuator-port={actuator.port}",
            f"--boundary={boundary}",
            f"--decisions-log={decisions_log}",
            "--stale-ms=80",
            "--latency-ms=15",
            "--dropout-ms=50",
            "--poll-ms=1",
        ],
        cwd=build_dir,
        name="supervisor",
    )
    commander = start_plain(
        [
            str(build_dir / "commander"),
            f"--supervisor-port={supervisor.port}",
            f"--scenario={scenario_path}",
            f"--log={commander_log}",
        ],
        cwd=build_dir,
        name="commander",
    )

    emit(f"actuator pid={actuator.proc.pid} port={actuator.port}")
    emit(f"supervisor pid={supervisor.proc.pid} port={supervisor.port} (actuator-port={actuator.port})")
    emit(f"commander pid={commander.proc.pid} (three distinct OS processes -- REQ-001)")

    rc_commander = wait_clean(commander, timeout_s=15.0)
    rc_supervisor = wait_clean(supervisor, timeout_s=15.0)
    rc_actuator = wait_clean(actuator, timeout_s=15.0)
    emit(f"exit codes: commander={rc_commander} supervisor={rc_supervisor} actuator={rc_actuator}")

    ok = True
    if rc_commander != 0 or rc_supervisor != 0 or rc_actuator != 0:
        emit("FAIL: a process exited non-zero")
        for s in (commander, supervisor, actuator):
            err = read_all_stderr(s)
            if err.strip():
                emit(f"  {s.name} stderr: {err.strip()}")
        ok = False

    decisions, malformed_seen = parse_decisions_log(decisions_log)
    forwarded_seqs, hold_seqs = parse_actuator_log(actuator_log)

    violations_total = sum(1 for e in expected.values() if e["kind"] == "violation")
    violations_vetoed_correctly = 0
    violations_leaked = 0
    face_mismatches = []
    for seq, exp in expected.items():
        if exp["kind"] != "violation":
            continue
        d = decisions.get(seq)
        if d is None:
            face_mismatches.append((seq, exp["face"], "no decision logged"))
            continue
        if d.get("req") != "REQ-003":
            face_mismatches.append((seq, exp["face"], f"req={d.get('req')}"))
            continue
        if exp["face"] not in d.get("detail", ""):
            face_mismatches.append((seq, exp["face"], f"detail={d.get('detail')}"))
            continue
        violations_vetoed_correctly += 1
        if seq in forwarded_seqs:
            violations_leaked += 1

    safe_total = sum(1 for e in expected.values() if e["kind"] == "safe")
    safe_accepted = 0
    for seq, exp in expected.items():
        if exp["kind"] != "safe":
            continue
        d = decisions.get(seq)
        if d is not None and d.get("req") == "none" and seq in forwarded_seqs:
            safe_accepted += 1

    emit(f"violations: {violations_total}")
    emit(f"violations vetoed with correct REQ-003 + face name: {violations_vetoed_correctly}/{violations_total}")
    emit(f"violations that leaked to the actuator's received-command log: {violations_leaked}")
    emit(f"safe poses: {safe_total}, accepted and forwarded to actuator: {safe_accepted}/{safe_total}")
    emit(f"malformed line logged and process continued (REQ-019): {malformed_seen}")
    if face_mismatches:
        emit(f"face/requirement mismatches: {face_mismatches}")

    if violations_vetoed_correctly != violations_total:
        ok = False
    if violations_leaked != 0:
        ok = False
    if safe_accepted != safe_total:
        ok = False
    if not malformed_seen:
        ok = False

    emit(f"RESULT: {'PASS' if ok else 'FAIL'}")

    (docs_dir / "boundary_scenario_output.txt").write_text("\n".join(report_lines) + "\n")

    trace = [
        {
            "test": "e2e.boundary_violation_scenario",
            "passed": ok,
            "requirements": [
                "REQ-001", "REQ-003", "REQ-004", "REQ-005", "REQ-006",
                "REQ-008", "REQ-010", "REQ-019", "REQ-020", "REQ-022",
            ],
        }
    ]
    (out_dir / "e2e_boundary_trace.json").write_text(json.dumps(trace, indent=2))

    summary = {
        "violations_total": violations_total,
        "violations_vetoed_correctly": violations_vetoed_correctly,
        "violations_leaked_to_actuator": violations_leaked,
        "safe_total": safe_total,
        "safe_accepted": safe_accepted,
        "malformed_handled": malformed_seen,
        "ok": ok,
    }
    (out_dir / "boundary_summary.json").write_text(json.dumps(summary, indent=2))

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

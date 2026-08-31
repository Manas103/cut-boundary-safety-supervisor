"""Process-lifecycle helpers shared by the two end-to-end driver scripts.

Every process this module starts is a subprocess.Popen object the caller
keeps a direct reference to for the rest of that script's execution; nothing
here ever discovers a PID by scraping ps/tasklist, and nothing here ever
sends a signal to a PID it did not itself receive from Popen. Cleanup always
goes through that same Popen object's own .terminate()/.kill(), and only
after giving the process a chance to exit on its own (every process in this
project is designed to exit cleanly on a SHUTDOWN message propagated from
the commander, so terminate()/kill() here is a safety net, not the normal
path).
"""
from __future__ import annotations

import re
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


@dataclass
class Started:
    proc: subprocess.Popen
    port: Optional[int]
    name: str


def start_with_port(args, cwd: Path, name: str, timeout_s: float = 10.0) -> Started:
    """Starts a process expected to print 'PORT <n>' as its first stdout line."""
    proc = subprocess.Popen(
        args, cwd=str(cwd), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1
    )
    deadline = time.time() + timeout_s
    line = ""
    while time.time() < deadline:
        line = proc.stdout.readline()
        if line:
            break
        if proc.poll() is not None:
            break
    m = re.match(r"PORT (\d+)", line.strip()) if line else None
    if not m:
        stderr = proc.stderr.read() if proc.stderr else ""
        cleanup(proc)
        raise RuntimeError(f"{name}: expected 'PORT <n>' on stdout, got {line!r}. stderr={stderr}")
    return Started(proc=proc, port=int(m.group(1)), name=name)


def start_plain(args, cwd: Path, name: str) -> Started:
    proc = subprocess.Popen(
        args, cwd=str(cwd), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1
    )
    return Started(proc=proc, port=None, name=name)


def wait_clean(started: Started, timeout_s: float = 10.0) -> int:
    """Waits for the held Popen to exit on its own; only escalates to
    terminate()/kill() on that same object if it does not, and always waits
    on the object again after each signal so the PID is confirmed reaped
    before this function returns."""
    proc = started.proc
    try:
        return proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        pass
    proc.terminate()
    try:
        return proc.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        pass
    proc.kill()
    return proc.wait(timeout=5.0)


def cleanup(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            pass


def read_all_stderr(started: Started) -> str:
    try:
        return started.proc.stderr.read() or ""
    except Exception:
        return ""

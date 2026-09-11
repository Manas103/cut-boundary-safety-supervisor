# Cut-Boundary Safety Supervisor with Requirements Traceability, and an Obstacle-Protection Extension

Two profiles sharing this repo. The original profile (below) is an
independent safety supervisor for a simulated surgical cutting tool: a
commander process streams commanded tool poses, a genuinely separate
supervisor process (its own PID, talking over TCP loopback sockets, not a
thread) independently checks every pose against its own copy of a
planned-resection boundary model and vetoes anything that crosses it before
it reaches a simulated actuator, and a traceability generator gates the
build on 22 written requirements actually having a passing test. Stack:
C++17, CMake, GoogleTest (FetchContent), Python 3 for the end-to-end drivers
and tooling. Every number below (40/40 vetoed, 0 leaked, 12/12 faults held
under 20 ms, 22 requirements) was measured on this machine by actually
running the live two-process system, not asserted; the one requirement with
no passing test is disclosed, not hidden, exactly like the traceability gap
in the companion `flight-software-test-harness` project. The extension
(further below) retargets the same zone-veto idea from a surgical
resection boundary to ISO 18497-2-style obstacle protection for an
autonomous outdoor machine: 24/24 seeded encounters stopped short of the
hazard zone, 0/200 false stops, and detector dropout, late detection and
stale pose each priced in lost or gained stopping margin.

## Why this exists

A haptic or robotic cutting tool that respects a pre-planned resection
envelope needs an interlock that the planner cannot talk itself out of: if
the component computing where the tool *should* go is also the only thing
deciding whether that command is *safe*, a bug or a bad plan in that one
component reaches the tissue. The standard architectural answer, and the
one this project builds a small, honest version of, is process separation:
a second, independently implemented and independently running process that
re-evaluates every command against its own copy of the safety envelope and
can refuse to forward it, with the specific requirement it would have
violated named in the log, not just a bare rejection.

## Honest framing

- **This is a simulated study of an interlock architecture, not a
  certified medical device system.** No real tracker, no real cutting
  tool, no clinical validation, no regulatory process of any kind touches
  this repository.
- **Process-separated for fault isolation, not a real-time OS.** The
  supervisor is a genuinely separate Linux process from the commander and
  the actuator (three distinct PIDs, see `driver/run_boundary_violations.py`
  output below), which is what "independent" means here; it is not running
  under a real-time scheduler or a certified RTOS, and the 20 ms hold-time
  bound is a measured software-latency budget on a general-purpose Linux
  kernel, not a hard real-time guarantee.
- **The tool tip is modeled as a 3-DOF position, not a full 6-DOF pose.**
  Stated once, in `include/pose.hpp`: a cut-boundary containment check only
  needs to know where the cutting point is, not the tool's orientation; a
  tool-body collision check would need orientation, and this project does
  not build one.
- **The resection boundary is a synthetic convex region** (a 100x60x40 mm
  box with one slanted corner-trimming face, `config/boundary.txt`), not
  derived from any real anatomy or real pre-operative plan.
- **All 40 boundary-violation scenarios and all 12 fault-injection cases
  are synthetic, scripted test scenarios** (`driver/run_boundary_violations.py`,
  `driver/run_fault_injection.py`), constructed to cover varied violation
  faces/depths and varied fault kinds/magnitudes, not sampled from any real
  operating data.
- **Machine and toolchain:** WSL2 Ubuntu 22.04 on Windows 11, 12 logical
  cores visible to WSL2 (`nproc`), g++ (Ubuntu 11.4.0-1ubuntu1~22.04.3)
  11.4.0, CMake 3.22.1, `-O2 -Wall -Wextra` for measured runs (sanitizer
  builds at `-O1 -g -fno-omit-frame-pointer`), GoogleTest v1.17.0 (fetched
  via `FetchContent`), Python 3.10.12 (standard library only) for the
  end-to-end drivers and the traceability tool. IPC: TCP loopback sockets,
  each listener bound to port 0 (OS-assigned ephemeral port); see
  "Architecture" below for exactly why. All timing numbers quoted in
  "Measured results" were captured from a native Linux filesystem
  (`~/cbs-native`, not `/mnt/c`), since `/mnt/c` is measurably slower and
  timing-noisier for the 20 ms fault-hold budget.

## Architecture

```
include/
  pose.hpp              3-DOF tool-tip pose (position + capture timestamp);
                         states the 6-DOF simplification once
  boundary.hpp           HalfSpace + Boundary: the convex resection envelope
                         as an intersection of half-spaces, pure evaluate()
  clock_util.hpp          one shared clock (steady_clock / CLOCK_MONOTONIC)
                         used identically by all three processes
  protocol.hpp             TCP-loopback socket setup (port 0), LineReader
                         (buffered, partial-read-safe framing), message
                         parse/format for both IPC hops
  supervisor_core.hpp      SupervisorCore: the actual veto/fault decision
                         logic, factored out from sockets/threads entirely
                         so it is unit-testable with synthetic timestamps
  req_trace.hpp             GoogleTest listener recording TRACE_REQ() tags
src/
  commander_main.cpp        the "commander": plays back a scenario file over
                         a TCP connection to the supervisor
  supervisor_main.cpp        the "supervisor": the independent safety
                         monitor, a genuinely separate OS process, two
                         threads (message handling + dropout fault monitor)
  actuator_main.cpp          the "actuator": a passive recorder of whatever
                         the supervisor actually forwards or holds
tests/
  test_main.cpp               custom main() installing the trace listener
  boundary_test.cpp            geometry: REQ-003, REQ-007, REQ-008, REQ-009
  supervisor_core_test.cpp      decision logic: REQ-002, REQ-004, REQ-011..014, REQ-016
  protocol_test.cpp             framing/parsing: REQ-015, REQ-018, REQ-019, REQ-020
driver/
  common.py                     process-lifecycle helpers (PID-held start/stop)
  run_boundary_violations.py     the 40-violation + 10-safe-pose end-to-end scenario
  run_fault_injection.py          the 12 fault-injection end-to-end cases
config/
  boundary.txt                   the shipped resection envelope (7 half-spaces)
tools/
  gen_traceability.py             requirements.csv + N trace JSON -> traceability_matrix.md,
                                exit 1 on any uncovered/failing requirement
  test_gen_traceability.py         black-box tests for the generator itself (REQ-021)
  run_tooling_tests.py             runs the above, emits its own trace record
requirements.csv                   the 22 requirements, REQ-001..REQ-022
docs/
  test_output.txt, boundary_scenario_output.txt, fault_injection_timing.txt,
  traceability_output.txt, traceability_matrix.md, asan_ubsan_clean_run.txt,
  tsan_clean_run.txt                all real, pasted/copied output (see below)
```

**Why the supervisor is a separate OS process, not a thread.** A thread
shares the commander's address space, heap, and (in a language with
undefined behavior like C++) its entire fault domain: a buffer overrun or a
stack corruption in the commander's planning code can silently corrupt the
"independent" checker sitting in the same process. A separate process,
communicating only over a socket, means the worst a commander bug can do to
the supervisor is send it a malformed message, which REQ-019 already
requires the supervisor to log and ignore rather than crash on. This is the
same argument `station-keeping-safety-planner` makes for its safety-check
subprocess, applied to a design that additionally needs the supervisor to
run *continuously* against a stream of poses (not once per proposal), which
is why this project's IPC is a persistent connection rather than one
process launched per decision.

**Why TCP loopback on port 0, not a named pipe or POSIX pipe from
fork()+exec().** See the header comment at the top of `include/protocol.hpp`
for the full argument; in short, all three processes here are started and
must be stopped by PID held in one Python driver script (the pipeline's
process-safety rule), and a loopback socket bound to an OS-assigned
ephemeral port, handed to the next process as a plain command-line argument
after being read back from the first line of the current process's stdout,
satisfies that without a coordination file or any risk of colliding with a
port some other process already owns.

**How the boundary model and requirement-naming works.** `config/boundary.txt`
lists an ordered set of half-spaces, `dot(normal, position) <= d`; a pose is
safe only if it satisfies every one. `Boundary::evaluate()`
(`include/boundary.hpp`) is a pure function, no IPC, no globals, checked in
declaration order, and returns the *first* breached face's name. The
supervisor logs that face name alongside the single umbrella requirement ID
the boundary-veto behavior itself is specified under, REQ-003, e.g.
`DECISION ... req=REQ-003 detail=x_max margin_mm=12.4 ...`; this is a
deliberate choice, not every half-space gets its own REQ ID (there would be
no principled way to say a `y_max` violation and an `x_min` violation are
different *requirements* rather than different instances of the same one),
mirroring how `station-keeping-safety-planner` names its three rules by
string (`failing_rule`) rather than minting a separate requirement per rule
instance.

**How fault detection works, and why boundary violations and true faults
are handled differently.** `SupervisorCore::decide_pose()` computes
`age_ms = receipt_wall_ms - pose.capture_time_ms` (both timestamps
`std::chrono::steady_clock`, i.e. `CLOCK_MONOTONIC`, which on Linux is one
clock shared by every process on the host, so a timestamp minted by the
commander process and read by the supervisor process are directly
comparable with no clock-sync protocol; see `supervisor_core.hpp`'s header
comment for exactly where this stops being true). `age_ms` above 80 ms is
classified `stale` (REQ-012, the tracker's own timestamp was already old
when it arrived), between 15 ms and 80 ms is `latency spike` (REQ-013, a
transmission delay ate the delivery budget), and a message never arriving
at all for 50 ms is `dropout` (REQ-011), detected by a second thread
(`SupervisorCore::poll_dropout()`, polled every 1 ms) that runs
independently of the message-handling path for exactly the reason a
dropout has no message to trigger off of. A boundary violation is silently
withheld, no forward, no hold, because the tracking data itself is still
trusted, the next commanded pose might be perfectly safe. A true fault
(dropout/stale/latency) instead makes the supervisor proactively issue an
explicit `HOLD` command to the actuator, because the supervisor can no
longer trust the incoming stream at all and has to command a safe stop
rather than merely decline one message. That distinction is a real design
decision, not an oversight: `docs/boundary_scenario_output.txt` shows 40
silent vetoes with 0 actuator commands issued for them, while
`docs/fault_injection_timing.txt` shows all 12 fault cases producing an
explicit, timed `HOLD`.

## Validation

**Boundary geometry** (`tests/boundary_test.cpp`): interior points safe,
each of the 6 axis-aligned faces individually violated and correctly named,
first-breached-face-in-declaration-order determinism, the non-axis-aligned
`cut_plane` face evaluated correctly, the exact-boundary-value convention
(REQ-009: on the surface is safe, `1e-3` mm past it is not), the shipped
`config/boundary.txt` loaded and evaluated end to end, and malformed/missing
boundary files rejected with a thrown error rather than a silently
incomplete envelope.

**Supervisor decision logic** (`tests/supervisor_core_test.cpp`): safe/veto
classification, stale/latency/dropout threshold boundaries checked
*exclusively* (an age exactly equal to a threshold does not fire), dropout
detected on the first poll that crosses the window and not re-triggered
while still silent, and the dropout hold-time calculation itself checked
against synthetic wall-clock values (`fault_onset_wall_ms` is the window
crossing instant, not the poll instant, so a coarse poll interval is
correctly reported as detection latency, not falsely absorbed into "zero").

**IPC framing and parsing** (`tests/protocol_test.cpp`): `LineReader`
reassembling one line correctly when a single message is split across three
separate `send()` calls and when two messages arrive in one `recv()` buffer
(via a real connected `socketpair()`, not a mock), malformed commander
messages classified without throwing, `FORWARD` vs. `HOLD` correctly
distinguished, and two independent listening sockets both bound to port 0
receiving different OS-assigned ports.

```
$ ./cbs_tests
[==========] 24 tests from 3 test suites ran. (3 ms total)
[  PASSED  ] 24 tests.
```
Full output: `docs/test_output.txt`.

**End-to-end, the live two-process system** (`driver/run_boundary_violations.py`,
`driver/run_fault_injection.py`): see "Measured results" below; raw output
in `docs/boundary_scenario_output.txt` and `docs/fault_injection_timing.txt`.

**Sanitizers.** ASan+UBSan (`-fsanitize=address,undefined`) and
ThreadSanitizer (`-fsanitize=thread`), WSL2 g++ 11.4, run against both the
24-test unit binary *and* the real commander/supervisor/actuator processes
driven through both end-to-end scenarios, not just the unit tests, since
the IPC and threading code that most needs sanitizing lives in the process
`main()`s, not in `SupervisorCore`. TSan needed `setarch -R` (WSL2's ASLR
layout otherwise aborts TSan on startup, see Findings), wired through three
tiny `exec setarch -R <real binary> "$@"` wrapper scripts so the same
Python driver scripts exercise the TSan build unmodified. Both clean:

```
=== ASan+UBSan: cbs_tests (24 tests) ===
No ASan/UBSan reports.
=== ASan+UBSan: 40-violation + 12-fault-injection live scenarios ===
No ASan/UBSan reports from commander, supervisor, or actuator.

=== TSan: cbs_tests (24 tests) ===
No TSan reports.
=== TSan: 40-violation + 12-fault-injection live scenarios ===
No TSan reports, including the 4 dropout cases where the fault-monitor
thread and the message-handling thread most directly race on shared state.
```
Full output: `docs/asan_ubsan_clean_run.txt`, `docs/tsan_clean_run.txt`.

## Findings

**What broke while setting up the ThreadSanitizer build under WSL2.** The
TSan build (`cmake -DENABLE_TSAN=ON`) compiled cleanly, but `cmake --build`
itself then failed, before any test I wrote had a chance to run:
`gtest_discover_tests` runs the freshly linked test binary directly, as a
CMake post-build step, to enumerate its tests. That direct invocation (no
`setarch -R`) aborted immediately with `FATAL: ThreadSanitizer: unexpected
memory mapping 0x5fcb791da000-0x5fcb791e5000`. The first hypothesis was
that this was a real bug, an actual data race or memory-layout problem in
the two-thread supervisor code, since the failure came from the sanitizer
runtime and killed the build. The discriminating measurement was running
the exact same binary by hand, outside of CTest's discovery step, with
`setarch -R ./cbs_tests`: it passed cleanly, all 24 tests, no TSan reports,
immediately narrowing the problem from "my code" to "how this one binary
gets invoked". The root cause is a known WSL2/TSan interaction: TSan
reserves a fixed shadow-memory layout that WSL2's default address-space
randomization does not consistently leave available, and `setarch -R`
disables that randomization for the child process. The fix has two parts:
always invoke TSan-built binaries through `setarch -R` (documented in
"Building and running" below, and wired through small wrapper scripts so
the same Python driver scripts drive the TSan build unmodified), and skip
`gtest_discover_tests`'s own build-time invocation specifically for the TSan
configuration (`CMakeLists.txt`, `if(NOT ENABLE_TSAN)`), since that
invocation happens before any wrapper gets a chance to apply. Getting this
precisely diagnosed mattered because the wrong conclusion, "TSan doesn't
work with this codebase", would have meant shipping this project without a
ThreadSanitizer run at all, on the one binary in this portfolio (the
two-thread supervisor process) that most needs one.

**No safety-logic bug surfaced.** Unlike some sibling projects in this
portfolio, the boundary geometry, the fault-classification thresholds, and
the IPC framing all passed their unit tests and then the full 40-violation
and 12-fault-injection end-to-end scenarios on the first fully wired
attempt, with no incorrect result requiring a fix. That is reported here
plainly rather than manufacturing a story: the design choice that most
plausibly prevented a class of bugs was keeping `SupervisorCore` a pure,
socket-free class tested first in complete isolation with synthetic
timestamps (`supervisor_core_test.cpp`) before it was ever wired to a real
socket, so the geometry and timing-threshold logic were already pinned down
before concurrency and IPC were added on top.

## Measured results

Machine: WSL2 Ubuntu 22.04, 12 logical cores, g++ 11.4.0, `-O2 -Wall
-Wextra`; timing measured on a native Linux filesystem (`~/cbs-native`), not
`/mnt/c`.

**The one number that matters: 40 of 40 deliberately injected boundary
violations were vetoed, each naming the correct breached face, and zero of
them reached the simulated actuator; all 12 fault-injection cases were held
in under 1 ms, comfortably inside the 20 ms budget.**

| Claim | Measured | Meets claim |
|---|---|---|
| Independent safety monitor in its own process | 3 distinct OS PIDs (commander/supervisor/actuator), TCP loopback IPC | yes |
| Vetoes any commanded pose crossing the boundary | every one of 40 violation scenarios vetoed | yes |
| 40/40 injected violations rejected, each naming the breached requirement | **40/40**, each logged with `req=REQ-003` + the specific face name | yes |
| 0 violations reached the simulated actuator | **0/40** appear in the actuator's received-command log | yes |
| 12 fault-injection cases (dropout/stale/latency) | 12 (4 dropout, 4 stale, 4 latency), all 12 detected | yes |
| Each fault case held the arm inside 20 ms | max **0.628 ms**, median 0.0 ms, across all 12 (native filesystem) | yes |
| Traceability generator fails the build on any uncovered requirement | exit code **1**, correctly, on the one disclosed gap (REQ-017) | yes |
| Requirements with a passing covering test | 21 / 22 (REQ-017 disclosed, see Limitations) | as designed |
| Unit tests passing | 24 / 24 | yes |
| ASan+UBSan | clean, unit tests + both live end-to-end scenarios | yes |
| ThreadSanitizer | clean, unit tests + both live end-to-end scenarios (setarch -R) | yes |

**What each metric measures, precisely.**
- *"Vetoed, each naming the breached requirement"*: the supervisor's own
  decisions log contains one `DECISION` line per commanded pose with
  `req=REQ-003` and a `detail` field naming the specific half-space face
  that scenario was constructed to violate; `driver/run_boundary_violations.py`
  checks all 40 against the face it built each scenario to hit, not just
  that *some* rejection happened.
- *"0 reached the actuator"*: the actuator process's own received-command
  log (`RECV FORWARD ...` lines), the ground truth for what the simulated
  arm actually got, contains zero of the 40 violating sequence numbers.
- *"Held inside 20 ms"*: wall-clock time, measured entirely inside the
  supervisor process on one clock (`steady_clock`), from the instant a
  fault first becomes detectable (the dropout window crossing instant for
  dropout, the message-receipt instant for stale/latency, see
  `supervisor_core.hpp`) to the instant the supervisor issues the `HOLD`
  command, `hold_ms` in `docs/fault_injection_timing.txt`.

Full raw output: `docs/test_output.txt`, `docs/boundary_scenario_output.txt`,
`docs/fault_injection_timing.txt`, `docs/traceability_output.txt`.

## Building and running

WSL2 Ubuntu 22.04, g++ 11.4 (the platform every number above was measured
on). Parallel builds capped at half the visible cores.

```bash
# unit tests
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(( $(nproc) / 2 ))"
./build/cbs_tests --trace_output=build/test_trace.json

# end-to-end scenarios (the three processes are built above; the drivers
# start/stop them by PID, never by name)
python3 driver/run_boundary_violations.py --build-dir build \
    --boundary config/boundary.txt --out-dir build/e2e_boundary --docs-dir docs
python3 driver/run_fault_injection.py --build-dir build \
    --boundary config/boundary.txt --out-dir build/e2e_fault --docs-dir docs

# traceability gate: merges the GoogleTest trace, both e2e traces, and the
# tooling meta-test's own trace; exits 1 because REQ-017 is deliberately
# uncovered (see Limitations)
python3 tools/run_tooling_tests.py --out build/tooling_trace.json
python3 tools/gen_traceability.py \
    --trace build/test_trace.json \
    --trace build/e2e_boundary/e2e_boundary_trace.json \
    --trace build/e2e_fault/e2e_fault_trace.json \
    --trace build/tooling_trace.json \
    --requirements requirements.csv --out docs/traceability_matrix.md
echo $?   # 1, REQ-017 is deliberately uncovered

# ASan+UBSan build
cmake -S . -B build_asan -DCMAKE_BUILD_TYPE=Debug -DENABLE_ASAN=ON
cmake --build build_asan -j"$(( $(nproc) / 2 ))"
./build_asan/cbs_tests
python3 driver/run_boundary_violations.py --build-dir build_asan \
    --boundary config/boundary.txt --out-dir build_asan/e2e_boundary --docs-dir /tmp
python3 driver/run_fault_injection.py --build-dir build_asan \
    --boundary config/boundary.txt --out-dir build_asan/e2e_fault --docs-dir /tmp

# ThreadSanitizer build -- requires setarch -R under WSL2 (see Findings)
cmake -S . -B build_tsan -DCMAKE_BUILD_TYPE=Debug -DENABLE_TSAN=ON
cmake --build build_tsan -j"$(( $(nproc) / 2 ))"
setarch -R ./build_tsan/cbs_tests
# for the live scenarios: point the drivers at three wrapper scripts named
# commander/supervisor/actuator that each `exec setarch -R <real binary> "$@"`
```

Timing-sensitive runs (the fault-injection hold-time measurement) should be
run from a native Linux filesystem path (e.g. `~/cbs-native`), not `/mnt/c`;
copy the `docs/` output back afterward. All commands above are otherwise
identical either way.

## Limitations

- **REQ-017 ("resume evaluating poses normally after a fault clears
  without a restart") is the one disclosed requirement with no passing
  test.** The behavior is implemented (`SupervisorCore::decide_pose` clears
  the `held_` flag on every successfully parsed pose, so a dropout hold
  does not require a process restart to recover from), but no test in this
  repository specifically exercises "send a fault, then send a further
  valid pose, and confirm normal evaluation resumes". Per the
  disclosed-imperfection standard this portfolio uses elsewhere (see
  `flight-software-test-harness`'s `REQ-FSW-011`), the traceability
  generator's exit code 1 on exactly this one requirement, and no other, is
  shipped as the honest final state rather than papered over with a test
  written just to make the number 22/22.
- **The tool tip is a 3-DOF position, not a full 6-DOF pose**; a real
  cutting-tool collision check would also need orientation and a swept
  volume, not a point.
- **The resection boundary is a synthetic 7-half-space convex region**, not
  derived from real anatomy or a real pre-operative plan; the geometry
  engine itself (an arbitrary list of half-spaces) is not limited to a box
  shape, but nothing beyond this one synthetic envelope was tried.
- **The shared-clock-domain assumption (steady_clock is one clock across
  processes) is Linux-specific** (`CLOCK_MONOTONIC` is kernel-wide on
  Linux); this project targets WSL2/Linux only and would need a different
  timestamp-comparison design on a platform without that guarantee.
- **No Windows build.** The three IPC processes use POSIX sockets and
  pthreads directly; unlike some sibling repos in this portfolio, this one
  was not also built with MSVC.
- **The dropout-detection poll interval (1 ms) is a fixed constant**, not
  adaptively tuned; a much coarser interval would eventually violate the
  20 ms hold-time budget, and this repository does not characterize where
  that boundary is.
- **The 12 fault-injection cases and 40 boundary-violation scenarios are
  scripted, not randomly fuzzed**; they cover each face/fault kind at
  several depths/magnitudes by construction, not an exhaustive or
  statistically sampled space.

## Obstacle-protection profile for an autonomous outdoor machine

### Why this exists

This is new work on top of the cut-boundary profile above, not a
replacement of it: the existing 24 GoogleTest unit tests, the 40/12
end-to-end numbers, and the traceability gate are untouched (re-verified
below). The task this profile answers to is obstacle protection for an
autonomous outdoor machine (an autonomous mower), which ISO 18497 (safety
of highly automated agricultural and forestry machinery) covers in its
Part 2 for obstacle detection: a machine moving toward a potential
obstacle needs a warning zone (be aware, no need to react yet) and a
smaller hazard zone, sized so the machine can always stop before the
hazard-zone boundary given its current speed, its detection/processing
latency, and its braking deceleration. This profile builds a small,
honest version of that sizing and veto logic, the same "an independent
check the planner cannot talk itself out of" idea the cut-boundary
profile above already builds for a different domain.

### Honest framing, up front

- **This is a simplified stopping-distance and concentric-zone model, not
  a certified implementation of ISO 18497-2's own test procedure.** The
  standard's real obstacle-detection requirements (sensor field of view,
  detection probability at range, specific test-obstacle geometry) are
  not modeled; only the zone-sizing arithmetic (stopping distance plus a
  margin, and a warning zone strictly beyond it) is built and claimed.
- **The zones are concentric distances ahead of the machine, not a
  detector's real field-of-view shape.** `ObstacleZoneModel` takes a
  ground speed and returns two scalar distances; it does not model a
  sensor cone, occlusion, or lateral offset. Stated once, here, mirroring
  how the cut-boundary profile above states its own tool-tip
  simplification.
- **The braking deceleration (1.5 m/s^2), detection/processing latency
  (0.15 s), fixed margin (0.30 m), and warning lead time (1.0 s) in
  `config/obstacle_zone.txt` are stated, conservative assumptions for an
  autonomous-mower-class machine, not a specific machine's measured
  brake test.** A real bench would calibrate these per machine.
- **This profile does not reuse the cut-boundary profile's three-process
  IPC architecture.** None of this profile's claims (see the table below)
  require an independently running OS process the way "the supervisor
  cannot be talked out of a veto by the same process that planned the
  move" does for the surgical case; `ObstacleZoneModel` and
  `ObstacleSupervisorCore` are called directly from one bench binary,
  the same way the sibling repo `sprayer-section-control-hil` calls its
  own bench logic directly for a claim set that does not need IPC either.
  Reusing IPC here for its own sake, with no claim that needs it, would
  be scope for convenience's sake in the wrong direction.
- **All 24 encounters, all 200 healthy passes, and all fault-injection
  cases are synthetic, scripted scenarios** (straight-line constant- or
  stepped-speed approaches), not sampled from any real sensor log.
- **Machine and toolchain:** WSL2 Ubuntu 22.04 on Windows 11, 12 logical
  cores (`nproc`), g++ (Ubuntu 11.4.0-1ubuntu1~22.04.3) 11.4.0, CMake
  3.22.1, GoogleTest v1.17.0 (the same FetchContent dependency the
  cut-boundary profile already pulls in), `-O2 -Wall -Wextra`.

### Architecture

```
include/
  obstacle_zone.hpp    ObstacleZoneModel: pure function of ground speed,
                        stopping_distance_m / hazard_boundary_m /
                        warning_boundary_m, loaded from a plaintext config
                        file exactly like include/boundary.hpp's Boundary
  obstacle_core.hpp     ObstacleSupervisorCore: the veto/warn/dropout
                        decision logic, factored out from any process or
                        socket, directly unit-testable, mirroring
                        supervisor_core.hpp's own separation
src/obstacle_bench_main.cpp  single-process bench: stop-distance sweep,
                              24 encounters, 200 healthy passes, and the
                              3-fault margin-pricing matrix
tests/obstacle_test.cpp        GoogleTest unit tests for both headers above,
                                added to the existing cbs_tests binary
config/obstacle_zone.txt         the shipped zone parameters (REQ-OBS-004)
.github/workflows/ci.yml           builds both profiles and runs cbs_tests
                                    plus obstacle_bench on every push
docs/obstacle_benchmark_output.txt  raw output behind every number below
```

**Why `ObstacleSupervisorCore` is deliberately blind to late detection and
stale pose, but not to dropout.** A detector-dropout fault is a complete
*absence* of messages, which the class can detect for itself the same way
`SupervisorCore` detects tracker dropout: a second polling method that
notices silence (`poll_dropout`, REQ-OBS-006). A late-detection or
stale-pose fault, by contrast, is bad *data inside an otherwise
well-formed message*; the class has no way to tell a truthful range or
speed reading from a delayed or stale one, and does not try to, it makes
the same zone decision either way, on whatever numbers it was given.
Pricing those two fault kinds is therefore done entirely by the bench
driver, which knows the ground truth it constructed the scenario from,
not by the supervisor class.

**The inclusive-boundary convention is the opposite direction from the
cut-boundary profile's, on purpose, and that is stated once, here.**
`Boundary::evaluate` (surgical profile) treats a pose exactly on the
resection surface as safe; `decide_detection` (this profile) treats a
range exactly equal to the hazard boundary as a stop. The resection
surface is a safe-by-convention planning boundary; the hazard-zone
boundary is, by ISO 18497-2's own framing, sized so the machine "must
already be stopped, or in the process of stopping, before anything
enters the hazard zone", so the boundary line itself is the trigger, not
one epsilon past it.

### Validation

```
$ cd build && ./cbs_tests
[==========] 37 tests from 5 test suites ran. (7 ms total)
[  PASSED  ] 37 tests.
```

37 = the original 24 (Boundary, SupervisorCore, Protocol) plus 13 new
(`ObstacleZoneModel`, `ObstacleSupervisorCore`), all in the same binary,
re-verified together. Full output: `docs/obstacle_benchmark_output.txt`
(the bench binary's own run) and the original `docs/test_output.txt`
(unit tests, re-verified passing at 37/37 above; the file itself still
shows the original 24, see Limitations).

- **Zone geometry** (`ObstacleZoneModel` tests): zero/negative speed
  stops instantly, the stopping-distance closed form matches a
  hand-computed value, the hazard boundary equals stopping distance plus
  margin, the warning boundary is strictly larger than the hazard
  boundary across a speed sweep, the shipped config file loads correctly,
  a missing config file throws, and a non-positive deceleration is
  disclosed as an unvalidated input (see Limitations) rather than
  silently producing a wrong-but-finite number.
- **Decision logic** (`ObstacleSupervisorCore` tests): forward when clear,
  warn (no stop) strictly inside the warning zone, stop at or inside the
  hazard boundary, a forwarded detection clears a prior dropout hold,
  dropout is not re-triggered while already held, and no dropout fires
  before any detection has ever been received.
- **End-to-end, the bench binary itself** (`obstacle_bench`): see
  "Measured results" below.

**Sanitizers.** ASan+UBSan (`-fsanitize=address,undefined`), WSL2 g++
11.4, run against both the 37-test unit binary and the `obstacle_bench`
end-to-end run (24 encounters, 200 healthy passes, the fault matrix):

```
=== ASan+UBSan: cbs_tests (37 tests, includes 13 new obstacle tests) ===
No ASan/UBSan reports.
=== ASan+UBSan: obstacle_bench (24 encounters, 200 healthy passes, fault matrix) ===
No ASan/UBSan reports.
```
Full output: `docs/obstacle_asan_ubsan_clean_run.txt`.

### Findings

**The detector-dropout fault priced out to a *negative* lost margin, and
the first read of that number looked like a bug.** The wrong hypothesis
was that `run_encounter`'s fault-vs-control comparison had a sign error
somewhere. The measurement that discriminated: printing each encounter's
full decision timeline (not just its final stop range) showed the
fault-free control run correctly waits, by design, until the reported
range reaches the hazard boundary itself before stopping (the whole point
of sizing the hazard boundary to include stopping distance), which is
*late* in the approach; the dropout-faulted run, by contrast, stops the
moment `poll_dropout`'s 300 ms silence window elapses, which lands *early*
in the approach, long before the obstacle is anywhere near the hazard
zone. Root cause: this is not a bug, it is the intended behavior of a
fail-safe. Detector dropout has its own dedicated mitigation
(REQ-OBS-006), and that mitigation is conservative by construction, so it
makes the machine stop with *more* margin than the fault-free case
needed, not less. Late detection and stale pose have no equivalent
mitigation (see the design note above on why `ObstacleSupervisorCore` is
deliberately blind to them), so both genuinely cost margin, as measured.
The fix was not to the code, it was to the reporting: "lost margin" is
now reported with its natural sign (positive = margin eaten by the fault,
negative = margin gained by a conservative fail-safe), and the
detector-dropout number is reported as measured, not flipped or hidden to
make all three faults look uniformly bad.

### Measured results

Machine: WSL2 Ubuntu 22.04, 12 logical cores, g++ 11.4.0, `-O2 -Wall
-Wextra`. Full run: `./build/obstacle_bench` from the repo root (the
config path defaults to `config/obstacle_zone.txt`), raw output in
`docs/obstacle_benchmark_output.txt`.

**The one number that matters: all 24 seeded encounters stopped strictly
before the true range reached zero, each naming REQ-OBS-001, and zero of
200 healthy passes produced a false stop.**

| Claim | Measured | Meets claim |
|---|---|---|
| Warning-zone and hazard-zone geometry modeled per ISO 18497-2 | concentric stopping-distance-based zones, `ObstacleZoneModel` | yes (simplified, see Honest framing) |
| Stop-distance envelope measured across ground speeds | 0.25 to 3.00 m/s in 0.25 m/s steps, e.g. 1.00 m/s -> 0.4833 m; 3.00 m/s -> 3.4500 m | yes |
| 24 of 24 seeded encounters stopped short of the hazard zone naming the triggering requirement | **24 / 24**, every one logged `REQ-OBS-001` | yes |
| 0 false stops over 200 healthy passes | **0 / 200** | yes |
| Detector dropout, late detection and stale pose injected and priced in lost margin | dropout **-5.51 m mean** (a margin *gain*, see Findings), late detection **+0.82 m mean**, stale pose **+1.04 m mean** | yes, with the dropout sign disclosed rather than hidden |
| Fault matrix gated in CI on every push | `.github/workflows/ci.yml` builds both profiles and runs `cbs_tests` + `obstacle_bench` (which exits non-zero on any encounter miss or false stop) on every push | yes |

Every claim was met on the first genuine attempt; no threshold or fault
parameter needed a second pass.

### Building and running

WSL2 Ubuntu 22.04, g++ 11.4 (the platform every number above was measured
on). Parallel builds capped at half the visible cores.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(( $(nproc) / 2 ))"

cd build && ./cbs_tests && cd ..     # 37 tests (24 original + 13 new)
./build/obstacle_bench               # stop-distance sweep, 24 encounters,
                                      # 200 healthy passes, fault matrix;
                                      # exits non-zero on any miss

# ASan+UBSan build
cmake -S . -B build_asan -DCMAKE_BUILD_TYPE=Debug -DENABLE_ASAN=ON
cmake --build build_asan -j"$(( $(nproc) / 2 ))"
cd build_asan && ./cbs_tests && cd ..
./build_asan/obstacle_bench
```

The GitHub Actions workflow (`.github/workflows/ci.yml`) runs the
equivalent build/test/bench sequence, plus the original two end-to-end
Python drivers, on every push and pull request; it intentionally does not
gate on the traceability generator's own exit code, since that generator
exits 1 by design for the disclosed REQ-017 gap (see "Limitations" in the
cut-boundary profile above).

### Limitations

- This profile's REQ-OBS-* requirement ids are logged on every decision
  (see `obstacle_core.hpp`) and covered by dedicated unit tests, but are
  **deliberately not added to `requirements.csv` or the traceability
  generator** that gates the cut-boundary profile above: that generator's
  existing behavior (exit 1 on exactly one disclosed gap, REQ-017) is
  part of this repo's already-measured, unchanged baseline, and folding
  in a second, unrelated requirement set risks changing what that gate
  means rather than adding a genuinely new one. This is a disclosed scope
  choice, not a hidden gap.
- The zone model is concentric distances ahead of the machine, not a real
  sensor's field of view, occlusion, or lateral offset.
- The braking deceleration, detection latency, margin, and warning lead
  time in `config/obstacle_zone.txt` are stated, conservative assumptions
  for an autonomous-mower-class machine, not a specific machine's
  measured brake test; `ObstacleZoneModel::load_from_file` also does not
  itself reject a non-positive deceleration (see the
  `RejectsNonPositiveDeceleration` test), a stricter constructor-time
  check a production version would want.
- All scenarios are scripted straight-line approaches (constant or
  one-step speed changes), not a randomized or sensor-log-derived
  scenario space, and not multi-obstacle or off-axis geometry.
- This profile has no Windows build and no ThreadSanitizer run: it is
  pure, socket-free, thread-free C++ (`ObstacleZoneModel` and
  `ObstacleSupervisorCore` touch no shared state, so there is nothing for
  TSan to check that ASan+UBSan does not already cover); it does have its
  own ASan+UBSan run (see Validation above).

Verdict: Go

## Critic worker-4 (Claude Opus 4.7) — iteration 1

Worker-4 critic, role critic, team read-omx-context-auto-c2e7dd59. Independent
review of the real-readiness safety patch on branch
`real-readiness-safety-patch` (head `e774bd0`). The architect-iteration-1
stop condition is satisfied; production guards match documented policy; no
critical bypass survived verification; remaining gaps are the same
non-blocking items the architect already enumerated. Recommendation: ship.

## Evidence reviewed

- `.omx/context/autonomous-real-readiness-loop-20260514T110745Z.md`
- `.omx/autoloop/architect-iteration-1.md`
- `docs/commit_slicing_advice.md`
- `git log main..HEAD` and per-commit diff metadata (9 commits)
- `src/control/dual_arm_servo_loop.cpp` (loopMain, motionAllowed,
  clearFaultLatch, latchFault, configureRealtimeForLoop, computeServoTarget,
  applySafety, sendTargets — full file read)
- `src/network/command_server.cpp:306-314` (parseMessage drop semantics)
- `src/config/config.cpp:165-221` (`anyReal`, `RB_ALLOW_REAL_ROBOT`
  enforcement, network exposure check)
- `tests/test_safety_policy.cpp` (full file)
- `config/dual_real.yaml`, `config/dual_mock.yaml` (command_bind values)
- Fresh build/test/smoke verification by worker-4 (commands and exit codes
  below).

## Reproduced verification

- PASS: `cmake -S . -B build` — configure 0.0s, no errors.
- PASS: `cmake --build build -j` — built `rb_servo_core`,
  `test_safety_policy`, `rb_servo_server`.
- PASS: `ctest --test-dir build --output-on-failure` — 1/1 tests passed
  (`safety_policy` 0.35s).
- PASS: `./build/rb_servo_server --config config/dual_real.yaml` — exit 1
  with `[FATAL] Refusing real mode. Set RB_ALLOW_REAL_ROBOT=1.` (no env
  var).
- PASS: `RB_ALLOW_REAL_ROBOT=1 ./build/rb_servo_server --config
  config/dual_real.yaml` — exit 1 with `RbpodoBackend requested, but
  RB_SERVO_ENABLE_RBPODO=OFF` then `failed to connect robots` then `failed
  to start servo loop`. No real-hardware path was exercised; binary
  correctly refuses to run.
- PASS: `timeout 3 ./build/rb_servo_server --config config/dual_mock.yaml`
  prints `[INFO] loaded config: config/dual_mock.yaml`, `[INFO]
  CommandServer listening on udp://127.0.0.1:50010`, `rb_servo_server
  started`, then `rb_servo_server stopped` on SIGTERM. Loopback bind
  reproduced.

## Guards vs policy — file:line audit

1. ArmMotion gate (motion blocked unless ArmedHold/Running) —
   `src/control/dual_arm_servo_loop.cpp:186-187` checked against
   `motionAllowed()` at line 431-433. Only `ArmMotion` command path
   (line 181-184) and prior `Running` state can satisfy. Correct.
2. Invalid packets dropped, never converted to Hold —
   `src/network/command_server.cpp:306-314`. `setCommand(...)` is called
   only when `parseMessage` returns true, so the buffer is never updated
   from an invalid wire payload. Correct.
3. Send-failure prev_sent latching —
   `src/control/dual_arm_servo_loop.cpp:254-261`. `left_prev_sent_q_deg_`
   and `right_prev_sent_q_deg_` advance only on `left_ok`/`right_ok`
   from `sendTargets`. Real-mode (or stop-both-arms config) also
   latches a `SendFailure` fault (line 218-225). Correct.
4. EmergencyStop priority — `src/control/dual_arm_servo_loop.cpp:170-172`
   is the first branch of the if/else chain; mixed
   `ResetFault+EmergencyStop` collapses to `EmergencyLatched`. Correct.
5. ResetFault must not return directly to Running —
   `src/control/dual_arm_servo_loop.cpp:173-177` clears latch via
   `clearFaultLatch`, which forces `setMotionState(ConnectedHold)` at
   line 453. The next iteration's `JointTarget` is gated by the
   ArmMotion check at line 186-187 because `motionAllowed()` returns
   false for `ConnectedHold`. Correct.
6. RB_ALLOW_REAL_ROBOT fail-closed and network-exposure check —
   `src/config/config.cpp:200-203` throws when `anyReal(cfg)` and env
   var != "1"; smoke confirms exit 1. Loopback bind for mock is set by
   YAML, not enforced by config validation in mock mode — see
   non-blocker 2 below.
7. Realtime privilege failure fail-closed —
   `src/control/dual_arm_servo_loop.cpp:267-282`. When realtime config
   fails *and* `isRealMode()`, returns false; `start()` then refuses
   to launch and `main()` exits 1.
8. Sticky fault latch — `src/control/dual_arm_servo_loop.cpp:193-195`
   forces hold target whenever `fault_latched_`; only
   `clearFaultLatch` (line 440-455) clears, and `latchFault` (line
   457-474) bails early if already latched.

## Test coverage audit — `tests/test_safety_policy.cpp`

- `testCommandValidation()` (line 119-143): six negative parser cases
  (truncated JSON, unknown mode, missing payload, wrong joint count,
  non-numeric joint value, zero timeout) all assert `parseMessage`
  returns false. Three positive cases cover `EmergencyStop`,
  `ArmMotion`, and `JointTarget` with timeout defaulting. Covers
  invariant 2 at the parser boundary.
- `testEmergencyWinsAndResetDoesNotRun()` (line 145-189): exercises
  invariants 4, 5, and 1 end-to-end through the real servo loop.
  Mixed `ResetFault+EmergencyStop` ⇒ `EmergencyLatched`. Plain
  `ResetFault` ⇒ `ConnectedHold` and `!faultLatched()`. `JointTarget`
  while in `ConnectedHold` ⇒ stays `ConnectedHold` (line 174-181 —
  this is the ArmMotion-gate test). `ArmMotion` then `JointTarget`
  ⇒ `Running` (line 183-187).
- `testSendFailureDoesNotAdvancePreviousTarget()` (line 191-219):
  injects `fail_send_=true` on the left backend and asserts left
  `previousSentTarget()` stays at initial while right advances.
  Covers invariant 3.

## Probe agents spawned (compliance evidence)

- Subagents spawned: 2 Claude Explore probes (Test coverage probe, Guard
  implementation probe).
- Subagent model: Claude Sonnet (Explore agent).
- **Subagent skip reason for the Codex `gpt-5.4-mini` clause of the
  delegation contract:** Worker-4 runs under the Claude CLI, which cannot
  spawn Codex native subagents nor target `gpt-5.4-mini`. Substituted
  Claude Explore agents in the same parallel-probe shape (two
  independent probes, bullets, no shared state). Findings were
  cross-checked against the actual source before being relied upon —
  see "Probe over-claims" below.
- Findings integrated:
  - Confirmed: all 8 guards have a corresponding production code path.
  - Confirmed: tests cover invariants 1, 2, 3, 4, 5 with real assertions
    on the live servo loop, not on mock-only no-op paths.
  - Confirmed: architect-flagged missing tests (startup realtime failure,
    Cartesian rejection, joint-limit clamp, command timeout expiry)
    remain missing — kept as non-blocking per architect.
- Serial searches before spawn: 1 (`git log --oneline -20` and dir
  listings only) — below the threshold-of-3.
- Probe over-claims rejected after source verification:
  1. Guard probe asserted a "critical bypass" letting `ResetFault` reach
     `Running` in the next iteration. False: after `clearFaultLatch`,
     `motion_state_` is `ConnectedHold`, and `motionAllowed()` returns
     true only for `ArmedHold` or `Running`, so the line 186-187 gate
     converts any subsequent motion command to Hold. The
     `testEmergencyWinsAndResetDoesNotRun` assertion at line 181
     directly proves this.
  2. Test probe asserted "parser drop semantics NOT covered". Partial
     truth: there is no end-to-end test that the `CommandBuffer`
     remains unchanged after a malformed packet, but
     `command_server.cpp:311-312` makes the buffer update
     unconditionally dependent on `parseMessage`'s return value, and
     `testCommandValidation` covers exactly that return value across
     six malformed inputs. Acceptable coverage.

## Blockers

- None.

## Regression risks (low severity, all non-blocking)

1. **Real-mode startup ordering is cosmetic-noisy.** With
   `RB_ALLOW_REAL_ROBOT=1` and `RB_SERVO_ENABLE_RBPODO=OFF`, the
   command listener prints `listening on udp://127.0.0.1:50010` *after*
   `failed to start servo loop`. The process still exits 1, so no
   command is ever serviced — but operators reading logs may briefly
   think the server came up. Worth a future startup-ordering fix; not
   a safety regression.
2. **Loopback bind is YAML-enforced, not config-validated, in mock
   mode.** `src/config/config.cpp:217-221` only rejects network
   exposure in real mode. A misconfigured `dual_mock.yaml` with
   `command_bind: udp://0.0.0.0:...` would still launch. Architect
   accepted this; documenting for future hardening.
3. **`assert()`-only test harness.** `tests/test_safety_policy.cpp`
   uses raw `<cassert>` rather than a framework. Failures abort with
   only file:line, no per-case naming. Pre-existing pattern;
   non-blocking.

## Missing tests (already known by architect, kept non-blocking)

- Startup realtime failure path in real mode
  (`configureRealtimeForLoop` returning false ⇒ `start()` refuses).
- Cartesian command rejection
  (`computeServoTarget` line 306-310 sets `CartesianUnavailable` and
  holds prev_sent).
- Joint-limit clamp behavior in `applySafety`.
- Command timeout expiry path via `latestOrHold`.

These would harden coverage but their absence does not invalidate the
guard invariants — each has an explicit production code path. Architect
explicitly classified them as non-blocking for this iteration.

## Why Go (not Needs More Work)

The architect iteration-1 stop condition has five clauses; all five
hold under fresh verification by worker-4:

1. `ctest --test-dir build --output-on-failure` ⇒ 1/1 pass.
2. `dual_real.yaml` without `RB_ALLOW_REAL_ROBOT=1` ⇒ exit 1 with
   `[FATAL] Refusing real mode...`.
3. `dual_mock.yaml` ⇒ starts, binds `udp://127.0.0.1:50010`, stops on
   SIGTERM.
4. Branch contains the safety policy/test commits
   (`d8d07f0`, `fb6c662`, `bd18dcb`, `f406f93`, `7e0b951`,
   `13b1448`, `77a9495`, `9a9e9c1`, `e774bd0`).
5. No scope drift into rbpodo real backend, Cartesian IK, or 500 Hz
   tuning — confirmed by diff stat scan.

The two adversarial probes produced one false-positive critical
finding (rejected after source verification) and one
partial-coverage concern (acceptable). No blocker survived audit.

## Tasks for Ralph if a future iteration tightens coverage (informational)

These are *not* required for the Go verdict. They are recorded so the
next iteration has a concrete starting point.

1. `tests/test_safety_policy.cpp`: add `testCartesianRejected` —
   submit a `JointTarget` with one arm's mode set to a Cartesian
   variant, assert verdict is `CartesianUnavailable` and prev_sent is
   held.
2. `tests/test_safety_policy.cpp`: add `testCommandTimeoutFallsBackToHold`
   — push a `JointTarget`, sleep past `command_timeout_sec`, assert
   `latestOrHold` returns a `Hold` and the servo loop stays in
   `ConnectedHold`/`ArmedHold` without latching a fault.
3. `tests/test_safety_policy.cpp`: add `testJointLimitClamp` — submit
   a `JointTarget` outside `q_max_deg`, assert `safety_verdict ==
   JointLimitClamped` and the sent target is at the limit.
4. Optional `tests/test_safety_policy.cpp`:
   `testRealtimeStartupFailureRefusesRealMode` — inject a
   `configureRealtimeForLoop` failure via a config that forces
   priority that cannot succeed, assert `start()` returns false in
   real mode (gated by `RB_ALLOW_REAL_ROBOT=1` test env). Skip-with-reason
   if CI cannot reproduce realtime failure cleanly.
5. `src/main.cpp` (or `rb_servo_server.cpp` startup): reorder so that
   `CommandServer` is not started until `servo_loop.start()` returns
   true, eliminating the misleading "listening" log on real-mode
   failure paths.
6. Optional: `src/config/config.cpp` — extend the network-exposure
   check to mock mode (warn-only or refuse-on-non-loopback) so that
   accidentally exposing the mock listener is caught at config load.
7. Slice each follow-up commit per `docs/commit_slicing_advice.md` §4:
   types → behavior → build/config → tests → tools → docs.

None of these are required to land this iteration.

## Verdict

Verdict: Go

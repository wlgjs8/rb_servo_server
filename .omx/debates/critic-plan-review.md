# Critic Plan Review

Generated: 2026-05-14

## Execution Evidence

- Team command shape: `OMX_TEAM_WORKER_CLI_MAP=codex,claude,codex,claude omx team 4:critic ...`
- Team name: `omx-context-critic-pl-b342b995`
- Final team phase: `complete`
- Task state: 4 completed, 0 pending, 0 in progress, 0 failed
- Claude workers started in `don't ask` mode instead of bypass-permissions warning mode.
- Repo-local Claude config used: `.claude/settings.local.json`
- Worker-1 Codex pane accepted startup poorly, so a replacement Codex-Critic native subagent result was recorded into task-1 through the OMX team API. Worker-2, worker-3, and worker-4 completed through the team runtime.

## Reviewed Inputs

- `.omx/context/critic-plan-review-20260514T091652Z.md`
- `.omx/debates/plan-redteam-review.md`
- `rb-servo-server.md`
- `README.md`
- `docs/*.md`
- `CMakeLists.txt`
- `include/**`
- `src/**`
- `config/**`
- `tools/**`

## Role 1: Codex-Critic

Verdict from lane: `Go` only for a narrowed Milestone 1 mock-loop. `No-Go` for rbsim or real robot.

Defensible scope:

- The docs explicitly frame the near-term goal as mock-loop stabilization, not real robot support.
- Real robot paths are gated by `RB_SERVO_ENABLE_RBPODO` and `RB_ALLOW_REAL_ROBOT=1`.
- Missing payload paths already sanitize many invalid commands to Hold, so the earlier missing-payload-to-zero claim should not be overstated.

Objections:

1. `EmergencyStop` can be masked by `ResetFault` because the loop processes ResetFault first and rewrites the command to Hold in `src/control/dual_arm_servo_loop.cpp:132-141`.
2. `stop_both_arms_on_single_arm_error` is configured but not enforced because `SafetyFilter::shouldStopBothArms()` is unused.
3. Regex parsing can throw from malformed numeric arrays and kill the command receive thread.
4. The realtime path still uses mutex-backed `ThreadSafeBuffer`.
5. Failed `sendServoJ` still advances previous-sent target state.
6. There is no `enable_testing()` / `add_test()` baseline.
7. `RbpodoBackend` remains a placeholder and must not be treated as rbsim/real evidence.

Rebuttals:

1. Mock-only progress is acceptable, but only with explicit No-Go labeling for rbsim/real.
2. Missing payload is not the main parser risk; malformed numeric payloads and parser exceptions are.
3. Sequential left/right send is tolerable for mock but not evidence of real coordinated bimanual timing.
4. Async logging is acceptable for short mock smoke tests, but unbounded queue/string behavior is not realtime-safe.
5. `StatePublisher` must publish servo-loop-owned snapshots, not call `MockBackend::readState()` from another thread.

## Role 2: Claude-Critic

Verdict from lane: reject the prior conditional-Go plan as written. Recommend `Needs More Investigation` unless key blockers are fixed and tests exist.

Additional findings:

1. `latch_fault_on_emergency_stop` is parsed/configured but the loop latches EmergencyStop unconditionally.
2. Plain `running_` flags in `DualArmServoLoop`, `ServoLogger`, and `StatePublisher` should be reviewed for data races, similar to the already flagged `CommandServer::socket_fd_` race.
3. `extractObject()` does not handle braces inside JSON strings.
4. A malformed numeric token in `extractArray()` can throw `std::stod`, escape the parse path, and stop the receive thread.
5. Negative `timeout_sec` can poison stale-command logic through signed-to-unsigned conversion.
6. `SnapToActual` plus prev/prevprev updates can collapse acceleration-filter history.
7. `ServoLogger::push()` uses a mutex on the servo loop path, adding another priority-inversion vector.
8. Realtime setup failures can be fail-open if `lockMemory()` / realtime priority setup errors do not become hard startup failures when realtime is requested.
9. P2 replacements lack rollback contracts: no fallback option, soak gate, side-by-side period, or revert plan.

Top requested improvements:

1. Pick a deterministic EmergencyStop-vs-ResetFault rule and test it.
2. Treat parser hardening as current safety work, not loose P2 work.
3. Wire or remove dead config knobs.
4. Establish tests before P0/P1 behavior edits.
5. Add rollback contracts for parser/buffer/send/logging rewrites.

## Role 3: Codex-Plan-Builder

Verdict from lane: `Go` only for narrowed Milestone 1 mock-loop stabilization. `No-Go` for rbsim/real.

Minimal plan:

1. Scope gate first: explicitly label the next task as mock-only M1.
2. Add no-dependency CTest harness before behavior changes:
   - `enable_testing()` in `CMakeLists.txt`.
   - small assert-based test executables under `tests/`.
   - parser tests without requiring UDP sockets.
3. Fix P0 safety semantics:
   - EmergencyStop before ResetFault, or reject mixed EmergencyStop/ResetFault deterministically.
   - apply or remove `stop_both_arms_on_single_arm_error`.
   - do not advance previous-sent target after failed `sendServoJ`.
4. Keep P1 work constrained:
   - StatePublisher publishes servo-loop-owned samples.
   - rbpodo remains compile/runtime gated.
5. Make P2 blockers explicit before rbsim/real:
   - replace regex parser.
   - replace mutex command buffer for realtime.
   - decide dual-arm send synchronization.
   - bound logger queue or define drop/backpressure behavior.

Verification evidence from lane:

- `cmake --build build -j` passed.
- `ctest --test-dir build --output-on-failure` found no tests.
- `cppcheck --enable=warning,style,performance,portability --error-exitcode=1 include src` exited 0 with only a style note.
- A short executable smoke attempt was not usable because UDP socket creation was blocked in the sandbox.

## Role 4: Claude-Verifier

Verdict from lane: `Needs More Investigation`, resolving to Go only if R1-R3 are treated as Go-blocking.

Failure conditions:

1. ResetFault masks EmergencyStop in mixed packets.
2. `prev_sent_q` advances after failed send.
3. both-arm stop policy is configured but not applied.
4. mutex-backed command buffer is used from the realtime loop.
5. `socket_fd_` is a plain int shared across stop/threadMain paths.
6. no CTest baseline exists.
7. logger queue is unbounded and fault reasons are unescaped in CSV.
8. `MockBackend::readState()` mutates plant state.
9. acceleration clamp semantics are split across safety and trajectory filters without focused tests.

Regression checklist:

1. Mixed `{left: ResetFault, right: EmergencyStop}` must latch fault.
2. Simulated `sendServoJ` failure must not advance previous-sent target.
3. Single-arm state/safety error must obey both-arm stop policy.
4. Missing target payload must Hold, not zero.
5. Cartesian mode must Hold/previous-target with no jump.
6. Stale command must Hold both arms.
7. Mock sine smoke must show expected timing and no spurious safety verdicts.
8. rbpodo must remain compile/runtime gated.
9. shutdown under traffic should have no race/UB under TSan or equivalent.
10. logger must not grow unbounded during disk stall.
11. future StatePublisher must not call `MockBackend::readState()` directly.
12. `enable_testing()` plus CTest must cover the safety cases above.

## Cross-Rebuttals

- The strongest defense is correct but narrow: mock-only M1 is defensible. It does not justify rbsim/real claims.
- The strongest red-team critique is also correct: parser exceptions and EmergencyStop precedence are not cosmetic debt; they are current safety defects even in mock mode.
- Missing-payload-to-zero should be refuted as an overstatement because `command_server.cpp` sanitizes missing payloads to Hold, but malformed numeric payloads remain a real parser failure path.
- Replacing every subsystem immediately is too broad; tests plus P0 safety semantics should come first.
- P2 rewrites must be named as rbsim/real gates with rollback/soak contracts, not vague future cleanup.

## Final Plan

1. Treat the next implementation round as `Milestone 1 mock-only stabilization`.
2. Add the CTest baseline first.
3. Fix and test these P0 items before any Go claim:
   - EmergencyStop precedence over ResetFault.
   - parser exception handling for malformed numeric payloads.
   - positive/finite timeout validation.
   - both-arm stop policy semantics.
   - failed-send previous-target handling.
4. Add P1 guards:
   - explicit behavior for `latch_fault_on_emergency_stop`.
   - data-race review for lifecycle flags and socket fd.
   - StatePublisher snapshot ownership rule.
5. Make P2 rbsim/real gates explicit:
   - parser replacement or hardened decoder.
   - non-blocking/realtime-safe command buffer.
   - dual-arm send timing contract.
   - bounded logging/backpressure policy.
   - rollback contracts and soak criteria.

## Conclusion

Needs More Investigation

The plan is not ready to proceed as a broad implementation or rbsim/real roadmap. It can proceed only after the immediate investigation is converted into concrete P0 test cases and decisions for EmergencyStop precedence, parser exception handling, timeout validation, both-arm stop semantics, and failed-send state handling.

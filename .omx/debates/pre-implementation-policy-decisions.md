# Pre-Implementation Policy Decisions

Source review:
- `.omx/debates/critic-plan-review.md`
- `.omx/debates/plan-redteam-review.md`
- OMX team `critic-plan-review-md-5ec13dd9`, `4:architect`, mixed CLI map `codex,claude,codex,claude`

Scope:
- No implementation was performed.
- This document fixes implementation policy before code changes.
- File references are evidence anchors, not a complete implementation diff.

## 1. EmergencyStop vs ResetFault simultaneous input

Decision:
- EmergencyStop wins over ResetFault in the same received command.
- If a command requests both `EmergencyStop` and `ResetFault`, the loop must latch EmergencyStop and must not clear an existing or newly requested emergency latch in that tick.
- `ResetFault` is valid only when the command does not also request EmergencyStop.

Rationale:
- Current ordering in `src/control/dual_arm_servo_loop.cpp:132-141` processes ResetFault first, rewrites the command to Hold, then checks EmergencyStop. This can let ResetFault mask EmergencyStop.
- `docs/fail_safe_policy.md:25` says EmergencyStop latches current/last-safe pose. A reset command must not override the higher-priority stop command in the same packet.

Rejected alternatives:
- ResetFault wins: unsafe because a mixed packet can erase the most conservative command.
- Process both sequentially: ambiguous and currently produces masking behavior.
- Drop mixed packets without latching: less safe than honoring EmergencyStop.

Required tests:
- Mixed `EmergencyStop` plus `ResetFault` command latches EmergencyStop.
- Existing `fault_latched_ == true`, mixed command keeps the latch set.
- Pure `ResetFault` still clears a latch when robot state is healthy.
- Pure `EmergencyStop` behavior remains unchanged.

Implementation touchpoints:
- `src/control/dual_arm_servo_loop.cpp:132-141`
- `commandRequestsResetFault(...)`
- `commandRequestsEmergencyStop(...)`
- loop-level fault latch tests

Go-blocking 여부:
- Yes.

Escalation Required:
- No.

## 2. malformed JSON / malformed numeric payload

Decision:
- Malformed packets must be contained per packet and must not terminate `CommandServer`.
- Malformed JSON syntax, malformed numeric tokens, numeric overflow, invalid array length, and malformed required payloads must result in "drop packet and continue receiving"; no new command may be written to `CommandBuffer`.
- If the malformed packet partially parses into a motion mode but required payload is invalid, the effective behavior must be Hold, never zero joint arrays.

Rationale:
- `src/network/command_server.cpp:92-104` uses regex array extraction and unguarded `std::stod`; bad tokens can throw.
- `src/network/command_server.cpp:279-282` catches exceptions only around the whole receive loop, logs, and sets `running_ = false`, creating a one-packet receive-thread DoS.
- `docs/network_protocol.md:138` and `docs/fail_safe_policy.md:15-18` require absent or malformed payloads to Hold rather than become motion.

Rejected alternatives:
- Let the outer `threadMain()` catch kill the receive thread: makes a malformed packet a persistent communication failure.
- Convert malformed numeric tokens to zero: violates the no-synthesized-zero invariant in `docs/fail_safe_policy.md:7-9`.
- Replace the whole parser immediately as the only fix: desirable later, but too broad for the P0 containment fix.

Required tests:
- Bad JSON does not stop the receive thread.
- `q_target_deg` with `abc`, `nan`, `inf`, or `1e9999` is rejected without updating `CommandBuffer`.
- Wrong-length arrays become Hold or packet rejection, never zero-filled motion.
- Oversized/truncated UDP packet at buffer limit is rejected.
- Boolean fields such as `coupled_timeout: 12` are not silently accepted as `true`.

Implementation touchpoints:
- `src/network/command_server.cpp:68-104`
- `src/network/command_server.cpp:271-282`
- `src/network/command_server.cpp:285-338`
- parser unit tests that call `CommandServer::parseMessage(...)` without UDP sockets

Go-blocking 여부:
- Yes.

Escalation Required:
- No.

## 3. `timeout_sec <= 0`

Decision:
- `timeout_sec` must be finite and strictly positive at every command and config boundary.
- A packet-level or per-arm `timeout_sec <= 0`, NaN, infinity, or overflow must be rejected before it reaches `CommandBuffer`.
- Config values for `servo.command_timeout_sec` and `safety.command_timeout_sec` must also be finite and strictly positive at load time.
- Do not silently coerce invalid command timeouts to `0.2` inside stale-command enforcement.

Rationale:
- `src/control/command_buffer.cpp:22` currently coerces non-positive left timeout to `0.2`, hiding invalid input.
- `src/network/command_server.cpp:302-309` parses top-level timeout into both arms, while `src/network/command_server.cpp:184-185` can override per-arm timeout.
- Config parse paths in `src/config/config.cpp:211,226` accept parsed doubles without bounds validation.
- `coupled_timeout` is parsed at `src/network/command_server.cpp:296` but is not enforced in stale logic.

Rejected alternatives:
- Treat `timeout_sec <= 0` as "never timeout": unsafe stale-command behavior.
- Keep fallback `0.2`: masks invalid senders and contradicts explicit command contracts.
- Use only left-arm timeout forever: incomplete for dual-arm commands and inconsistent with per-arm parsing.

Required tests:
- `timeout_sec: 0`, negative, NaN, infinity, and overflow are rejected.
- Missing timeout uses the configured default, not a hard-coded hidden default.
- Per-arm invalid timeout rejects the packet or arm command according to parser policy.
- Stale conversion to Hold works for valid timeout.
- `coupled_timeout` behavior is either implemented or explicitly rejected by tests and docs.

Implementation touchpoints:
- `src/network/command_server.cpp:302-309`
- `src/network/command_server.cpp:184-185`
- `src/control/command_buffer.cpp:21-29`
- `src/config/config.cpp:211,226`
- `include/rb_servo/config/config.hpp:39,52`

Go-blocking 여부:
- Yes.

Escalation Required:
- No.

## 4. `stop_both_arms_on_single_arm_error`

Decision:
- When `safety.stop_both_arms_on_single_arm_error == true`, any state-level or target-filter safety error on either arm must drive both arms into the existing fault-latch hold path.
- When the setting is false, the faulted arm holds and the healthy arm may continue, but only for mock/simulation policy tests.
- The default remains true.

Rationale:
- `SafetyFilter::shouldStopBothArms(...)` exists in `src/control/safety_filter.cpp:74-82` but is effectively unused by `src/control/dual_arm_servo_loop.cpp`.
- Current `applySafety()` in `src/control/dual_arm_servo_loop.cpp:247-307` evaluates each arm independently, so one unsafe arm can hold while the other continues tracking.
- For dual-arm operation, asymmetric motion after one arm faults is the higher-risk default.

Rejected alternatives:
- Always stop both arms without config: removes existing operator-test configurability.
- Stop both only when both arms are unsafe: defeats the safety purpose.
- Only set a combined verdict but still send the healthy arm target: preserves the current defect.

Required tests:
- Left state error with policy true holds both arms and latches fault.
- Right disconnected state with policy true holds both arms.
- Single-arm tracking error with policy true holds both arms.
- Same scenarios with policy false hold only the faulted arm.
- ResetFault recovery re-baselines and resumes only when state is healthy.

Implementation touchpoints:
- `src/control/dual_arm_servo_loop.cpp:247-307`
- `src/control/safety_filter.cpp:74-82`
- `include/rb_servo/config/config.hpp:45`
- `src/config/config.cpp:228`

Go-blocking 여부:
- Yes.

Escalation Required:
- No.

## 5. `sendServoJ` failure handling

Decision:
- A failed `sendServoJ` must not advance `left_prev_sent_q_deg_` or `right_prev_sent_q_deg_` for the failed arm.
- In real mode, any `sendServoJ` failure must latch a fault and hold both arms when `stop_both_arms_on_single_arm_error` is true.
- In mock/simulation, a failed send must at least preserve the last successfully sent target and record the failure in the sample.

Rationale:
- `sendTargets()` reports `left_ok` and `right_ok`, but `src/control/dual_arm_servo_loop.cpp:194-197` unconditionally advances previous sent targets after sending.
- This can make future safety checks believe a failed target was actually sent.

Rejected alternatives:
- Keep updating previous sent target unconditionally: corrupts the last-safe basis.
- Ignore send failures and rely on next tick: unsafe for real robot and misleading for logs.
- Retry indefinitely inside the servo loop: risks blocking the real-time loop.

Required tests:
- Failed left send does not update `left_prev_sent_q_deg_`.
- Failed right send does not update `right_prev_sent_q_deg_`.
- Real-mode failed send latches fault.
- Mock-mode failed send is logged and preserves previous target.
- Partial failure with both-arm-stop policy true holds both arms.

Implementation touchpoints:
- `src/control/dual_arm_servo_loop.cpp:164-197`
- `src/control/dual_arm_servo_loop.cpp:309-315`
- `src/robot/mock_backend.cpp`
- `src/robot/rbpodo_backend.cpp`
- `ServoSample.left_send_ok/right_send_ok`

Go-blocking 여부:
- Yes.

Escalation Required:
- No.

## 6. realtime setup failure handling

Decision:
- Real mode must fail startup if requested realtime setup cannot be established.
- Mock and simulation may warn and continue, but the degraded realtime status must be visible in logs and later state telemetry.
- `enable_realtime_priority`, priority range, CPU core, and memory lock preconditions must be validated before the servo loop begins.

Rationale:
- `src/core/realtime.cpp:13-58` returns booleans for memory lock, realtime priority, and CPU pinning, but `src/control/dual_arm_servo_loop.cpp:101-107` discards the return values.
- `src/config/config.cpp:266-271` already hard-refuses real mode unless `RB_ALLOW_REAL_ROBOT=1`; realtime failure should be treated with the same safety seriousness in real mode.
- `include/rb_servo/config/config.hpp:55` and `src/config/config.cpp:185` disagree on the default for `enable_realtime_priority`, creating config-source ambiguity.

Rejected alternatives:
- Warn-and-continue in real mode: hides a broken safety precondition.
- Fail for mock/simulation whenever realtime setup fails: hurts developer workflow without improving hardware safety.
- Move all realtime responsibility to deployment docs: leaves unsafe runtime behavior possible.

Required tests:
- Real mode with missing realtime permission refuses startup.
- Mock/simulation with missing realtime permission logs degraded mode and continues.
- Invalid `realtime_priority` and out-of-range `cpu_core` fail config validation.
- Memory lock failure in real mode refuses startup.
- Config defaults are consistent whether built directly or loaded from YAML.

Implementation touchpoints:
- `src/core/realtime.cpp`
- `include/rb_servo/core/realtime.hpp`
- `src/control/dual_arm_servo_loop.cpp:101-107`
- `src/config/config.cpp:185,213-215,266-271`
- `include/rb_servo/config/config.hpp:55-57`
- `src/main.cpp`

Go-blocking 여부:
- Yes for real/rbsim readiness. No for mock-only P0 if degraded status is explicit.

Escalation Required:
- No.

## 7. UDP bind / network exposure default

Decision:
- Safe default bind must be loopback: `udp://127.0.0.1:50010` for command input.
- Wildcard bind `udp://0.0.0.0:50010` must be explicit opt-in in config and must emit a startup warning naming the exposed host and port.
- Real mode must reject wildcard command bind unless a deliberate override is present, for example `RB_ALLOW_NETWORK_EXPOSURE=1` or an explicit config flag.
- State publisher exposure follows the same rule when it is implemented.

Rationale:
- `include/rb_servo/config/config.hpp:66` defaults command bind to `udp://0.0.0.0:50010`.
- `config/dual_mock.yaml`, `config/dual_rbsim.yaml`, and `config/dual_real.yaml` also expose command input on `0.0.0.0`.
- `src/network/command_server.cpp:243-251` binds the configured IPv4 address directly. There is no authentication or source allowlist.
- `StatePublisher` is currently not wired in `src/main.cpp`, but `config/dual_*.yaml` uses `tcp://0.0.0.0:50110`, which is inconsistent with the command parser's current `udp://`-only support.

Rejected alternatives:
- Keep wildcard as the default: exposes a motion-control surface by default.
- Remove wildcard support: makes remote control impossible for legitimate lab setups.
- Rely only on host firewall: correct deployment layer, but not a safe application default.

Required tests:
- Default config binds command input to loopback.
- Wildcard bind in mock/simulation starts only with explicit opt-in and warns.
- Wildcard bind in real mode refuses startup without explicit exposure override.
- Invalid or unsupported URI schemes are rejected clearly.
- State publisher cannot be wired with unsupported `tcp://` until transport policy exists.

Implementation touchpoints:
- `include/rb_servo/config/config.hpp:65-68`
- `config/dual_mock.yaml`
- `config/dual_rbsim.yaml`
- `config/dual_real.yaml`
- `src/config/config.cpp:234-237`
- `src/network/command_server.cpp:28-42,243-251`
- `src/network/state_publisher.cpp`

Go-blocking 여부:
- Yes for real/rbsim readiness. No for local mock if loopback is enforced.

Escalation Required:
- No for the safe default. Yes only if operations wants wildcard bind by default.

## 8. StatePublisher state ownership

Decision:
- `DualArmServoLoop::loopMain()` is the sole production owner of robot state snapshots.
- `StatePublisher` must receive state only through `StatePublisher::updateState(const DualRobotState&)`.
- `StatePublisher::threadMain()` must never call `IRobotBackend::readState(...)`, instantiate a backend, or otherwise pull robot state independently.

Rationale:
- `MockBackend::readState()` mutates simulated state in `src/robot/mock_backend.cpp:24-44`; a second reader would double-advance the plant model.
- `DualArmServoLoop` already reads both robot states at `src/control/dual_arm_servo_loop.cpp:124-126`.
- `StatePublisher` already has `updateState(...)` and `latest_state_` in `src/network/state_publisher.cpp:14-16` and `include/rb_servo/network/state_publisher.hpp`.

Rejected alternatives:
- Publisher directly reads backend state: creates thread-safety and mock-model correctness risks.
- Add `peekState()` to the backend interface: expands backend contract unnecessarily.
- Make the publisher own the servo loop state: wrong direction for the current architecture.

Required tests:
- `updateState()` stores the latest snapshot.
- StatePublisher wired into the loop publishes exactly the snapshots read by the loop.
- Static or grep test rejects backend `readState()` calls outside the servo loop and backend implementations.
- MockBackend `readState()` call count remains one per servo tick when StatePublisher is enabled.

Implementation touchpoints:
- `include/rb_servo/network/state_publisher.hpp`
- `src/network/state_publisher.cpp`
- `include/rb_servo/control/dual_arm_servo_loop.hpp`
- `src/control/dual_arm_servo_loop.cpp:124-126`
- `src/main.cpp`
- `src/robot/mock_backend.cpp`

Go-blocking 여부:
- No for current mock-only work because StatePublisher is not wired. Yes before StatePublisher wiring lands.

Escalation Required:
- No.

## 9. parser / command buffer / logger replacement and rollback policy

Decision:
- P0 implementation must harden current components before broad replacement:
  - parser: add per-packet exception containment and malformed numeric rejection;
  - command buffer: keep current mutex buffer for mock P0, but do not claim realtime safety;
  - logger: keep async logger for mock P0, but do not add blocking work on the servo path.
- Full replacements are deferred behind behavior-lock tests and explicit rollback gates:
  - parser replacement must preserve malformed-input behavior and command schema tests;
  - command buffer replacement must prove no stale-command, timeout, or latest-command regression;
  - logger replacement must prove bounded servo-loop overhead and no sample loss policy regression.
- Replacement rollout must keep the old component selectable for one milestone or one release candidate until soak evidence passes.

Rationale:
- Current parser is regex-based in `src/network/command_server.cpp`.
- Current command buffer is mutex-backed through `include/rb_servo/core/thread_safe_buffer.hpp` and `src/control/command_buffer.cpp`.
- Current logger is async but still entered from the servo loop at `src/control/dual_arm_servo_loop.cpp:190-192`.
- `docs/design_review_fixes.md:107-111` already lists lock-free/priority-inheritance buffer and production JSON parser as future work, not immediate prerequisites for mock P0.

Rejected alternatives:
- Replace parser, buffer, and logger together: too broad and hard to rollback.
- Defer all hardening until replacement: leaves current safety defects live.
- Remove old implementations immediately after replacement: no rollback path for timing or parser-regression failures.

Required tests:
- Behavior-lock parser corpus for valid commands, missing payload, malformed payload, malformed numeric values, timeout edge cases, and EmergencyStop/ResetFault.
- CommandBuffer tests for latest command, stale command, invalid timeout rejection, and coupled timeout policy.
- Logger tests for bounded push behavior and graceful shutdown.
- Soak test at target servo rate with parser, buffer, and logger enabled.
- Rollback test or configuration proof that the previous component can be selected if replacement fails.

Implementation touchpoints:
- `src/network/command_server.cpp`
- `src/control/command_buffer.cpp`
- `include/rb_servo/core/thread_safe_buffer.hpp`
- `src/logging/servo_logger.cpp`
- `src/control/dual_arm_servo_loop.cpp:190-197`
- `CMakeLists.txt`
- `docs/network_protocol.md`
- `docs/fail_safe_policy.md`

Go-blocking 여부:
- Yes for broad replacement or real/rbsim readiness.
- No for immediate mock P0 safety fixes if hardening is kept minimal and behavior-locked.

Escalation Required:
- Yes for production rollout thresholds: soak duration, acceptable jitter/sample-loss limits, and when old components may be removed.

## Final Conclusion

Needs Human Decision.

Reason:
- Architect team can decide and proceed with immediate P0 safety policy fixes for EmergencyStop precedence, parser containment, timeout validation, both-arm stop enforcement, failed-send handling, realtime real-mode fail-start, loopback network default, and StatePublisher ownership.
- Human/product/operations decision remains required only for production replacement rollout thresholds in policy 9, and for any request to keep wildcard network bind as a default contrary to policy 7.

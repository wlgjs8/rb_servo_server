# TODO1-3 Real-Readiness Roadmap

Created: 2026-05-15
Base branch: `main`
Base commit: `0ee312a`

## Requirements Summary

This plan turns `TODO1.md`, `TODO2.md`, and `TODO3.md` into an execution roadmap after the verified real-readiness safety patch was promoted to `main`.

Current status:

- `main` contains the verified safety patch from `real-readiness-safety-patch`.
- M1 mock safety hardening closeout is implemented in the follow-up branch: `ServoSnapshot`, send timing columns, logger zero-capacity drop handling, and additional safety tests.
- Mock safety loop is Go for continued hardening.
- rbsim/real robot motion remains No-Go until rbpodo state/read/send paths are implemented and bring-up evidence exists.
- TODO2's ArmMotion bootstrap race is fixed structurally by a lifecycle-command queue in `CommandBuffer`, with a tool-side settle delay as an extra guard.
- TODO3's P0 state-validity guard is fixed: startup refuses invalid robot state, safe hold uses only valid `q_actual`, and `RbpodoBackend` refuses operation while incomplete.

Primary invariant:

No failure path may synthesize a new motion target from default-initialized joint arrays. Failure must hold the last successfully sent target or a validated current joint state.

## Evidence From Current Main

- Startup order now starts `servo_loop` before `command_server`, so external command ingress only opens after backend/realtime/startup hold succeeds: `src/main.cpp:58`.
- Lifecycle commands are queued separately from latest motion commands: `src/control/command_buffer.cpp:43`.
- Invalid/stale command buffer entries become Hold instead of using hard-coded timeout recovery: `src/control/command_buffer.cpp:33`.
- Startup validates both robot states before seeding previous/fault hold targets: `src/control/dual_arm_servo_loop.cpp:118`.
- Runtime invalid robot state latches/holds according to policy: `src/control/dual_arm_servo_loop.cpp:203`.
- Safe hold uses only valid joint state: `src/control/dual_arm_servo_loop.cpp:315`.
- `RobotState::has_valid_joint_state` exists and defaults false: `include/rb_servo/core/types.hpp:119`.
- `RbpodoBackend` refuses connect/read/send/stop/reset success until real implementation exists: `src/robot/rbpodo_backend.cpp:34`.
- Real mode network exposure guard applies to both `command_bind` and `state_pub_bind`, fail-closed for unknown/non-loopback bind hosts: `src/config/config.cpp:169`.
- Logger is bounded and records motion state plus drop count: `include/rb_servo/logging/servo_logger.hpp:23`, `src/logging/servo_logger.cpp:48`, `src/logging/servo_logger.cpp:82`.
- `StatePublisher` is still a placeholder and must not read backends directly when implemented: `src/network/state_publisher.cpp:33`.

## Acceptance Criteria

Before any real robot motion:

1. `cmake -S . -B build && cmake --build build -j && ctest --test-dir build --output-on-failure` passes.
2. `dual_mock` smoke shows `JointTarget`, `Running`, multiple command seqs, and non-trivial sent joint motion in `logs/servo_log.csv`.
3. Real mode fails without `RB_ALLOW_REAL_ROBOT=1`.
4. Real mode fails if `command_bind` or `state_pub_bind` is exposed without `RB_ALLOW_NETWORK_EXPOSURE=1`.
5. `RB_SERVO_ENABLE_RBPODO=ON` does not permit servo startup until rbpodo state validity and send paths are implemented.
6. Any future IK/Cartesian/action failure holds last safe target and never uses `JointArray{}` as a target.
7. Each milestone below ends with a critic/code-review pass and updated machine-readable evidence under `.omx/state` or `.omx/plans`.

## Implementation Roadmap

### M1 - Mock Safety Hardening Closeout

Goal:
Close remaining TODO1 mock-mode safety/test/documentation gaps without enabling rbpodo motion.

Already done:

- Startup ordering.
- CommandBuffer invalid timeout fallback removal.
- Lifecycle queue for ArmMotion/DisarmMotion/EmergencyStop/ResetFault.
- Bounded logger and CSV escaping.
- Explicit `RB_CHECK`-style tests.
- State validity guard.

Remaining work:

1. Add a `ServoSnapshot` type as the single read surface for publisher/debug/test.
2. Move direct test/debug reads of mutable servo loop state behind snapshot access.
3. Add send timing fields to `ServoSample`: send start/end timestamps, duration, and skew.
4. Add missing P0 tests from TODO1 that are not already covered: velocity clamp max step, acceleration overshoot, robot disconnected/error real policy, oversized UDP packet drop, mixed EmergencyStop+ResetFault priority, and explicit coupled timeout behavior.
5. Update README/network docs so command/state endpoint defaults and smoke expectations match current config.

Implementation touchpoints:

- `include/rb_servo/core/types.hpp`
- `include/rb_servo/control/dual_arm_servo_loop.hpp`
- `src/control/dual_arm_servo_loop.cpp`
- `src/logging/servo_logger.cpp`
- `tests/test_safety_policy.cpp`
- `docs/testing.md`
- `docs/network_protocol.md`
- `rb-servo-server.md`

Verification:

- Unit tests for snapshot content and safety verdicts.
- Mock sine smoke validates actual non-trivial motion.
- Logger overflow test proves `push()` does not block indefinitely and increments drop count.

Go/No-Go:

- Go only for mock/rbsim planning after all tests pass.
- Still No-Go for rbpodo motion.

### M2 - StatePublisher From ServoSnapshot

Goal:
Implement the minimum state stream needed for teleop/policy/data tools without creating backend read races.

Policy:

- `StatePublisher` must publish only servo loop snapshots.
- `StatePublisher` must not call `IRobotBackend::readState()`.
- Initial protocol is UDP JSON on `network.state_pub_bind`.
- Publish rate should be configurable and lower than or equal to servo rate, default 50 Hz.

Implementation touchpoints:

- `include/rb_servo/network/state_publisher.hpp`
- `src/network/state_publisher.cpp`
- `src/main.cpp`
- `include/rb_servo/config/config.hpp`
- `src/config/config.cpp`
- `include/rb_servo/core/types.hpp`
- `docs/network_protocol.md`

Required tests:

- Publisher rejects exposed bind in real mode unless `RB_ALLOW_NETWORK_EXPOSURE=1`.
- Publisher emits valid JSON with tick, motion_state, safety_verdict, fault state, q_actual, q_sent, send_ok, and command seq.
- Publisher still shuts down cleanly if no subscriber exists.
- Publisher does not mutate mock backend plant by causing additional `readState()` calls.

Go/No-Go:

- Go for policy/teleop observation in mock.
- No-Go for closed-loop real motion until rbpodo state reader is implemented.

### M3 - rbpodo Build Integration And Hold-Only Backend

Goal:
Replace the rbpodo placeholder refusal with a real hold-only backend that can connect, initialize, read valid state, and send only validated hold targets.

Required human/product decisions before implementation:

- Exact rbpodo version installed on the robot PC.
- Whether rbpodo is provided through system package, vcpkg/conan, source checkout, or vendored submodule.
- First bring-up ACK policy: recommended `BlockingAck` for simulation/hold-only.
- Whether real hardware is available during this milestone or only rbsim.

Policy:

- `RB_SERVO_ENABLE_RBPODO=ON` must use `find_package(rbpodo REQUIRED)` or fail at configure time.
- No FetchContent network download should be required on robot PCs.
- `readState()` may set `has_valid_joint_state=true` only after reading real joint data from a trusted rbpodo data channel.
- `sendServoJ()` must return structured timing/error data before dual-arm sync work.
- No Cartesian/action/gripper command is enabled in this milestone.

Implementation touchpoints:

- `CMakeLists.txt`
- `include/rb_servo/robot/i_robot_backend.hpp`
- `include/rb_servo/robot/rbpodo_backend.hpp`
- `src/robot/rbpodo_backend.cpp`
- `include/rb_servo/config/config.hpp`
- `src/config/config.cpp`
- `config/dual_rbsim.yaml`
- `config/dual_real.yaml`
- `tests/test_safety_policy.cpp`
- `docs/fail_safe_policy.md`
- `docs/testing.md`

Required tests:

- Configure fails clearly if `RB_SERVO_ENABLE_RBPODO=ON` and rbpodo is missing.
- With rbpodo enabled but state invalid/stale, servo loop refuses startup.
- `readState()` stale timestamp causes `RobotStateError`.
- Send failure latches fault in real mode.
- Stop/reset failure is visible and not reported as success.

Bring-up acceptance:

- rbsim connect/readState only.
- rbsim Hold-only at 10 Hz, 50 Hz, then 100 Hz.
- Real config without `RB_ALLOW_REAL_ROBOT=1` still fails.
- Real config with env but invalid state still fails before command ingress.

Go/No-Go:

- Go for rbsim hold-only only after state-valid and send-failure tests pass.
- No-Go for real motion until H0-H6 hardware bring-up evidence exists.

### M4 - Single-Arm 100 Hz Bring-Up

Goal:
Validate one arm with real/rbsim servo_j hold and tiny sine before enabling dual-arm motion.

Implementation touchpoints:

- Add one-arm enable/disable config.
- Add `DisabledBackend` or equivalent explicit disabled-arm backend.
- Add send timing to backend result and logger.
- Add one-arm tools for hold and small sine.

Required tests/smokes:

- Hold target repeats `q_actual` and never jumps to zero.
- Single-arm +/-0.25 deg sine at 20-50 Hz command input and 100 Hz servo.
- EmergencyStop latches, ResetFault returns ConnectedHold, ArmMotion required before motion.
- 10-minute soak records missed tick, drop, send failure, tracking error, send latency.

Go/No-Go:

- Go for single-arm 100 Hz only after soak evidence.
- No-Go for 200 Hz or dual-arm until latency/skew data is reviewed.

### M5 - Dual-Arm Same-Tick 100 Hz

Goal:
Run both arms from one target generation tick and measure send skew before changing sender architecture.

Policy:

- Do not introduce parallel sender threads until measured sequential send skew requires it.
- In real dual-arm mode, one arm send failure is treated as coupled batch failure unless explicitly changed by policy review.

Implementation touchpoints:

- `ServoSample` send timing fields.
- `DualArmServoLoop::sendTargets`.
- Optional `DualArmSender` only after data shows sequential send is inadequate.
- Plotting/analysis script for skew histograms.

Required tests/smokes:

- Dual hold.
- Dual small sine.
- One-arm send failure causes both-arm fault/hold in real/coupled mode.
- Skew distribution logged and reviewed.

Go/No-Go:

- Go for dual 100 Hz after soak and skew review.
- No-Go for 200 Hz until jitter/send latency thresholds are explicit.

### M6 - Cartesian TCP Layer

Goal:
Enable `TcpTargetStand`, `TcpDeltaStand`, and `TcpDeltaLocal` only through explicit FK/IK result types and workspace guards.

Policy:

- IK/FK failure returns `CartesianSolveResult{ok=false}` and holds previous safe target.
- IK/FK failure never returns or consumes default `JointArray{}`.
- Frame convention must be tested before any robot motion.

Implementation touchpoints:

- `include/rb_servo/control/cartesian_controller.hpp`
- `src/control/cartesian_controller.cpp`
- new `RobotKinematics` abstraction
- config for URDF/MJCF, frames, workspace, singularity thresholds
- `tests/test_safety_policy.cpp` or separate kinematics tests

Required tests:

- Unreachable pose holds previous safe target.
- 1 mm `TcpDeltaStand` moves in expected stand-frame direction in mock/kinematic test.
- `TcpDeltaLocal` uses TCP local frame convention.
- Left/right base transforms produce expected sign/direction.
- Workspace/singularity/collision guards block unsafe targets.

Go/No-Go:

- Go for simulation only after deterministic FK/IK tests pass.
- No-Go for real Cartesian motion until joint-space real bring-up is stable.

### M7 - Gripper, Action Chunk, Camera/Dataset

Goal:
Prepare policy/data collection layers after joint and Cartesian control are stable.

Order:

1. Gripper command/state/logging.
2. Action chunk protocol at 10-30 Hz with C++ interpolation at servo rate.
3. Jerk-limited interpolation candidate evaluation, preferably offline before production.
4. Camera recorder and dataset builder with steady/system time alignment.

Implementation touchpoints:

- `ArmCommand` gripper fields and typed `GripperCommand`/`GripperState`.
- backend gripper interface.
- `ActionChunkBuffer`.
- camera recorder process and metadata JSONL.
- dataset builder for robot/camera nearest/interpolated alignment.

Required tests:

- Gripper command does not move arm targets on malformed payload.
- Expired action chunk holds safely.
- IK/action failure holds last safe target.
- Dataset replay can reconstruct robot state and camera frames for a short mock episode.

Go/No-Go:

- Go only after M1-M6 are stable.

## Risks And Mitigations

- Risk: rbpodo API changes or unavailable dependency.
  Mitigation: pin the tested rbpodo version, fail configure clearly, and avoid robot-PC network downloads.

- Risk: StatePublisher causes extra backend reads and changes mock timing/state.
  Mitigation: publish only servo loop snapshots.

- Risk: coupled dual-arm send semantics are ambiguous.
  Mitigation: adopt coupled batch failure for real mode until a policy review approves per-arm accept.

- Risk: EmergencyStop naming implies hardware E-stop.
  Mitigation: document current software behavior as ProtectiveStop/FaultHold semantics and reserve hardware E-stop for physical safety circuit.

- Risk: Cartesian/IK failure reintroduces zero-target paths.
  Mitigation: require explicit result structs and tests that fail if `JointArray{}` is used on solver failure.

## Verification Steps For Every Milestone

1. `git status --short --branch` before edits.
2. Targeted unit tests for new behavior.
3. `cmake -S . -B build`.
4. `cmake --build build -j`.
5. `ctest --test-dir build --output-on-failure`.
6. Relevant mock/rbsim/real smoke from the milestone.
7. `git diff --check`.
8. Code review or `4:critic`; continue until Go.
9. Commit using Lore format.

## Recommended OMX Execution

For M1/M2, use sequential Ralph with critic gates:

```bash
omx ralph "$(cat .omx/plans/todo123-real-readiness-roadmap.md)

Execute M1 only. Do not implement rbpodo motion, Cartesian IK, gripper, action chunks, or camera/dataset work.
Stop only after tests, mock smoke, docs, and critic Go."
```

For M3 and later, use explicit team lanes because hardware/dependency/testing concerns split cleanly:

```bash
OMX_TEAM_WORKER_CLI_MAP=codex,claude,codex,claude \
omx team 4:architect "
Use .omx/plans/todo123-real-readiness-roadmap.md.
Plan M3 rbpodo hold-only implementation.
Separate implementation details from human-required hardware/dependency decisions.
Do not edit source files."
```

After M3 planning is approved:

```bash
OMX_TEAM_WORKER_CLI_MAP=codex,claude,codex,claude \
omx team 4:executor "
Implement only the approved M3 rbpodo hold-only plan.
Do not enable Cartesian/action/gripper.
Collect build/test/smoke evidence and return a Go/No-Go handoff."
```

## ADR

Decision:
Proceed milestone-by-milestone. Keep main mock-safe, refuse real motion until rbpodo state/read/send validity is proven, and defer Cartesian/action/data layers until joint-space bring-up is stable.

Drivers:

- Prevent default joint arrays from becoming motion targets.
- Keep command ingress closed until robot startup is safe.
- Preserve reviewable, reversible commits.
- Require evidence before moving from mock to rbsim to real hardware.

Alternatives considered:

- Implement rbpodo, StatePublisher, Cartesian, gripper, and action chunks in one pass.
  Rejected because it couples hardware risk, parser/protocol risk, and motion-planning risk.

- Enable rbpodo placeholder once compiled.
  Rejected because valid joint state is not proven and zero-target startup risk returns.

- Jump directly to Cartesian/TCP control.
  Rejected because joint-space real bring-up and send timing must be stable first.

Consequences:

- Real robot motion remains intentionally blocked until M3/M4 evidence exists.
- Some TODO1 future features remain planned but not executable yet.
- StatePublisher becomes the next key enabler for teleop/policy observation.

Follow-ups:

- Start M1 closeout or M2 StatePublisher first.
- Run M3 only after rbpodo dependency/version and available rbsim/real hardware decisions are known.

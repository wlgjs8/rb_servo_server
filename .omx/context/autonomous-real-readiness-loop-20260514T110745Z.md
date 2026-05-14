# Autonomous Real Readiness Loop Context

## Task statement

Run an unattended implementation and review loop for the rb_servo_server real-readiness safety work:
`4:architect -> ralph -> 4:critic -> ralph -> 4:critic ...` until a critic team returns `Go`.

## Desired outcome

- Continue implementing the remaining real-readiness plan without user interaction.
- Keep changes in small Lore-format commits.
- Add or strengthen tests for every policy or behavioral change.
- Run build/tests/smoke checks after each implementation iteration.
- Use mixed Codex/Claude team reviews where practical: `codex,claude,codex,claude`.
- Stop only when critic verdict is `Go`, a hard blocker is proven, or the configured max iteration count is reached.

## Known facts/evidence

- Current branch: `real-readiness-safety-patch`.
- Remote branch is pushed to `origin/real-readiness-safety-patch`.
- Existing commits already cover:
  - explicit `ArmMotion` gate before motion commands
  - parser drop semantics for invalid packets
  - send failure `prev_sent` handling
  - real-mode network/realtime/safety guards
  - safety policy tests and docs
- Fresh verification already passed:
  - `cmake -S . -B build && cmake --build build -j`
  - `ctest --test-dir build --output-on-failure`
  - `timeout 1s ./build/rb_servo_server --config config/dual_mock.yaml`
  - `./build/rb_servo_server --config config/dual_real.yaml` fails without `RB_ALLOW_REAL_ROBOT=1`
- Prior policy/review artifacts:
  - `.omx/debates/critic-plan-review.md`
  - `.omx/debates/pre-implementation-policy-decisions.md`

## Constraints

- No human interaction should be required during the loop.
- Do not touch real hardware.
- Do not weaken real-mode guards to make tests pass.
- Do not delete tests to make the build green.
- Avoid broad refactors unless required by a concrete critic finding.
- Preserve mock behavior and keep mock smoke runnable.
- Every commit must follow Lore commit protocol and include the OMX co-author trailer when committed by OMX tooling.

## Unknowns/open questions

- Whether Claude workers can run fully unattended depends on local Claude permission settings.
- `omx team` requires tmux runtime; the controller must run inside a tmux session.
- Real hardware and successful realtime privilege paths cannot be proven on this host without operator-controlled environment and robot access.

## Likely codebase touchpoints

- `src/control/dual_arm_servo_loop.cpp`
- `include/rb_servo/control/dual_arm_servo_loop.hpp`
- `src/network/command_server.cpp`
- `include/rb_servo/network/command_server.hpp`
- `src/config/config.cpp`
- `include/rb_servo/config/config.hpp`
- `src/core/realtime.cpp`
- `config/*.yaml`
- `tests/`
- `docs/fail_safe_policy.md`
- `docs/network_protocol.md`
- `docs/testing.md`
- `rb-servo-server.md`

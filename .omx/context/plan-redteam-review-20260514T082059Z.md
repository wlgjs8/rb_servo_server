# Context Snapshot: plan-redteam-review

## Task statement
Run `$team 4:executor` for an aggressive read-only review of the current codebase and existing plan. Do not implement. Produce final leader-authored report at `.omx/debates/plan-redteam-review.md`.

## Desired outcome
A structured debate among four roles:
1. Codex-Architect defends the existing plan with implementation/file/test feasibility.
2. Claude-RedTeam aggressively critiques hidden edge cases, races, safety, rollback, and missing tests.
3. Codex-Implementer rewrites the plan as a minimal-change implementation approach reflecting critique.
4. Claude-Verifier validates remaining failure conditions and defines a regression checklist.

## Constraints
- No source/code/config/doc implementation changes by workers.
- Read-only commands only, except leader may create `.omx/context/*` intake and final `.omx/debates/plan-redteam-review.md`.
- Each role must contribute at least 5 counterarguments.
- Include rebuttal and counter-rebuttal between roles.
- Final conclusion: exactly one of Go / No-Go / Needs More Investigation.

## Existing plan/design evidence
- Primary plan/spec: `rb-servo-server.md`.
- Overview/status: `README.md`.
- Architecture/behavior docs: `docs/architecture.md`, `docs/control_loop.md`, `docs/network_protocol.md`.
- Review/fix plan: `docs/design_review_fixes.md`.
- File maps: `docs/file_guide.md`, `docs/code_list.md`.
- Feature-specific plans: `docs/force_control.md`, `docs/camera_sync.md`, `docs/config_examples.md`.

## Codebase shape
- C++/CMake project.
- Domains: `core`, `config`, `network`, `robot`, `control`, `sensor`, `logging` under `src/` and `include/`.
- Build graph: `CMakeLists.txt` creates `rb_servo_core` library and `rb_servo_server` executable.
- Tools: `tools/send_dual_joint_sine.py`, `tools/send_dual_hold.py`, `tools/send_reset_fault.py`, `tools/plot_servo_log.py`.
- Configs: `config/dual_mock.yaml`, `config/dual_rbsim.yaml`, `config/dual_real.yaml`.

## Likely touchpoints to inspect
- Command ingress: `src/network/command_server.cpp`, `src/control/command_buffer.cpp`.
- Main loop: `src/control/dual_arm_servo_loop.cpp`.
- Filtering/safety: `src/control/trajectory_filter.cpp`, `src/control/safety_filter.cpp`.
- Backend: `src/robot/backend_factory.cpp`, `src/robot/mock_backend.cpp`, `src/robot/rbpodo_backend.cpp`.
- Observability placeholders: `src/network/state_publisher.cpp`, `src/logging/servo_logger.cpp`.
- Deferred features: `src/control/cartesian_controller.cpp`, `src/control/force_controller.cpp`, `src/sensor/*`.

## Known validation commands from docs
- `cmake -S . -B build`
- `cmake --build build -j`
- `./build/rb_servo_server --config config/dual_mock.yaml`
- `python3 tools/send_dual_joint_sine.py --rate 20 --amp-deg 2 --freq 0.2`
- `python3 tools/send_dual_hold.py`
- `python3 tools/send_reset_fault.py`
- No explicit unit-test/ctest target found during intake.

## Unknowns/open questions for workers
- Whether the current plan safely distinguishes mock/rbsim/real robot behavior and rollback boundaries.
- Whether command buffering and servo loop threading introduce races or stale command hazards.
- Whether fail-safe behavior is testable without hardware and without runtime-only manual checks.
- Whether state publishing/observability gaps block safe verification.
- Whether the existing docs overpromise force/cartesian/camera-sync features relative to scaffolding.

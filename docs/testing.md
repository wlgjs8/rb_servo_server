# Testing

Build and run the safety policy tests:

```bash
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Mock smoke test:

```bash
./build/rb_servo_server --config config/dual_mock.yaml
python3 tools/send_dual_joint_sine.py --rate 30 --amp-deg 2 --freq 0.2
```

The sine tool sends `ArmMotion` before its first `JointTarget` and waits briefly by default so a one-slot command receiver cannot lose the arm transition. The C++ `CommandBuffer` also preserves lifecycle commands separately from latest motion targets.

The smoke is meaningful only if `logs/servo_log.csv` shows `JointTarget`, `Running`, and non-trivial sent joint motion. After `EmergencyStop` and `ResetFault`, send `ArmMotion` again before motion targets.

The CSV should also contain send timing columns:

- `left_send_start_ns`, `left_send_end_ns`
- `right_send_start_ns`, `right_send_end_ns`
- `send_skew_us`
- `left_send_duration_us`, `right_send_duration_us`

Use these columns as measurement evidence before changing the sender architecture or attempting rbsim/real bring-up.

Full milestone budget checks use the stdlib analyzer:

```bash
python3 tools/analyze_servo_log.py --profile mock200 logs/servo_log.csv
python3 tools/analyze_servo_log.py --profile rbsim100 logs/servo_log.csv
```

`mock200` expects a 60 s, 200 Hz run. `rbsim100` expects a 30 s, 100 Hz run.
The analyzer fails closed on missing columns, dropped samples, send failures, bad
duration/rate/jitter/skew/send-duration budgets, and mock tracking error above
2 deg.

Real-mode guard smoke test:

```bash
./build/rb_servo_server --config config/dual_real.yaml
```

Without `RB_ALLOW_REAL_ROBOT=1`, this must fail before connecting to hardware.

rbpodo-placeholder guard smoke:

```bash
cmake -S . -B build_rbpodo_on -DRB_SERVO_ENABLE_RBPODO=ON
cmake --build build_rbpodo_on -j
RB_ALLOW_REAL_ROBOT=1 ./build_rbpodo_on/rb_servo_server --config config/dual_real.yaml
```

Until the real rbpodo state reader and servo command path are implemented, this must fail before servo loop startup.

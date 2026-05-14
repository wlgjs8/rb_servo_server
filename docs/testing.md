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

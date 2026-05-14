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

The sine tool sends `ArmMotion` before its first `JointTarget`. After `EmergencyStop` and `ResetFault`, send `ArmMotion` again before motion targets.

Real-mode guard smoke test:

```bash
./build/rb_servo_server --config config/dual_real.yaml
```

Without `RB_ALLOW_REAL_ROBOT=1`, this must fail before connecting to hardware.

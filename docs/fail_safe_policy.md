# Fail-safe Policy

This project treats every command-to-joint conversion as fallible. A failure must never become a synthesized zero pose.

## Invariant

```text
No failure path may output [0, 0, 0, 0, 0, 0] unless that was a validated user command.
```

## Failure handling table

| Situation | Behavior |
|---|---|
| stale command | Hold previous safe target |
| missing `q_target_deg` for `JointTarget` | convert to Hold |
| malformed array length | convert to Hold |
| unsupported Cartesian command | Hold previous safe target |
| future IK failure | Hold previous safe target or fault latch |
| joint command outside limits | clamp to configured limits |
| one late servo tick | filter dt is capped |
| tracking error in mock/rbsim | snap target to actual by default |
| tracking error in real | latch fault by default |
| robot disconnected/error | latch current/last-safe pose by default |
| EmergencyStop | latch current/last-safe pose |

## ResetFault

A fault latch ignores motion commands until reset.

```json
{"seq": 10, "mode": "ResetFault"}
```

After reset, the server re-baselines previous targets to the current actual q if available.

## Future Cartesian/IK rule

When `CartesianController` is implemented, its API should return an explicit result:

```cpp
struct CartesianSolveResult {
    bool ok;
    JointArray q_target_deg;
    SafetyVerdict failure_reason;
};
```

Never return a default-constructed `JointArray` on IK failure. Return `ok=false`, and let `DualArmServoLoop` hold or latch according to config.

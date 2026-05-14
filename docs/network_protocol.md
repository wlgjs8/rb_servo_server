# Network Protocol

`rb_servo_server` currently uses UDP JSON for the Python → C++ command channel.

Default:

```yaml
network:
  command_bind: "udp://127.0.0.1:50010"
```

In real mode, `udp://0.0.0.0:50010` is rejected unless `RB_ALLOW_NETWORK_EXPOSURE=1` is set.

## Important timing rule

The authoritative timestamp for timeout/staleness is the C++ receive time.

Python may send `host_time_ns` for debugging, but `CommandServer` overwrites the command's internal timestamp with `nowSteadyNs()` at packet receive time.

## Minimal Hold command

```json
{
  "seq": 1,
  "mode": "Hold",
  "timeout_sec": 0.2,
  "left": {},
  "right": {}
}
```

## Joint target command

The server must first be armed:

```json
{"seq": 1, "mode": "ArmMotion"}
```

```json
{
  "seq": 2,
  "mode": "JointTarget",
  "timeout_sec": 0.2,
  "coupled_timeout": true,
  "left": {
    "q_target_deg": [0, -30, 80, 0, 60, 0]
  },
  "right": {
    "q_target_deg": [0, -30, 80, 0, 60, 0]
  }
}
```

Top-level `mode` applies to both arms unless an arm object has its own `mode`.

## Joint velocity command

```json
{
  "seq": 3,
  "mode": "JointVelocity",
  "timeout_sec": 0.2,
  "left": {
    "dq_target_deg_s": [1, 0, 0, 0, 0, 0]
  },
  "right": {
    "dq_target_deg_s": [-1, 0, 0, 0, 0, 0]
  }
}
```

## Future Cartesian command

These fields are parsed into `ArmCommand`, but Cartesian IK is intentionally deferred.

```json
{
  "seq": 4,
  "mode": "TcpDeltaStand",
  "timeout_sec": 0.2,
  "left": {
    "tcp_delta_stand": [0.001, 0, 0, 0, 0, 0]
  },
  "right": {
    "tcp_delta_stand": [-0.001, 0, 0, 0, 0, 0]
  }
}
```

## Future force-control fields

Force-control fields are parsed, but not connected to the active joint-only path.

```json
{
  "seq": 5,
  "mode": "TcpPoseTarget",
  "timeout_sec": 0.2,
  "left": {
    "tcp_target_stand": [0.3, 0.1, 0.4, 0, 0, 0],
    "force_control": {
      "mode": "Admittance",
      "target_wrench": [0, 0, -5, 0, 0, 0],
      "enabled_axis": {"z": true},
      "max_pos_offset_m": 0.01,
      "max_pos_step_m": 0.001
    }
  },
  "right": {
    "tcp_target_stand": [0.3, -0.1, 0.4, 0, 0, 0]
  }
}
```

## Future binary protocol

UDP JSON is fine for 10–30 Hz policy commands. If command rate, action chunks, or image/state payloads grow, replace this with one of:

- shared-memory ring buffer
- ZeroMQ
- msgpack
- flatbuffers
- protobuf

## ResetFault command

A latched fault can be cleared with:

```json
{"seq": 10, "mode": "ResetFault"}
```

After reset, the server re-baselines previous targets to current actual q when available.
It stays in `ConnectedHold`; send `ArmMotion` again before motion targets.

## Missing payload safety

Motion modes require their payloads:

- `JointTarget` requires `q_target_deg` with 6 values.
- `JointVelocity` requires `dq_target_deg_s` with 6 values.
- `TcpPoseTarget` requires `tcp_target_stand` with 6 values.
- `TcpDeltaStand` requires `tcp_delta_stand` with 6 values.
- `TcpDeltaLocal` requires `tcp_delta_local` with 6 values.

If the required payload is absent or malformed, the packet is dropped and the command buffer remains unchanged.

# Viser GUI + Docker operator console

This milestone adds a browser operator console without changing ownership of the high-rate servo loop. `rb_servo_server` continues to own robot backend reads and servo sends; the GUI only consumes `StatePublisher` UDP snapshots and emits validated UDP JSON commands through the existing command protocol.

## Services and ports

Default compose stack:

- `rb_servo_gui`: Python viser web GUI, HTTP `8080`, UDP state listener `50110`.
- `rb_servo_server`: C++ server, UDP command listener `50010`, mock config for first runtime.
- `rb_sim`: profile-gated placeholder under `--profile sim`; it is `No-Go` until rbpodo/rbsim readiness tests pass.

The GUI container does **not** mount `/var/run/docker.sock`. Container start/stop controls are status/manual only until a constrained ops helper is implemented.

## Run mock GUI stack

```bash
docker compose build rb_servo_server rb_servo_gui
docker compose up rb_servo_gui rb_servo_server
```

Open <http://localhost:8080>. The server uses `config/dual_mock_compose.yaml`, which binds commands on `udp://0.0.0.0:50010` and publishes state to `udp://rb_servo_gui:50110` through Docker Compose DNS. Static container IPs are not required.

The compose server image targets the build stage so `docker compose run --rm rb_servo_server cmake ...` remains available for containerized regression checks.

Host build/test remains:

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

GUI contract tests:

```bash
PYTHONPATH=gui/rb_servo_gui python3 -m unittest discover -s gui/rb_servo_gui/tests
```

Mock smoke without a browser:

```bash
python3 tools/mock_gui_smoke.py
```

## Safety boundaries

- Real robot motion is out of scope and disabled in the GUI. Real mode is connect/status visibility only.
- Simulation motion is `No-Go` until rbpodo/rbsim connect, valid state read, truthful `servo_j` send, stop/reset, hold, and low-amplitude jog tests pass.
- TCP/Cartesian jog is visible as unavailable. Pressing the control sends no Cartesian target; it reports that FK/IK is deferred.
- Joint jog requires a fresh valid state snapshot, clamps per-command step size, sets a bounded timeout, and never synthesizes `[0,0,0,0,0,0]` when state is missing or invalid.
- Desired mode and observed server mode are separate. Selecting a desired mode without an ops surface does not claim that the running C++ process changed config.
- GUI state comes from UDP snapshots only; it does not import or read robot backends.

## Troubleshooting

- **Disconnected/stale GUI:** confirm `rb_servo_server` is running and `state_pub_bind` targets the GUI listener (`rb_servo_gui:50110` in compose, `127.0.0.1:50110` for host smoke).
- **No-Go simulation:** expected until simulator assets and rbpodo backend readiness are proven. `submodules/` remains ignored and is not part of the default build context.
- **Real guard:** motion buttons remain blocked by design. Do not use the GUI for real robot motion in this milestone.
- **Container controls disabled:** expected because the GUI has no Docker daemon authority. Use `docker compose up/down/restart` manually.

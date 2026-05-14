# Config Examples

The scaffold config parser supports the simple YAML shape used in `config/*.yaml`:

```yaml
section:
  key: value
  array_key: [1, 2, 3]
```

It is intentionally minimal. Replace with `yaml-cpp` before adding complex nested structures.

## Mock

```bash
./build/rb_servo_server --config config/dual_mock.yaml
```

Mock mode uses `MockBackend` for both arms.

## Rainbow simulator

```bash
./build/rb_servo_server --config config/dual_rbsim.yaml
```

Requires `RbpodoBackend` implementation and the Rainbow simulator endpoints.

## Real robot

```bash
RB_ALLOW_REAL_ROBOT=1 ./build/rb_servo_server --config config/dual_real.yaml
```

The environment variable is intentionally required.

## Force control

Force control is present but disabled by default:

```yaml
force_control:
  enable: false
```

Enable only after Cartesian FK/IK and F/T sensor handling are implemented and tested.

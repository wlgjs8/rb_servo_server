from __future__ import annotations

import os
import time
from typing import Any

from .command_client import CommandClient
from .safety import OperatorSafety, Readiness
from .state_receiver import StateReceiver, StateStore


def _env_int(name: str, fallback: int) -> int:
    try:
        return int(os.environ.get(name, str(fallback)))
    except ValueError:
        return fallback


def _mount_position(mounts: dict[str, Any], arm: str, fallback: tuple[float, float, float]) -> tuple[float, float, float]:
    try:
        pose = mounts[arm]["base_pose_in_stand"]
        return (float(pose.get("x", fallback[0])), float(pose.get("y", fallback[1])), float(pose.get("z", fallback[2])))
    except Exception:
        return fallback


def _format_joints(q_values: tuple[float, ...]) -> str:
    return ", ".join(f"{value:.2f}" for value in q_values)


def _joint_marker_position(base: tuple[float, float, float], q_values: tuple[float, ...]) -> tuple[float, float, float]:
    # Marker-only fallback: not FK. It gives operators visible left/right state
    # changes without pretending Cartesian kinematics are available.
    shoulder = q_values[0] / 180.0 if q_values else 0.0
    elbow = q_values[1] / 180.0 if len(q_values) > 1 else 0.0
    wrist = q_values[2] / 180.0 if len(q_values) > 2 else 0.0
    return (base[0] + 0.08 * shoulder, base[1] + 0.08 * elbow, base[2] + 0.04 + 0.06 * wrist)


def _add_scene_fallback(server: Any) -> dict[str, Any]:
    """Add stand/base frames and state-marker fallback without requiring URDF assets."""
    handles: dict[str, Any] = {}
    try:
        handles["stand"] = server.scene.add_frame("/stand", axes_length=0.25, axes_radius=0.01)
        handles["left_base"] = server.scene.add_frame("/stand/left_base", wxyz=(1.0, 0.0, 0.0, 0.0), position=(0.1601, -0.1725, 0.5825))
        handles["right_base"] = server.scene.add_frame("/stand/right_base", wxyz=(1.0, 0.0, 0.0, 0.0), position=(-0.1601, -0.1725, 0.5825))
        if hasattr(server.scene, "add_icosphere"):
            handles["left_marker"] = server.scene.add_icosphere("/stand/left_state_marker", radius=0.025, color=(80, 160, 255), position=(0.1601, -0.1725, 0.68))
            handles["right_marker"] = server.scene.add_icosphere("/stand/right_state_marker", radius=0.025, color=(255, 160, 80), position=(-0.1601, -0.1725, 0.68))
        elif hasattr(server.scene, "add_point_cloud"):
            handles["left_marker"] = server.scene.add_point_cloud("/stand/left_state_marker", points=((0.1601, -0.1725, 0.68),), colors=((80, 160, 255),), point_size=0.04)
            handles["right_marker"] = server.scene.add_point_cloud("/stand/right_state_marker", points=((-0.1601, -0.1725, 0.68),), colors=((255, 160, 80),), point_size=0.04)
    except Exception as exc:
        handles["scene_error"] = str(exc)
    return handles


def update_scene_markers(scene_handles: dict[str, Any], latest: Any) -> None:
    mounts = latest.mounts if isinstance(latest.mounts, dict) else {}
    left_base = _mount_position(mounts, "left", (0.1601, -0.1725, 0.5825))
    right_base = _mount_position(mounts, "right", (-0.1601, -0.1725, 0.5825))
    updates = {
        "left_base": left_base,
        "right_base": right_base,
        "left_marker": _joint_marker_position(left_base, latest.left.q_actual_deg),
        "right_marker": _joint_marker_position(right_base, latest.right.q_actual_deg),
    }
    for key, position in updates.items():
        handle = scene_handles.get(key)
        if handle is None:
            continue
        try:
            handle.position = position
        except Exception:
            try:
                handle.points = (position,)
            except Exception:
                pass


def build_gui(server: Any, safety: OperatorSafety, store: StateStore) -> dict[str, Any]:
    handles: dict[str, Any] = {}
    handles["scene"] = _add_scene_fallback(server)

    tabs = server.gui.add_tab_group()
    with tabs.add_tab("Status"):
        handles["connection"] = server.gui.add_text("Connection", initial_value="disconnected", disabled=True)
        handles["mode"] = server.gui.add_text("Observed mode", initial_value=safety.observed_server_mode, disabled=True)
        handles["readiness"] = server.gui.add_text("Readiness", initial_value="No-Go: no state", disabled=True)
        handles["motion"] = server.gui.add_text("Motion state", initial_value="unknown", disabled=True)
        handles["fault"] = server.gui.add_text("Fault", initial_value="none", disabled=True)
        handles["ops"] = server.gui.add_text(
            "Container ops",
            initial_value="manual compose commands only; no Docker socket in GUI",
            disabled=True,
        )
        mode_group = server.gui.add_button_group("Desired mode", ("mock", "simulation", "real"))
        handles["mode_group"] = mode_group

        @mode_group.on_click
        def _(_: Any) -> None:
            safety.set_desired_mode(mode_group.value)

    with tabs.add_tab("Lifecycle"):
        handles["lifecycle_buttons"] = {}
        for mode in ("ArmMotion", "DisarmMotion", "Hold", "EmergencyStop", "ResetFault"):
            button = server.gui.add_button(mode)
            handles[f"button_{mode}"] = button
            handles["lifecycle_buttons"][mode] = button

            @button.on_click
            def _(_: Any, mode: str = mode) -> None:
                ok, message = safety.send_lifecycle(mode)
                handles["last_action"].value = ("OK: " if ok else "BLOCKED: ") + message

        handles["last_action"] = server.gui.add_text("Last action", initial_value="none", disabled=True)

    with tabs.add_tab("Joint jog"):
        arm_group = server.gui.add_button_group("Arm", ("left", "right"))
        joint_slider = server.gui.add_slider("Joint index", min=1, max=6, step=1, initial_value=1)
        delta_slider = server.gui.add_slider("Step deg", min=-2.0, max=2.0, step=0.1, initial_value=0.5)
        jog_button = server.gui.add_button("Send bounded joint jog")
        handles["jog_button"] = jog_button
        handles["jog_status"] = server.gui.add_text("Jog status", initial_value="idle", disabled=True)

        @jog_button.on_click
        def _(_: Any) -> None:
            ok, message = safety.jog_joint(arm_group.value, int(joint_slider.value) - 1, float(delta_slider.value))
            handles["jog_status"].value = ("OK: " if ok else "BLOCKED: ") + message

    with tabs.add_tab("TCP jog (unavailable)"):
        tcp_button = server.gui.add_button("Try TCP jog", disabled=True)
        handles["tcp_button"] = tcp_button
        handles["tcp_status"] = server.gui.add_text(
            "TCP status",
            initial_value="Unavailable: FK/IK deferred; no Cartesian command will be sent",
            disabled=True,
        )

        @tcp_button.on_click
        def _(_: Any) -> None:
            _, message = safety.tcp_jog_unavailable()
            handles["tcp_status"].value = "BLOCKED: " + message

    with tabs.add_tab("Debug"):
        handles["tick"] = server.gui.add_number("tick", initial_value=0, disabled=True)
        handles["left_q"] = server.gui.add_text("left q_actual", initial_value="[]", disabled=True)
        handles["right_q"] = server.gui.add_text("right q_actual", initial_value="[]", disabled=True)
        handles["left_sent"] = server.gui.add_text("left q_sent", initial_value="[]", disabled=True)
        handles["right_sent"] = server.gui.add_text("right q_sent", initial_value="[]", disabled=True)
        handles["left_prev_sent"] = server.gui.add_text("left previous sent", initial_value="[]", disabled=True)
        handles["right_prev_sent"] = server.gui.add_text("right previous sent", initial_value="[]", disabled=True)
        handles["timestamps"] = server.gui.add_text("timestamps", initial_value="n/a", disabled=True)
        handles["timing"] = server.gui.add_text("period/jitter/filter", initial_value="n/a", disabled=True)
        handles["send_durations"] = server.gui.add_text("send durations", initial_value="n/a", disabled=True)
        handles["arm_modes"] = server.gui.add_text("arm modes/connections", initial_value="unknown", disabled=True)
        handles["logger"] = server.gui.add_text("logger health", initial_value="unknown", disabled=True)
        handles["packets"] = server.gui.add_text("state packets", initial_value="0 received / 0 invalid", disabled=True)

    return handles


def _set_disabled(handle: Any, disabled: bool) -> None:
    try:
        handle.disabled = disabled
    except Exception:
        pass


def update_gui(handles: dict[str, Any], safety: OperatorSafety, store: StateStore) -> None:
    disabled_states = safety.control_disabled_states()
    for mode, button in handles.get("lifecycle_buttons", {}).items():
        _set_disabled(button, disabled_states.get(f"lifecycle:{mode}", True))
    if "jog_button" in handles:
        _set_disabled(handles["jog_button"], disabled_states.get("jog", True))
    if "tcp_button" in handles:
        _set_disabled(handles["tcp_button"], True)

    if "mode_group" in handles:
        current_mode = handles["mode_group"].value
        if current_mode != safety.desired_mode:
            safety.set_desired_mode(current_mode)
    latest = store.latest()
    stale = store.is_stale()
    readiness = safety.readiness()
    if latest is None:
        handles["connection"].value = "disconnected/stale"
        handles["readiness"].value = readiness.no_go_reason or "No-Go: no state stream"
        handles["packets"].value = f"{store.received_packets} received / {store.invalid_packets} invalid"
        return

    handles["connection"].value = "stale" if stale else "live"
    handles["mode"].value = f"desired={safety.desired_mode}, observed={safety.observed_server_mode}"
    readiness_parts = [
        f"configured={readiness.configured}",
        f"running={readiness.running}",
        f"connected={readiness.connected}",
        f"ready={readiness.ready}",
        f"fault={readiness.fault}",
    ]
    if readiness.no_go_reason:
        readiness_parts.append("No-Go: " + readiness.no_go_reason)
    handles["readiness"].value = ", ".join(readiness_parts)
    handles["motion"].value = latest.motion_state
    handles["fault"].value = latest.fault_reason if latest.fault_latched else "none"
    update_scene_markers(handles.get("scene", {}), latest)
    handles["tick"].value = latest.tick
    handles["left_q"].value = _format_joints(latest.left.q_actual_deg)
    handles["right_q"].value = _format_joints(latest.right.q_actual_deg)
    handles["left_sent"].value = _format_joints(latest.left.q_sent_deg)
    handles["right_sent"].value = _format_joints(latest.right.q_sent_deg)
    handles["left_prev_sent"].value = _format_joints(latest.left.q_previous_sent_deg)
    handles["right_prev_sent"].value = _format_joints(latest.right.q_previous_sent_deg)
    raw = latest.raw
    handles["timestamps"].value = f"host={raw.get('host_time_ns')} loop_start={raw.get('loop_start_time_ns')} loop_end={raw.get('loop_end_time_ns')} left_host={raw.get('left', {}).get('host_time_ns')} right_host={raw.get('right', {}).get('host_time_ns')}"
    handles["timing"].value = f"period={raw.get('period_ms')} ms jitter={raw.get('jitter_ms')} ms filter={raw.get('filter_dt_ms')} ms skew={raw.get('send_skew_us')} us"
    handles["send_durations"].value = f"left={raw.get('left', {}).get('send_duration_us')} us right={raw.get('right', {}).get('send_duration_us')} us"
    handles["arm_modes"].value = f"left={latest.left.mode}/{latest.left.connection_state} right={latest.right.mode}/{latest.right.connection_state}"
    handles["logger"].value = str(latest.logger_health)
    handles["packets"].value = f"{store.received_packets} received / {store.invalid_packets} invalid"


def main() -> None:
    try:
        import viser
    except ImportError as exc:
        raise SystemExit("viser is required for the browser GUI. Install with `pip install -e gui/rb_servo_gui`.") from exc

    host = os.environ.get("RB_GUI_HOST", "0.0.0.0")
    port = _env_int("RB_GUI_PORT", 8080)
    state_host = os.environ.get("RB_GUI_STATE_BIND", "0.0.0.0")
    state_port = _env_int("RB_GUI_STATE_PORT", 50110)
    command_host = os.environ.get("RB_GUI_COMMAND_HOST", "127.0.0.1")
    command_port = _env_int("RB_GUI_COMMAND_PORT", 50010)
    observed_mode = os.environ.get("RB_GUI_OBSERVED_MODE", "mock")
    ops_available = os.environ.get("RB_GUI_OPS_AVAILABLE", "0") == "1"

    store = StateStore()
    receiver = StateReceiver(store, host=state_host, port=state_port)
    receiver.start()
    safety = OperatorSafety(
        store,
        CommandClient(command_host, command_port),
        desired_mode="mock",
        observed_server_mode=observed_mode,  # type: ignore[arg-type]
        sim_readiness=Readiness(no_go_reason="rbpodo/rbsim readiness tests have not passed"),
        ops_available=ops_available,
    )
    server = viser.ViserServer(host=host, port=port)
    handles = build_gui(server, safety, store)
    print(f"rb_servo_gui listening on http://{host}:{port}, UDP state {state_host}:{state_port}", flush=True)

    try:
        while True:
            update_gui(handles, safety, store)
            time.sleep(0.1)
    finally:
        receiver.stop()


if __name__ == "__main__":
    main()

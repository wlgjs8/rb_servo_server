from __future__ import annotations

import json
import time
import unittest

from rb_servo_gui.app import _joint_marker_position, _mount_position
from rb_servo_gui.command_client import CommandClient
from rb_servo_gui.safety import OperatorSafety, Readiness
from rb_servo_gui.state_receiver import StateStore


def sample_state(**overrides):
    arm = {
        "mode": "Hold",
        "q_actual_deg": [0, -30, 80, 0, 60, 0],
        "q_sent_deg": [0, -30, 80, 0, 60, 0],
        "q_previous_sent_deg": [0, -30, 80, 0, 60, 0],
        "send_ok": True,
        "send_start_ns": 1,
        "send_end_ns": 2,
        "send_duration_us": 1.0,
        "has_valid_joint_state": True,
        "connection_state": "Connected",
        "robot_time_ns": 1,
        "host_time_ns": 2,
        "error_code": 0,
        "tcp_stand": None,
        "tcp_base": None,
        "tcp_deferred": True,
    }
    data = {
        "schema_version": 1,
        "tick": 1,
        "loop_start_time_ns": 1,
        "loop_end_time_ns": 2,
        "host_time_ns": 2,
        "period_ms": 5.0,
        "jitter_ms": 0.0,
        "filter_dt_ms": 5.0,
        "command_seq": 1,
        "left": dict(arm),
        "right": dict(arm),
        "send_skew_us": 0.0,
        "safety_verdict": "Ok",
        "motion_state": "ConnectedHold",
        "fault_latched": False,
        "latched_fault_reason": "Ok",
        "fault_reason": "",
        "logger_health": {"ok": True, "dropped_samples": 0},
        "mounts": {},
        "tcp_fields_deferred": True,
    }
    data.update(overrides)
    return data


class RecordingClient(CommandClient):
    def __init__(self):
        super().__init__("127.0.0.1", 9)

    def send(self, packet):
        self.sent_packets.append(dict(packet))


class GuiContractsTest(unittest.TestCase):
    def make_safety(self, state=None, *, desired="mock", observed="mock", sim_ready=False):
        store = StateStore(stale_after_sec=0.5)
        if state is not None:
            self.assertTrue(store.update_from_json_bytes(json.dumps(state).encode(), received_monotonic=time.monotonic()))
        client = RecordingClient()
        safety = OperatorSafety(
            store,
            client,
            desired_mode=desired,
            observed_server_mode=observed,
            sim_readiness=Readiness(running=True, connected=True, ready=sim_ready, no_go_reason="sim readiness not proven"),
        )
        return store, client, safety

    def test_valid_state_updates_latest_and_invalid_state_is_not_motion_ready(self):
        store, _, safety = self.make_safety(sample_state())
        self.assertFalse(store.is_stale())
        self.assertEqual(store.latest().tick, 1)
        bad = sample_state(left={"q_actual_deg": [0, 0, 0], "has_valid_joint_state": True})
        self.assertFalse(store.update_from_json_bytes(json.dumps(bad).encode()))
        self.assertEqual(store.invalid_packets, 1)
        self.assertTrue(safety.readiness().ready)

    def test_stale_or_missing_state_blocks_joint_jog_without_zero_fallback(self):
        _, client, safety = self.make_safety(None)
        ok, reason = safety.jog_joint("left", 0, 1.0)
        self.assertFalse(ok)
        self.assertIn("state stream", reason)
        self.assertEqual(client.sent_packets, [])

    def test_joint_jog_uses_latest_sent_state_and_clamps_step(self):
        _, client, safety = self.make_safety(sample_state())
        ok, reason = safety.jog_joint("left", 0, 99.0)
        self.assertTrue(ok, reason)
        packet = client.sent_packets[-1]
        self.assertEqual(packet["mode"], "JointTarget")
        self.assertEqual(packet["left"]["q_target_deg"], [2.0, -30.0, 80.0, 0.0, 60.0, 0.0])
        self.assertEqual(packet["right"]["q_target_deg"], [0.0, -30.0, 80.0, 0.0, 60.0, 0.0])
        self.assertGreater(packet["timeout_sec"], 0.0)

    def test_real_mode_blocks_lifecycle_and_motion(self):
        _, client, safety = self.make_safety(sample_state(), desired="real", observed="real")
        ok, reason = safety.send_lifecycle("ArmMotion")
        self.assertFalse(ok)
        self.assertIn("connect/status only", reason)
        ok, reason = safety.jog_joint("left", 0, 1.0)
        self.assertFalse(ok)
        self.assertEqual(client.sent_packets, [])

    def test_simulation_mode_is_no_go_until_readiness_passes(self):
        _, client, safety = self.make_safety(sample_state(), desired="simulation", observed="simulation", sim_ready=False)
        ok, reason = safety.jog_joint("left", 0, 1.0)
        self.assertFalse(ok)
        self.assertIn("sim readiness", reason)
        self.assertEqual(client.sent_packets, [])

    def test_desired_mode_does_not_claim_server_hot_switch(self):
        _, client, safety = self.make_safety(sample_state(), desired="simulation", observed="mock")
        ok, reason = safety.jog_joint("left", 0, 1.0)
        self.assertFalse(ok)
        self.assertIn("desired mode differs", reason)
        safety.set_desired_mode("real")
        self.assertIn("not reconfigured", safety.status_message)
        self.assertEqual(client.sent_packets, [])

    def test_tcp_jog_never_sends_cartesian_motion(self):
        _, client, safety = self.make_safety(sample_state())
        ok, reason = safety.tcp_jog_unavailable()
        self.assertFalse(ok)
        self.assertIn("no Cartesian motion command", reason)
        self.assertEqual(client.sent_packets, [])



    def test_scene_marker_helpers_use_mounts_and_joint_state(self):
        mounts = {"left": {"base_pose_in_stand": {"x": 1.0, "y": 2.0, "z": 3.0}}}
        self.assertEqual(_mount_position(mounts, "left", (0.0, 0.0, 0.0)), (1.0, 2.0, 3.0))
        marker = _joint_marker_position((1.0, 2.0, 3.0), (180.0, -180.0, 90.0, 0.0, 0.0, 0.0))
        self.assertAlmostEqual(marker[0], 1.08)
        self.assertAlmostEqual(marker[1], 1.92)
        self.assertAlmostEqual(marker[2], 3.07)

    def test_visual_disabled_state_matches_safety_blocks(self):
        _, _, real_safety = self.make_safety(sample_state(), desired="real", observed="real")
        real_states = real_safety.control_disabled_states()
        self.assertTrue(real_states["jog"])
        self.assertTrue(real_states["tcp_jog"])
        self.assertTrue(real_states["lifecycle:ArmMotion"])
        self.assertTrue(real_states["lifecycle:Hold"])

        _, _, stale_safety = self.make_safety(None)
        stale_states = stale_safety.control_disabled_states()
        self.assertTrue(stale_states["jog"])
        self.assertTrue(stale_states["lifecycle:ArmMotion"])

        _, _, mock_safety = self.make_safety(sample_state())
        mock_states = mock_safety.control_disabled_states()
        self.assertFalse(mock_states["jog"])
        self.assertFalse(mock_states["lifecycle:ArmMotion"])
        self.assertTrue(mock_states["tcp_jog"])

        _, _, sim_safety = self.make_safety(sample_state(), desired="simulation", observed="simulation", sim_ready=False)
        sim_states = sim_safety.control_disabled_states()
        self.assertTrue(sim_states["jog"])
        self.assertTrue(sim_states["lifecycle:ArmMotion"])

    def test_lifecycle_packets_match_existing_udp_protocol(self):
        _, client, safety = self.make_safety(sample_state())
        ok, reason = safety.send_lifecycle("EmergencyStop")
        self.assertTrue(ok, reason)
        self.assertEqual(client.sent_packets[-1]["mode"], "EmergencyStop")
        self.assertEqual(client.sent_packets[-1]["left"], {})
        self.assertEqual(client.sent_packets[-1]["right"], {})


if __name__ == "__main__":
    unittest.main()

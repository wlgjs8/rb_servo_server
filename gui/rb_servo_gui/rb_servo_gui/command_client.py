from __future__ import annotations

import json
import socket
import time
from typing import Any, Mapping


class CommandClient:
    def __init__(self, host: str = "127.0.0.1", port: int = 50010) -> None:
        self.host = host
        self.port = int(port)
        self._seq = 0
        self.sent_packets: list[Mapping[str, Any]] = []

    def next_seq(self) -> int:
        self._seq += 1
        return self._seq

    def build_lifecycle(self, mode: str, *, timeout_sec: float = 0.2) -> dict[str, Any]:
        return {"seq": self.next_seq(), "mode": mode, "timeout_sec": timeout_sec, "left": {}, "right": {}}

    def build_joint_target(
        self,
        left_q: tuple[float, ...],
        right_q: tuple[float, ...],
        *,
        timeout_sec: float = 0.2,
    ) -> dict[str, Any]:
        if len(left_q) != 6 or len(right_q) != 6:
            raise ValueError("joint targets must have 6 values per arm")
        return {
            "seq": self.next_seq(),
            "mode": "JointTarget",
            "timeout_sec": timeout_sec,
            "coupled_timeout": True,
            "left": {"q_target_deg": [float(v) for v in left_q]},
            "right": {"q_target_deg": [float(v) for v in right_q]},
        }

    def send(self, packet: Mapping[str, Any]) -> None:
        payload = json.dumps(packet, separators=(",", ":")).encode("utf-8")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.sendto(payload, (self.host, self.port))
        self.sent_packets.append(dict(packet))

    def send_lifecycle(self, mode: str, *, timeout_sec: float = 0.2) -> dict[str, Any]:
        packet = self.build_lifecycle(mode, timeout_sec=timeout_sec)
        self.send(packet)
        return packet

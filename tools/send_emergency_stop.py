#!/usr/bin/env python3
"""Send EmergencyStop command to rb_servo_server over UDP JSON."""

import argparse
import json
import socket
import time


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=50010)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    msg = {
        "seq": 0,
        "mode": "EmergencyStop",
        "host_time_ns": time.monotonic_ns(),
        "left": {},
        "right": {},
    }
    sock.sendto(json.dumps(msg).encode("utf-8"), (args.host, args.port))
    print("sent EmergencyStop")


if __name__ == "__main__":
    main()

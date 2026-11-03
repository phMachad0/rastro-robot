#!/usr/bin/env python3
"""Publish a short synthetic line-loss session through the local MQTT broker."""

import argparse
import json
import select
import subprocess
import sys
import time
from datetime import datetime, timezone


CONTROL_TOPIC = "robot/session/control"
STATUS_TOPIC = "robot/session/status"
TELEMETRY_TOPIC = "robot/telemetry"


def publish(host: str, port: int, topic: str, payload: dict) -> None:
    subprocess.run(
        [
            "mosquitto_pub",
            "-h",
            host,
            "-p",
            str(port),
            "-q",
            "1",
            "-t",
            topic,
            "-m",
            json.dumps(payload, separators=(",", ":")),
        ],
        check=True,
    )


def receive_status(process: subprocess.Popen, timeout: float) -> dict:
    ready, _, _ = select.select([process.stdout], [], [], timeout)
    if not ready:
        raise TimeoutError("No collector acknowledgement received on robot/session/status")
    line = process.stdout.readline()
    if not line:
        raise RuntimeError("MQTT status subscriber stopped before receiving an acknowledgement")
    # -v prefixes each payload with its MQTT topic.
    _, separator, payload = line.partition(" ")
    if not separator:
        raise RuntimeError(f"Unexpected MQTT status line: {line.strip()!r}")
    return json.loads(payload)


def telemetry(sequence: int, uptime_ms: int, line_detected: bool) -> dict:
    on_line = [False, False, False, False, False, line_detected]
    return {
        "source": "synthetic_replay",
        "device": {"uptime_ms": uptime_ms, "sequence": sequence},
        "robot": {"mode": "RUNNING", "fault": "-"},
        "health": {
            "line_available": True,
            "imu_available": True,
            "imu_status": "READY",
            "control_available": True,
        },
        "rates": {"control_hz": 250, "ir_sample_hz": 250, "imu_sample_hz": 200},
        "ir": {
            "raw": [4050, 4050, 4050, 4050, 4050, 1300 if line_detected else 4050],
            "on_line": on_line,
            "line_detected": line_detected,
            "position": 5000 if line_detected else 0,
        },
        "imu": {
            "accel_x_g": 0.02,
            "accel_y_g": 0.01,
            "accel_z_g": 1.0,
            "gyro_z_dps": 0.0,
            "gyro_z_raw_dps": 0.0,
        },
        "motors": {"left_applied": 0.5, "right_applied": 0.5, "armed": True},
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="localhost", help="MQTT broker host (default: localhost)")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port (default: 1883)")
    parser.add_argument("--interval", type=float, default=0.5, help="Seconds between samples")
    args = parser.parse_args()

    started_session = False
    subscriber = subprocess.Popen(
        [
            "mosquitto_sub",
            "-h",
            args.host,
            "-p",
            str(args.port),
            "-t",
            STATUS_TOPIC,
            "-v",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )
    try:
        time.sleep(0.3)  # Let the status subscription connect before starting.
        label = f"line-loss-replay-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
        publish(args.host, args.port, CONTROL_TOPIC, {"action": "start", "label": label})
        status = receive_status(subscriber, timeout=8)
        if status.get("status") != "started" or not status.get("session_id"):
            raise RuntimeError(f"Collector did not start replay session: {status}")
        started_session = True
        session_id = status["session_id"]
        print(f"Started replay session {session_id} ({label})")

        # Hold line detected, lose it for 1.5 s of device time, then recover.
        samples = [
            (1, 10000, True),
            (2, 10500, False),
            (3, 11000, False),
            (4, 11500, False),
            (5, 12000, True),
            (6, 12500, True),
        ]
        for sequence, uptime_ms, detected in samples:
            publish(
                args.host,
                args.port,
                TELEMETRY_TOPIC,
                telemetry(sequence, uptime_ms, detected),
            )
            print(f"sample {sequence}: line_detected={str(detected).lower()}")
            time.sleep(args.interval)

        # Give the collector time to process the final telemetry before end.
        time.sleep(0.5)
        publish(args.host, args.port, CONTROL_TOPIC, {"action": "end"})
        status = receive_status(subscriber, timeout=8)
        if status.get("status") != "ended" or status.get("session_id") != session_id:
            raise RuntimeError(f"Collector did not end replay session cleanly: {status}")
        started_session = False
        print(f"Ended replay session {session_id}; expected recovered line-loss duration: 1500 ms")
        return 0
    except (OSError, subprocess.CalledProcessError, RuntimeError, TimeoutError, json.JSONDecodeError) as error:
        print(f"Replay failed: {error}", file=sys.stderr)
        return 1
    finally:
        if started_session:
            try:
                publish(args.host, args.port, CONTROL_TOPIC, {"action": "end"})
            except (OSError, subprocess.CalledProcessError):
                pass
        subscriber.terminate()
        try:
            subscriber.wait(timeout=2)
        except subprocess.TimeoutExpired:
            subscriber.kill()
            subscriber.wait()


if __name__ == "__main__":
    raise SystemExit(main())

import json
import logging
import os
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

import paho.mqtt.client as mqtt
from influxdb_client import InfluxDBClient, Point, WriteOptions


logging.basicConfig(
    level=os.getenv("LOG_LEVEL", "INFO"),
    format="%(asctime)s %(levelname)s %(message)s",
)
log = logging.getLogger("robot-telemetry-collector")

MQTT_HOST = os.environ["MQTT_HOST"]
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_TOPIC = os.getenv("MQTT_TOPIC", "robot/telemetry")
MQTT_SESSION_CONTROL_TOPIC = os.getenv(
    "MQTT_SESSION_CONTROL_TOPIC", "robot/session/control"
)
MQTT_SESSION_STATUS_TOPIC = os.getenv(
    "MQTT_SESSION_STATUS_TOPIC", "robot/session/status"
)
DEVICE_ID = os.getenv("DEVICE_ID", "rastro-robot-1")
INFLUX_URL = os.getenv("INFLUX_URL", "http://influxdb:8086")
INFLUX_ORG = os.environ["INFLUX_ORG"]
INFLUX_BUCKET = os.environ["INFLUX_BUCKET"]
INFLUX_TOKEN = os.environ["INFLUX_TOKEN"]
SESSION_STATE_PATH = Path(
    os.getenv("SESSION_STATE_PATH", "/var/lib/telemetry-collector/session.json")
)

active_session = None
last_line_observation = None
active_line_loss = None


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def load_active_session():
    try:
        session = json.loads(SESSION_STATE_PATH.read_text(encoding="utf-8"))
        if not isinstance(session, dict) or not isinstance(
            session.get("session_id"), str
        ):
            raise ValueError("invalid session state")
        log.info("Resumed active telemetry session %s", session["session_id"])
        return session
    except FileNotFoundError:
        return None
    except (OSError, json.JSONDecodeError, ValueError) as error:
        log.error("Could not load persisted session state: %s", error)
        return None


def persist_active_session(session):
    SESSION_STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = SESSION_STATE_PATH.with_suffix(".tmp")
    temporary_path.write_text(json.dumps(session), encoding="utf-8")
    os.replace(temporary_path, SESSION_STATE_PATH)


def session_event_point(session: dict, event_type: str) -> Point:
    point = (
        Point("robot_session_events")
        .tag("device", DEVICE_ID)
        .tag("session_id", session["session_id"])
        .tag("event_type", event_type)
        .field("event_count", 1)
        .time(utc_now())
    )
    point.field("label", session.get("label", ""))
    return point


def publish_session_status(client, status: str, **details):
    payload = {"status": status, "timestamp": utc_now().isoformat(), **details}
    client.publish(
        MQTT_SESSION_STATUS_TOPIC,
        json.dumps(payload, separators=(",", ":")),
        qos=1,
        retain=False,
    )


def handle_session_control(client, write_api, message):
    global active_session, last_line_observation, active_line_loss

    if message.retain:
        publish_session_status(client, "error", reason="retained_commands_not_allowed")
        log.warning("Ignored retained session control message")
        return

    try:
        command = json.loads(message.payload.decode("utf-8"))
        if not isinstance(command, dict):
            raise ValueError("command must be a JSON object")
        action = command.get("action")
        if action not in ("start", "end"):
            raise ValueError("action must be 'start' or 'end'")

        if action == "start":
            if active_session is not None:
                publish_session_status(
                    client,
                    "error",
                    reason="session_already_active",
                    session_id=active_session["session_id"],
                )
                return
            label = command.get("label", "")
            if not isinstance(label, str) or len(label) > 120:
                raise ValueError("label must be a string of at most 120 characters")
            started_at = utc_now()
            active_session = {
                "session_id": (
                    f"{started_at.strftime('%Y%m%dT%H%M%SZ')}-{uuid4().hex[:8]}"
                ),
                "label": label.strip(),
                "started_at": started_at.isoformat(),
            }
            starting_session = active_session
            try:
                persist_active_session(starting_session)
            except OSError:
                active_session = None
                raise
            last_line_observation = None
            active_line_loss = None
            write_api.write(
                bucket=INFLUX_BUCKET,
                org=INFLUX_ORG,
                record=session_event_point(active_session, "start"),
            )
            log.info("Started telemetry session %s", active_session["session_id"])
            publish_session_status(client, "started", **active_session)
            return

        if active_session is None:
            publish_session_status(client, "error", reason="no_active_session")
            return

        ending_session = active_session
        close_line_loss_at_session_end(write_api, ending_session, utc_now())
        write_api.write(
            bucket=INFLUX_BUCKET,
            org=INFLUX_ORG,
            record=session_event_point(ending_session, "end"),
        )
        active_session = None
        last_line_observation = None
        active_line_loss = None
        SESSION_STATE_PATH.unlink(missing_ok=True)
        log.info("Ended telemetry session %s", ending_session["session_id"])
        publish_session_status(
            client,
            "ended",
            session_id=ending_session["session_id"],
            label=ending_session.get("label", ""),
        )
    except (UnicodeDecodeError, json.JSONDecodeError, OSError, ValueError, TypeError) as error:
        log.warning("Rejected session control message: %s", error)
        publish_session_status(client, "error", reason=str(error))


def add_field(point: Point, name: str, value):
    if isinstance(value, (bool, int, float, str)):
        point.field(name, value)


def add_float_field(point: Point, name: str, value):
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        point.field(name, float(value))


def telemetry_point(data: dict, session_id: str) -> Point:
    robot = data.get("robot") or {}
    point = (
        Point("robot_telemetry")
        .tag("device", DEVICE_ID)
        .tag("session_id", session_id)
        .tag("source", str(data.get("source", "unknown")))
        .tag("mode", str(robot.get("mode", "UNKNOWN")))
        .tag("fault", str(robot.get("fault", "UNKNOWN")))
        # Influx time remains collector receipt time; device uptime and sequence
        # are stored as fields so device-side intervals and missing slots can
        # be measured independently.
        .time(datetime.now(timezone.utc))
    )

    device = data.get("device") or {}
    health = data.get("health") or {}
    rates = data.get("rates") or {}
    ir = data.get("ir") or {}
    imu = data.get("imu") or {}
    motors = data.get("motors") or {}
    control = data.get("control") or {}

    add_field(point, "device_uptime_ms", device.get("uptime_ms"))
    add_field(point, "device_sequence", device.get("sequence"))

    for name in (
        "line_available",
        "imu_available",
        "control_available",
    ):
        add_field(point, name, health.get(name))
    add_field(point, "imu_status", health.get("imu_status"))

    for name in ("control_hz", "ir_sample_hz"):
        add_field(point, name, rates.get(name))
    add_float_field(point, "imu_sample_hz", rates.get("imu_sample_hz"))

    raw = ir.get("raw") or []
    on_line = ir.get("on_line") or []
    for index, value in enumerate(raw):
        add_field(point, f"ir_raw_{index}", value)
    for index, value in enumerate(on_line):
        add_field(point, f"ir_on_line_{index}", value)
    add_field(point, "line_detected", ir.get("line_detected"))
    add_field(point, "line_position", ir.get("position"))

    if isinstance(imu, dict):
        for name in (
            "accel_x_g",
            "accel_y_g",
            "accel_z_g",
            "gyro_z_dps",
            "gyro_z_raw_dps",
        ):
            add_float_field(point, name, imu.get(name))

    for name in ("left_applied", "right_applied"):
        add_float_field(point, f"motor_{name}", motors.get(name))
    add_field(point, "motor_armed", motors.get("armed"))
    for name in ("base_speed", "steering_action"):
        add_float_field(point, name, control.get(name))

    return point


def line_loss_event_point(
    loss: dict,
    status: str,
    duration_ms: float = 0.0,
    duration_source: str = "pending",
    recovered_at: str = "",
    open_observation_interval_ms: float = 0.0,
    recovery_data: dict | None = None,
    event_time: datetime | None = None,
) -> Point:
    point = (
        Point("robot_line_loss_events")
        .tag("device", DEVICE_ID)
        .tag("session_id", loss["session_id"])
        .tag("source", loss["source"])
        .time(event_time or loss["started_at"])
        .field("status", status)
        .field("start_basis", loss["start_basis"])
        .field("loss_duration_ms", float(duration_ms))
        .field("duration_source", duration_source)
        .field("recovered_at", recovered_at)
        .field("open_observation_interval_ms", float(open_observation_interval_ms))
        .field("start_mode", loss["mode"])
        .field("start_fault", loss["fault"])
    )
    for name, value in loss["context"].items():
        add_float_field(point, f"start_{name}", value)
    if recovery_data:
        robot = recovery_data.get("robot") or {}
        imu = recovery_data.get("imu") or {}
        motors = recovery_data.get("motors") or {}
        point.field("recovery_mode", str(robot.get("mode", "UNKNOWN")))
        point.field("recovery_fault", str(robot.get("fault", "UNKNOWN")))
        add_float_field(point, "recovery_gyro_z_dps", imu.get("gyro_z_dps"))
        add_float_field(point, "recovery_motor_left_applied", motors.get("left_applied"))
        add_float_field(point, "recovery_motor_right_applied", motors.get("right_applied"))
    return point


def line_loss_transition(data: dict, session_id: str, received_at: datetime):
    """Track observed line-loss edges and return an updated episode point."""
    global last_line_observation, active_line_loss

    ir = data.get("ir") or {}
    detected = ir.get("line_detected")
    if not isinstance(detected, bool):
        return None

    source = str(data.get("source", "unknown"))
    identity = session_id
    previous = last_line_observation
    last_line_observation = {"identity": identity, "detected": detected}

    if session_id == "unassigned":
        active_line_loss = None
        return None

    new_observation = previous is None or previous["identity"] != identity
    starts_loss = (new_observation and not detected) or (
        not new_observation and previous["detected"] and not detected
    )
    if starts_loss:
        robot = data.get("robot") or {}
        imu = data.get("imu") or {}
        motors = data.get("motors") or {}
        device = data.get("device") or {}
        uptime = device.get("uptime_ms")
        active_line_loss = {
            "session_id": session_id,
            "source": source,
            "started_at": received_at,
            "device_uptime_ms": uptime if isinstance(uptime, int) and not isinstance(uptime, bool) else None,
            "start_basis": "initial_observation" if new_observation else "observed_transition",
            "mode": str(robot.get("mode", "UNKNOWN")),
            "fault": str(robot.get("fault", "UNKNOWN")),
            "context": {
                "gyro_z_dps": imu.get("gyro_z_dps"),
                "motor_left_applied": motors.get("left_applied"),
                "motor_right_applied": motors.get("right_applied"),
            },
        }
        return line_loss_event_point(active_line_loss, "open")

    if (
        not new_observation
        and not previous["detected"]
        and detected
        and active_line_loss is not None
        and active_line_loss["session_id"] == session_id
    ):
        device = data.get("device") or {}
        current_uptime = device.get("uptime_ms")
        start_uptime = active_line_loss["device_uptime_ms"]
        if (
            isinstance(start_uptime, int)
            and not isinstance(start_uptime, bool)
            and isinstance(current_uptime, int)
            and not isinstance(current_uptime, bool)
            and current_uptime >= start_uptime
        ):
            duration_ms = float(current_uptime - start_uptime)
            duration_source = "device_uptime"
        else:
            duration_ms = max(
                0.0,
                (received_at - active_line_loss["started_at"]).total_seconds() * 1000.0,
            )
            duration_source = "collector_receipt"
        event = line_loss_event_point(
            active_line_loss,
            "recovered",
            duration_ms=duration_ms,
            duration_source=duration_source,
            recovered_at=received_at.isoformat(),
            recovery_data=data,
        )
        active_line_loss = None
        return event

    return None


def close_line_loss_at_session_end(write_api, session: dict, ended_at: datetime):
    global active_line_loss
    if (
        active_line_loss is None
        or active_line_loss["session_id"] != session["session_id"]
    ):
        return
    open_interval_ms = max(
        0.0,
        (ended_at - active_line_loss["started_at"]).total_seconds() * 1000.0,
    )
    write_api.write(
        bucket=INFLUX_BUCKET,
        org=INFLUX_ORG,
        record=line_loss_event_point(
            active_line_loss,
            "unresolved_at_session_end",
            duration_source="collector_receipt_interval",
            open_observation_interval_ms=open_interval_ms,
        ),
    )
    active_line_loss = None


def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        log.error("MQTT connection failed: %s", reason_code)
        return
    client.subscribe([(MQTT_TOPIC, 0), (MQTT_SESSION_CONTROL_TOPIC, 1)])
    log.info(
        "Subscribed to telemetry %s and session control %s at %s:%s",
        MQTT_TOPIC,
        MQTT_SESSION_CONTROL_TOPIC,
        MQTT_HOST,
        MQTT_PORT,
    )


def on_message(client, write_api, message):
    try:
        data = json.loads(message.payload.decode("utf-8"))
        if not isinstance(data, dict) or not isinstance(data.get("ir"), dict):
            raise ValueError("expected a telemetry object with an ir group")
        session_id = (
            active_session["session_id"] if active_session else "unassigned"
        )
        received_at = utc_now()
        write_api.write(
            bucket=INFLUX_BUCKET,
            org=INFLUX_ORG,
            record=telemetry_point(data, session_id).time(received_at),
        )
        event_point = line_loss_transition(data, session_id, received_at)
        if event_point is not None:
            write_api.write(
                bucket=INFLUX_BUCKET,
                org=INFLUX_ORG,
                record=event_point,
            )
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError, TypeError) as error:
        log.warning("Skipping invalid MQTT payload: %s", error)


def main():
    global active_session
    active_session = load_active_session()

    with InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG) as influx:
        write_api = influx.write_api(
            write_options=WriteOptions(batch_size=50, flush_interval=1000)
        )
        client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        client.on_connect = on_connect
        def route_message(mqtt_client, userdata, message):
            if message.topic == MQTT_SESSION_CONTROL_TOPIC:
                handle_session_control(mqtt_client, write_api, message)
            elif message.topic == MQTT_TOPIC:
                on_message(mqtt_client, write_api, message)

        client.on_message = route_message
        log.info("Connecting to MQTT broker %s:%s", MQTT_HOST, MQTT_PORT)
        client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
        try:
            client.loop_forever()
        finally:
            client.disconnect()
            write_api.close()


if __name__ == "__main__":
    main()

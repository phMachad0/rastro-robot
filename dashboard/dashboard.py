import json
import threading
import time
from collections import deque

import pandas as pd
import streamlit as st
import paho.mqtt.client as mqtt

from config import BROKER, PORT, TOPIC


st.set_page_config(
    page_title="ESP32 Robot Telemetry",
    page_icon="🤖",
    layout="wide",
)


class MQTTTelemetry:
    def __init__(self):
        self.lock = threading.Lock()

        self.telemetry = {}
        self.history = deque(maxlen=500)

        self.connected = False
        self.client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self.client.connect(BROKER, PORT, 60)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, reason_code, properties):
        print(f"Connected to MQTT broker: {reason_code}")
        self.connected = not reason_code.is_failure
        client.subscribe(TOPIC)

    def on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties):
        print(f"Disconnected from MQTT broker: {reason_code}")
        self.connected = False

    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode("utf-8"))
            if not isinstance(data, dict) or not isinstance(data.get("ir"), dict) or not isinstance(data.get("motors"), dict):
                return
            with self.lock:
                self.telemetry = data
                self.history.append({"timestamp": time.time(), **data})
        except (json.JSONDecodeError, UnicodeDecodeError):
            print("Received invalid JSON")

    def get_data(self):
        with self.lock:
            current = self.telemetry.copy()
            history = list(self.history)
        return current, history


@st.cache_resource
def get_mqtt():
    return MQTTTelemetry()


mqtt_data = get_mqtt()

st.title("🤖 ESP32 Robot Telemetry")
current, history = mqtt_data.get_data()

if mqtt_data.connected:
    st.success("MQTT connected")
else:
    st.error("MQTT disconnected")

if not current:
    st.warning("Waiting for telemetry...")
    time.sleep(1)
    st.rerun()

if history and time.time() - history[-1]["timestamp"] > 3:
    st.warning("No recent readings received. Values below are the last received sample.")
if current.get("source") == "synthetic":
    st.info("Synthetic demo values — no sensor or motor readings are coming from hardware.")

ir = current["ir"]
imu = current.get("imu")
motors = current["motors"]
robot = current.get("robot", {})
health = current.get("health", {})
rates = current.get("rates", {})

st.subheader("Robot health")
health_cols = st.columns(4)
health_cols[0].metric("Mode", robot.get("mode", "Unknown"))
fault = robot.get("fault", "Unknown")
health_cols[1].metric("Fault", "None" if fault == "-" else fault)
health_cols[2].metric("Line input", "Available" if health.get("line_available") else "Missing / stale")
imu_status = health.get("imu_status", "UNKNOWN")
if not health.get("imu_available") and imu_status == "READY":
    imu_status = "STALE"
health_cols[3].metric("IMU", imu_status)

st.caption(f"Line detected: {'yes' if ir.get('line_detected') else 'no'}")
rate_cols = st.columns(3)
for col, label, key, available in (
    (rate_cols[0], "Control rate", "control_hz", health.get("control_available")),
    (rate_cols[1], "IR sample rate", "ir_sample_hz", health.get("line_available")),
    (rate_cols[2], "IMU sample rate", "imu_sample_hz", health.get("imu_available")),
):
    value = rates.get(key, 0)
    col.metric(label, f"{value:.1f} Hz" if available else "Unavailable")

st.subheader("IR array")
raw = ir.get("raw", [])
on_line = ir.get("on_line", [])
for i, col in enumerate(st.columns(len(raw) or 1)):
    if i < len(raw):
        col.metric(f"IR {i} · ADC", raw[i])
        col.caption("Line" if i < len(on_line) and on_line[i] else "Background")
st.metric("Line position", ir.get("position") if ir.get("position") is not None else "Unavailable")
st.caption("Position: 0 = left, 5000 = right. No position is shown when the line is lost.")

st.subheader("MPU-6050")
if not imu:
    st.warning("MPU readings unavailable: check sensor, calibration and data-ready connection.")
else:
    for col, axis in zip(st.columns(3), "xyz"):
        col.metric(f"Acceleration {axis.upper()} (g)", f"{imu[f'accel_{axis}_g']:.3f}")
    st.metric("Yaw rate (°/s)", f"{imu['gyro_z_dps']:.3f}")
    st.metric("Gyro Z before firmware filter (°/s)", f"{imu['gyro_z_raw_dps']:.3f}")

st.subheader("Motor outputs")
left, right = st.columns(2)
left.metric("Left applied command", f"{motors['left_applied'] * 100:.1f}%")
right.metric("Right applied command", f"{motors['right_applied'] * 100:.1f}%")
if current.get("source") == "synthetic":
    st.caption(f"Demo motor state: {'simulated armed' if motors['armed'] else 'simulated disarmed'}. No motor driver is connected.")
else:
    st.caption(f"Driver: {'armed' if motors['armed'] else 'disarmed'}. Signed commands after trim/saturation; not measured wheel speed or PWM duty.")

if history:
    rows = []
    for item in history:
        row = {"timestamp": item["timestamp"]}
        row.update({f"IR {i}": value for i, value in enumerate(item["ir"]["raw"])})
        row.update(item.get("imu") or {})
        row.update({"Left command (%)": item["motors"]["left_applied"] * 100,
                    "Right command (%)": item["motors"]["right_applied"] * 100})
        rows.append(row)
    df = pd.DataFrame(rows)
    df["timestamp"] = pd.to_datetime(df["timestamp"], unit="s")
    df = df.set_index("timestamp")
    st.subheader("IR readings (ADC)")
    st.line_chart(df.filter(regex=r"^IR "))
    st.subheader("Acceleration (g)")
    st.line_chart(df.reindex(columns=[f"accel_{axis}_g" for axis in "xyz"]))
    st.subheader("Gyroscope (°/s)")
    st.line_chart(df.reindex(columns=["gyro_z_dps", "gyro_z_raw_dps"]))
    st.subheader("Applied motor commands (%)")
    st.line_chart(df[["Left command (%)", "Right command (%)"]])

time.sleep(1)
st.rerun()

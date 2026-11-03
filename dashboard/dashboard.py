import json
import threading
import time
from collections import deque

import pandas as pd
import streamlit as st
import paho.mqtt.client as mqtt
import altair as alt

from config import BROKER, COMMAND_TOPIC, PORT, TOPIC


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

    def send_command(self, cmd):
        self.client.publish(COMMAND_TOPIC, cmd, qos=1)

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

mode = current.get("robot", {}).get("mode")

def on_switch():
    if st.session_state.run_switch:
        if mode == "FAULT":
            # Commands share one ordered MQTT topic. The firmware queues both,
            # so the supervisor processes RESET before START.
            mqtt_data.send_command("reset")
        mqtt_data.send_command("start")
    else:
        mqtt_data.send_command("stop")


# Assigning session state in code does not fire on_change, so this only syncs the display.
st.session_state.run_switch = mode == "RUNNING"
switch_col, mode_col = st.columns([1, 5])
switch_col.toggle(
    "Robot",
    key="run_switch",
    on_change=on_switch,
    disabled=mode not in ("IDLE", "RUNNING", "FAULT"),
)
mode_col.caption(f"Mode: {mode or 'no telemetry'}")

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
if raw:
    adc_view = st.radio(
        "IR chart scale",
        ["Zoomed · 2500–4095", "Full · 0–4095"],
        horizontal=True,
        label_visibility="collapsed",
    )
    adc_min = 2500 if adc_view.startswith("Zoomed") else 0
    ir_frame = pd.DataFrame({
        "Sensor": [f"IR {i}" for i in range(len(raw))],
        "ADC": raw,
        "Displayed ADC": [max(value, adc_min) for value in raw],
        "Baseline": [adc_min] * len(raw),
        "Line detected": [bool(on_line[i]) if i < len(on_line) else False for i in range(len(raw))],
    })
    ir_chart = alt.Chart(ir_frame).mark_bar().encode(
        x=alt.X("Displayed ADC:Q", title=f"ADC reading · {adc_min}–4095", scale=alt.Scale(domain=[adc_min, 4095])),
        x2=alt.X2("Baseline:Q"),
        y=alt.Y("Sensor:N", title=None, sort=[f"IR {i}" for i in range(len(raw))]),
        color=alt.Color(
            "Line detected:N",
            title="Surface",
            scale=alt.Scale(domain=[True, False], range=["#f97316", "#38bdf8"]),
        ),
        tooltip=["Sensor", "ADC", alt.Tooltip("Line detected:N", title="Line")],
    ).properties(height=220)
    st.altair_chart(ir_chart, use_container_width=True)
    if adc_min:
        st.caption("Zoomed scale: readings below 2500 are shown at the left edge; hover a bar for its actual value.")
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
control = current.get("control", {})
command_frame = pd.DataFrame({
    "Side": ["Left", "Right"],
    "Command (%)": [motors["left_applied"] * 100, motors["right_applied"] * 100],
})
command_chart = alt.Chart(command_frame).mark_bar().encode(
    x=alt.X(field="Command (%)", type="quantitative", title="Applied command (%)", scale=alt.Scale(domain=[-100, 100]), axis=alt.Axis(tickCount=5)),
    y=alt.Y("Side:N", title=None, sort=["Left", "Right"]),
    color=alt.Color("Side:N", scale=alt.Scale(domain=["Left", "Right"], range=["#38bdf8", "#a78bfa"]), legend=None),
    tooltip=["Side", alt.Tooltip(field="Command (%)", type="quantitative", format=".1f")],
).properties(height=100)
zero_rule = alt.Chart(pd.DataFrame({"zero": [0]})).mark_rule(color="#94a3b8").encode(
    x=alt.X(field="zero", type="quantitative")
)
st.altair_chart(command_chart + zero_rule, use_container_width=True)
left, right = st.columns(2)
left.metric("Left applied command", f"{motors['left_applied'] * 100:.1f}%")
right.metric("Right applied command", f"{motors['right_applied'] * 100:.1f}%")
base_speed_col, steering_col = st.columns(2)
base_speed_col.metric("Requested base speed", f"{control.get('base_speed', 0) * 100:.1f}%")
steering_col.metric("Steering correction", f"{control.get('steering_action', 0) * 100:+.1f}%")
if current.get("source") == "synthetic":
    st.caption(f"Demo motor state: {'simulated armed' if motors['armed'] else 'simulated disarmed'}. No motor driver is connected.")
else:
    st.caption(f"Driver: {'armed' if motors['armed'] else 'disarmed'}. Signed commands after trim/saturation; not measured wheel speed or PWM duty.")

if history:
    rows = []
    for item in history:
        row = {"timestamp": item["timestamp"]}
        row.update({f"IR {i}": value for i, value in enumerate(item["ir"]["raw"])})
        row["Line position"] = item["ir"].get("position")
        row.update(item.get("imu") or {})
        row.update({"Left command (%)": item["motors"]["left_applied"] * 100,
                    "Right command (%)": item["motors"]["right_applied"] * 100})
        control_sample = item.get("control") or {}
        row.update({"Base speed (%)": control_sample.get("base_speed", 0) * 100,
                    "Steering correction (%)": control_sample.get("steering_action", 0) * 100})
        rows.append(row)
    df = pd.DataFrame(rows)
    df["timestamp"] = pd.to_datetime(df["timestamp"], unit="s")
    df = df.set_index("timestamp")

    st.subheader("Line position over time")
    position_history = df.reset_index()
    position_chart = alt.Chart(position_history).mark_line(
        invalid="break-paths-show-path-domains"
    ).encode(
        x=alt.X("timestamp:T", title=None),
        y=alt.Y(
            field="Line position",
            type="quantitative",
            title="Sensor-relative position",
            scale=alt.Scale(domain=[0, 5000]),
        ),
        tooltip=[
            "timestamp:T",
            alt.Tooltip(field="Line position", type="quantitative", format=".0f"),
        ],
    ).properties(height=180)
    st.altair_chart(position_chart, use_container_width=True)
    st.caption("Position is relative to the sensor array (0 = left, 5000 = right). Gaps indicate that the line was not detected; this is not a world-space trajectory.")

    with st.expander("IR sensor history", expanded=False):
        ir_columns = [f"IR {i}" for i in range(len(raw))]
        ir_history = df[ir_columns].reset_index().melt(
            id_vars="timestamp", var_name="Sensor", value_name="ADC"
        )
        ir_history_chart = alt.Chart(ir_history).mark_line().encode(
            x=alt.X("timestamp:T", title=None),
            y=alt.Y("ADC:Q", title="ADC", scale=alt.Scale(domain=[0, 4095])),
            tooltip=["timestamp:T", "Sensor:N", "ADC:Q"],
        ).properties(height=90).facet(
            facet=alt.Facet("Sensor:N", sort=ir_columns),
            columns=2,
        ).resolve_scale(y="shared")
        st.altair_chart(ir_history_chart, use_container_width=True)

    st.subheader("Acceleration (g)")
    st.line_chart(df.reindex(columns=[f"accel_{axis}_g" for axis in "xyz"]))
    st.subheader("Gyroscope (°/s)")
    st.line_chart(df.reindex(columns=["gyro_z_dps", "gyro_z_raw_dps"]))
    st.subheader("Applied motor commands (%)")
    st.line_chart(df[["Left command (%)", "Right command (%)"]])
    st.subheader("Controller inputs (%)")
    st.line_chart(df[["Base speed (%)", "Steering correction (%)"]])

time.sleep(1)
st.rerun()

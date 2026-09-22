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
        self.connected = True
        client.subscribe(TOPIC)

    def on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties):
        print(f"Disconnected from MQTT broker: {reason_code}")
        self.connected = False

    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode("utf-8"))
            with self.lock:
                self.telemetry = data
                self.history.append({"timestamp": time.time(), **data})
        except json.JSONDecodeError:
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

col1, col2, col3, col4, col5, col6 = st.columns(6)
col1.metric("Wi-Fi RSSI", f"{current.get('rssi', 0)} dBm")
col2.metric("Control Error", f"{current.get('control_error', 0):.3f}")
col3.metric("Left Motor", f"{current.get('motor_left', 0)} %")
col4.metric("Right Motor", f"{current.get('motor_right', 0)} %")
col5.metric("Loop time", f"{current.get('loop_time_us', 0)} us")
col6.metric("Control period", f"{current.get('control_period_us', 0)} us")

st.subheader("Line Sensors")
sensor1, sensor2, sensor3 = st.columns(3)
sensor1.metric("Left", current.get("sensor_left", 0))
sensor2.metric("Center", current.get("sensor_center", 0))
sensor3.metric("Right", current.get("sensor_right", 0))

if history:
    df = pd.DataFrame(history)
    df["timestamp"] = pd.to_datetime(df["timestamp"], unit="s")
    df = df.set_index("timestamp")

    st.subheader("Control Error")
    st.line_chart(df["control_error"])

    st.subheader("Motor Commands")
    st.line_chart(df[["motor_left", "motor_right"]])

    st.subheader("Line Sensors")
    st.line_chart(df[["sensor_left", "sensor_center", "sensor_right"]])

uptime_seconds = current.get("uptime_ms", 0) / 1000
st.caption(f"ESP32 uptime: {uptime_seconds:.1f} s")

time.sleep(1)
st.rerun()

# Local telemetry history

This optional stack records the existing `robot/telemetry` MQTT stream and
provides historical queries in Grafana. It does not replace Streamlit or change
the robot control path.

## First-time setup and start

Prerequisites: Docker Engine with the Compose plugin, `mosquitto_pub` and
`mosquitto_sub` on the host, and an MQTT broker reachable by both the ESP32 and
the collector container.

From this directory, create the stack environment file and set `MQTT_HOST` to
the broker address as seen from Docker. If Mosquitto runs on this same PC,
`host.docker.internal` is the default. If it runs on another machine, use that
machine's reachable IP or hostname. Replace the local InfluxDB and Grafana
passwords/tokens with unique values:

```sh
cp .env.example .env
chmod 600 .env
```

The ESP32 has separate, ignored local config files. From this directory, make
them from their examples and set the Wi-Fi credentials and broker address the
ESP32 itself can reach:

```sh
cp ../../include/secrets.example.h ../../include/secrets.h
cp ../../include/network_config.example.h ../../include/network_config.h
```

The broker addresses in `network_config.h` and `.env` can differ: the first is
used by the ESP32, the second by the collector container.

Start the broker first, then bring up the observability services:

```sh
docker compose up --build -d
docker compose ps
docker compose logs -f collector
```

The collector log should say it subscribed to `robot/telemetry` and
`robot/session/control`. Leave this terminal running or use `Ctrl+C` to return
to the shell; that only stops following the logs.

The stack listens only on the local machine:

- Grafana: <http://localhost:3001>
- InfluxDB: <http://localhost:8086>

The default Grafana login is controlled by `GRAFANA_ADMIN_USER` and
`GRAFANA_ADMIN_PASSWORD` in `.env`. The provisioned dashboards are **Rastro
Robot telemetry** (session summary, state-change timeline, named status cards,
and time-series view)
and **Rastro Robot session inspector** (timestamped sample table). Both use
the same Session selector; the dashboard files and InfluxDB datasource load
automatically.

The telemetry dashboard's session profile summarizes received samples,
scheduled-slot coverage, missing slots, largest sample gap, line-loss episode
count and observed duration, sensor-relative line-position variability,
corrected yaw-rate RMS and peak, left/right motor-command imbalance, and the
share of commands near saturation. The Grafana time picker is in the top-right
toolbar; select a time window there. Use the toolbar's refresh menu to pause
automatic refresh while inspecting a run. The state timeline shows
line detection, subsystem availability, motor arming, robot mode, and fault
values over the selected time range. The **Line loss episodes** table records
each observed loss with its start context, recovery status, and duration. It
uses device uptime when both transition samples provide it and falls back to
collector receipt time otherwise. Loss and recovery times are bounded by the
telemetry cadence and any message gaps; the event table's status distinguishes
recovered episodes from losses still open or unresolved at session end. The
collector's Influx `_time` remains the message receipt time. The collector
records episodes as telemetry arrives; it does not reconstruct this event
measurement from older stored samples.

The Grafana dashboard has a **Session** selector and a **Session boundaries**
table. Start and end sessions by publishing control messages to MQTT; Streamlit
does not relay telemetry or session commands. The collector saves the active
session in a Docker volume, so restarting the collector does not split a run.
Telemetry received outside a session is retained with `session_id=unassigned`
and omitted from the Grafana session selector.

Open one terminal to receive session acknowledgements:

```sh
mosquitto_sub -h localhost -p 1883 -t robot/session/status -v
```

In another terminal, start a named experiment:

```sh
mosquitto_pub -h localhost -p 1883 -q 1 \
  -t robot/session/control \
  -m '{"action":"start","label":"track-test-01"}'
```

The status topic returns the generated `session_id`. End the experiment with:

```sh
mosquitto_pub -h localhost -p 1883 -q 1 \
  -t robot/session/control \
  -m '{"action":"end"}'
```

The local commands use `localhost` because they run on the host. If Mosquitto
runs on this PC, the collector uses `host.docker.internal` to reach it from
Docker; for a broker on another machine, set `MQTT_HOST` in `.env` to its
Docker-reachable address. The collector also writes start/end markers to the
`robot_session_events` measurement. Messages must not be retained. If a start
is sent while another session is active, the collector reports an error and
keeps the existing session active. Previously captured automatic sessions
remain selectable; their session-boundary table is empty because those runs
predate explicit start/end markers.

## Lab run checklist

1. Connect the ESP32 and confirm its Wi-Fi network can reach the broker. From
   the repository root, flash the telemetry-only demo and open its serial
   monitor:

   ```sh
   pio run -e test_telemetry -t upload
   pio device monitor -e test_telemetry
   ```

   If `pio` is not on `PATH`, use the local PlatformIO executable:
   `/home/vinicius/.platformio/penv/bin/pio` in both commands.

   The test firmware publishes synthetic sensor samples and does not require
   the robot to drive. The serial monitor should show Wi-Fi/MQTT connection
   messages.

2. Check that messages arrive at the broker from a host terminal (replace the
   address with the broker host):

   ```sh
   mosquitto_sub -h BROKER_HOST -p 1883 -t robot/telemetry -v
   ```

3. In a second terminal, watch session acknowledgements:

   ```sh
   mosquitto_sub -h BROKER_HOST -p 1883 -t robot/session/status -v
   ```

4. Start a named run using the same broker address:

   ```sh
   mosquitto_pub -h BROKER_HOST -p 1883 -q 1 \
     -t robot/session/control \
     -m '{"action":"start","label":"lab-run-01"}'
   ```

   Confirm the status subscriber reports `started` and note its `session_id`.
   Let the experiment run, then end it:

   ```sh
   mosquitto_pub -h BROKER_HOST -p 1883 -q 1 \
     -t robot/session/control \
     -m '{"action":"end"}'
   ```

5. Open <http://localhost:3001>, select the new session in Grafana, and inspect
   **Line loss episodes**, the state timeline, and the session profile. If the
   session is not in the selector yet, refresh the dashboard.

6. Stop the stack from this directory with `docker compose down`. This keeps
   recorded data. Do not use `docker compose down -v` unless you intend to
   delete the InfluxDB data and Grafana state.

For the first smoke test, flash `test_telemetry`; its payload is tagged
`source=synthetic` so it can be filtered separately from real robot runs. The
collector timestamps each MQTT message when this PC receives it. The payload
also carries `device.uptime_ms` and `device.sequence`: uptime identifies device
time intervals, while sequence advances on every scheduled telemetry slot,
including slots missed while Wi-Fi or MQTT is unavailable. In InfluxDB these are
stored as `device_uptime_ms` and `device_sequence`. Grafana uses them to show
missing scheduled slots and the largest positive device sample interval. The
sequence restarts at device reboot, and uptime intervals spanning a reboot are
excluded from the interval summary.

When the robot is unavailable, replay a short synthetic line-loss session
through the local broker:

```sh
python3 tests/replay_line_loss.py
```

The replay uses `mosquitto_pub` and `mosquitto_sub` from the host, starts and
ends its own labeled session, and publishes `source=synthetic_replay` samples
for detected → lost → recovered. It exits without publishing telemetry if the
collector rejects the session start. To use a broker on another host, pass
`--host` and optionally `--port`. Select the new session in Grafana; the
**Line loss episodes** table should show one recovered event with a device-time
duration of 1,500 ms.

## Browse all stored telemetry

In InfluxDB, open **Data Explorer**, select the `robot-telemetry` bucket, and
choose the `robot_telemetry` measurement. The `_field` column lists the stored
values, while tags include `device`, `session_id`, `source`, `mode`, and
`fault`. Use the query editor in Flux mode to inspect the full schema or rows:

```flux
import "influxdata/influxdb/schema"

schema.measurementFieldKeys(
  bucket: "robot-telemetry",
  measurement: "robot_telemetry",
  start: -24h,
)
```

```flux
from(bucket: "robot-telemetry")
  |> range(start: -24h)
  |> filter(fn: (r) => r._measurement == "robot_telemetry")
  |> limit(n: 1000)
```

The first query lists fields seen in the last 24 hours; the second shows up to
1,000 field rows. InfluxDB stores each field as a separate row in query results.

## Storage and shutdown

InfluxDB starts with a 24-hour bucket retention. The compose services have
memory limits totaling 1.75 GiB. Measure actual disk use after a representative
run before increasing retention.

Stop services and keep recorded data:

```sh
docker compose down
```

Remove services **and all recorded InfluxDB/Grafana data**:

```sh
docker compose down -v
```

The `.env` file is ignored by Git. Do not commit it.

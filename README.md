# Atlas Flight Software

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-5.5-red.svg)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/)
[![Target](https://img.shields.io/badge/target-ESP32--S3-blue.svg)](https://www.espressif.com/en/products/socs/esp32-s3)
[![FreeRTOS](https://img.shields.io/badge/RTOS-FreeRTOS-green.svg)](https://www.freertos.org/)
[![LoRa](https://img.shields.io/badge/radio-LoRa%20E220-orange.svg)](https://www.cdebyte.com/)

Flight computer firmware of the **Aurora Rocketry Team** (Università di Bologna).
Atlas runs on the team's **Manny** board (ESP32-S3, 16 MB flash). It reads the
on-board sensors, runs the flight state machine, fires the recovery charges,
records every flight to external flash and sends live telemetry over LoRa. Before
launch, the board hosts a Wi-Fi web dashboard used to check, configure and update it.

---

## Table of Contents

- [Features](#features)
- [Hardware](#hardware)
- [Repository Layout](#repository-layout)
- [Architecture](#architecture)
- [Flight State Machine](#flight-state-machine)
- [Recovery](#recovery)
- [Ground Services (Web Dashboard)](#ground-services-web-dashboard)
- [Telemetry](#telemetry)
- [Flight Recorder](#flight-recorder)
- [Getting Started](#getting-started)
- [Configuration](#configuration)
- [Hardware-in-the-Loop Simulation](#hardware-in-the-loop-simulation)
- [Post-Flight Analysis](#post-flight-analysis)
- [Hardware Test Routine](#hardware-test-routine)
- [Documentation](#documentation)
- [Contributing](#contributing)
- [License](#license)

---

## Features

- **12-state flight FSM**: calibration → ground services → ready → launch → powered and ballistic flight → apogee → recovery → recovered. Each state starts only the FreeRTOS tasks it needs.
- **Barometric apogee detection**: a median filter smooths the pressure altitude, and a least-squares fit over a sliding window estimates vertical velocity. Apogee also has a time lockout and a time backstop.
- **One- or two-parachute recovery**: the recovery outputs fire as configurable pulses.
- **Runtime configuration stored in NVS**: flight parameters can be edited from the dashboard. They are validated, then locked into an immutable snapshot at ready-for-launch.
- **Ground Services web app**: the board serves a dashboard over its own Wi-Fi SoftAP, with live data, health diagnostics, a pre-launch checklist, guided hardware tests, a serial monitor, flight-file download and OTA updates. Critical actions must be HMAC-signed.
- **OTA with rollback**: the flash holds two app slots. The bootloader reverts a new image unless that image finishes setup and marks itself valid.
- **LoRa downlink**: a packed 67-byte telemetry frame goes out through an EByte E220 module.
- **Flight recorder**: each flight is written as JSONL to external SPI NOR flash (LittleFS), in a new file per flight.
- **Hardware-in-the-loop**: a build profile swaps the real sensors for a RocketPy simulation driven over TCP.

---

## Hardware

### Manny flight computer

| Item | Part / interface | Notes |
|------|------------------|-------|
| MCU | ESP32-S3-WROOM-1-N16 | 16 MB flash, dual core |
| IMU | Bosch **BNO055** (I2C, `0x29`) | 9-DOF with on-chip fusion; primary source for launch detection |
| Accelerometer | ST **LIS3DHTR** (I2C) | Secondary accelerometer |
| Barometer | TE **MS5611** (`MS561101BA03`, SPI) | A second barometer is supported but not fitted on Manny |
| External flash | SPI NOR, LittleFS at `/ext` | Flight recorder storage |
| Radio | EByte **E220-900T22D** (UART) | 868 MHz band LoRa telemetry and command uplink |
| GNSS | NMEA receiver (UART) | Driver present; pins not assigned on Manny |
| SD card | SPI | Driver present; not fitted on Manny |
| Recovery outputs | GPIO 5 (drogue), GPIO 4 (main) | Valid only with a 6.4 V board supply |
| Status | RGB LED (18/8/7), buzzer (21) | Status codes via `StatusManager` |

Manny's full pin map is in [`components/model/boards_hardware/manny/board.h`](components/model/boards_hardware/manny/board.h).
Manny has no arming input, so `MannyBoard::is_armed()` always returns `true`.

### Adding a board

Board-specific code sits behind [`IBoardHardware`](components/model/boards_hardware/IBoardHardware.hpp).
The board class owns the pin map, the shared I2C/SPI buses, NVS and the Wi-Fi SoftAP.
To add a board:

1. Implement `IBoardHardware`.
2. Add it to the `COMPILE_BOARD` choice in `main/Kconfig.projbuild`.
3. Alias it as `Board`.

---

## Repository Layout

```
atlas/
├── CMakeLists.txt            # ESP-IDF project; component list; AURORA_HIL_ENABLED switch
├── sdkconfig.defaults        # Target (esp32s3), FreeRTOS, partitions, console, HTTPD WS
├── partitions.csv            # nvs, otadata, phy_init, coredump, ota_0, ota_1 (7.5 MB each)
├── main/
│   ├── app_entry.cpp         # app_main(): selects flight or HIL setup/loop
│   ├── main.cpp              # Flight firmware bring-up
│   ├── main_hil.cpp          # HIL firmware bring-up (only built when HIL is enabled)
│   ├── Kconfig.projbuild     # Board, Ground Services secret, SoftAP settings
│   ├── idf_component.yml     # arduino-esp32, littlefs
│   ├── hil/                  # RocketPy HIL simulator, configs, captures (see its README)
│   └── dashboard/            # Legacy serial dashboards (Dash/Flask)
├── components/
│   ├── common/data/          # Sensor data types, ISensor, HIL command packet
│   ├── control/              # RocketFSM, TransitionManager, tasks, RuntimeConfig, web UI
│   │   └── src/tasks/web/    # Ground Services front end (embedded into the firmware)
│   ├── global/               # config.h, pins.h, SerialLogger, telemetry field names
│   ├── model/                # RocketModel (shared sensor/state store) + board hardware
│   ├── sensors/              # BNO055, LIS3DHTR, MS561101BA03 (MS5611), GPS
│   ├── telemetry/            # Packet protocol + E220 / SX126x LoRa transmitters
│   ├── logger/               # RocketLogger ring buffer + JSON payload serializers
│   ├── persistence/          # IStorage, MirrorStorage, Flash (LittleFS), SD
│   ├── protocols/            # I2CBus, SPIBus wrappers
│   ├── status/               # LED, buzzer, StatusManager
│   ├── tests/                # TestRoutine: guided hardware tests
│   └── third_party/          # Vendored libraries (RadioLib, E220, Eigen, TinyEKF, ...)
├── data_analysis/            # Flight-log analysis and flash dump tools
├── test/                     # Legacy manual sensor printers (Arduino/PlatformIO era)
└── Doxyfile                  # API documentation config
```

---

## Architecture

### Boot sequence (`main/main.cpp`)

1. Initialize the board, LEDs, buzzer and status patterns.
2. Initialize NVS and load `RuntimeConfig`. If this fails, the firmware keeps running on the compiled defaults.
3. Create the sensors (BNO055, MS5611, LIS3DHTR, GPS), the SD card and the external flash. A sensor that fails to initialize is logged and skipped.
4. Build the `RocketLogger` (JSON serializer), the `RocketModel` and the `TestRoutine`.
5. Wait for a GPS fix if a GPS is present, with a 3-minute timeout.
6. Create and start `RocketFSM`.
7. Mark a pending OTA image as valid, which cancels rollback.
8. Loop forever. The main loop logs a heartbeat every 5 s (heap, task stacks, logger usage) and the FSM state.

### Components

- **`RocketModel`** is the thread-safe store for the latest sensor readings and the estimated state (altitude, velocity, max altitude, rising flag, deployment commands). Tasks write to it and read from it. Its storage goes through a `MirrorStorage` that combines SD and flash.
- **`RocketFSM`** runs a 50 Hz FreeRTOS task. It checks automatic transitions, handles queued events and diffs the task sets on each transition: tasks that are not needed are stopped and new ones are started. A task watchdog guards the FSM task.
- **`TaskManager`** owns one instance of each task type and monitors stack usage.

### Tasks

| Task | Rate | Core | Role |
|------|------|------|------|
| `SensorTask` | 50 Hz | 0 | Polls BNO055, LIS3DHTR, MS5611 into `RocketModel`, with health checks |
| `GpsTask` | event-driven | 1 | Reads the NMEA parser into `RocketModel` |
| `AltitudeTask` | 50 Hz (configurable) | 0 | Pressure → altitude, median filter, OLS vertical velocity, apogee detection |
| `StorageLoggingTask` | ~12.5 Hz | 1 | Batches serialized log entries (4 KB buffer, 1 s flush) to the flight file |
| `TelemetryTask` | 500 ms | 1 | Builds and sends `TelemetryPacket` over LoRa; publishes link status |
| `GroundServicesTask` | — | 1 | HTTP/WebSocket server for the dashboard |
| `HilSimulationTask` | lockstep | 0 | Replaces Sensor/GPS tasks in HIL builds (TCP server on port 5000) |
| `AirbrakesTask` | — | 0 | Implemented and configurable, but currently disabled in every state |

---

## Flight State Machine

States are declared in [`FlightState.hpp`](components/control/src/FlightState.hpp).
Detection logic is in `RocketFSM::checkTransitions()` in [`RocketFSM.cpp`](components/control/src/RocketFSM.cpp).
All thresholds below are defaults and can be changed through `RuntimeConfig`.

| # | State | Exit condition (default) | Active tasks |
|---|-------|--------------------------|--------------|
| 0 | `INACTIVE` | Immediately → `CALIBRATING` | — |
| 1 | `CALIBRATING` | Barometer zeroed (100 samples) **and** IMU calibrated, or a 10 s timeout | Sensor, GPS, Telemetry |
| 2 | `GROUND_SERVICES` | An authenticated *Ready for launch* request from the dashboard. The FSM then prepares the flight file and locks the config | Ground Services, Sensor, GPS, Telemetry |
| 3 | `READY_FOR_LAUNCH` | Acceleration magnitude above 3 g for 250 ms | Sensor, GPS, Altitude, Telemetry |
| 4 | `LAUNCH` | Immediately → `ACCELERATED_FLIGHT` | + Storage |
| 5 | `ACCELERATED_FLIGHT` | 4000 ms after launch detection | Sensor, GPS, Altitude, Storage, Telemetry |
| 6 | `BALLISTIC_FLIGHT` | After a 5000 ms lockout, when the altitude is no longer rising, or 6700 ms after launch at the latest | same |
| 7 | `APOGEE` | Recovery fires on entry. Exits after the drogue delay (0 ms) | same |
| 8 | `STABILIZATION` | Altitude below 50 m AGL. Main fires on exit in two-parachute mode | same |
| 9 | `DECELERATION` | Altitude below 15 m AGL | same |
| 10 | `LANDING` | 2 s | same |
| 11 | `RECOVERED` | Terminal. Recorder stopped | Sensor, GPS, Telemetry |

An `EMERGENCY_ABORT` event returns the FSM to `INACTIVE` from any state.
`FORCE_TRANSITION` jumps directly to the target state.

---

## Recovery

`RecoveryConfig.mode` sets the recovery mode. The compiled default is
`AURORA_RECOVERY_MODE` in `config.h`, and the mode can be edited from the dashboard.

| Mode | At `APOGEE` | On leaving `STABILIZATION` |
|------|-------------|----------------------------|
| `OneParachuteMode` (default) | Fire **main and drogue** outputs (single chute wired to both) | — |
| `TwoParachuteMode` | Fire **drogue** | Fire **main** |

Each output fires as `pulse_count` pulses of `pulse_duration_ms` each (default 3 × 3 ms), and at most once per flight.

---

## Ground Services (Web Dashboard)

Ground Services runs only in the `GROUND_SERVICES` state. The board starts a Wi-Fi
SoftAP (default SSID `Aurora AP`, IP `192.168.4.1`) and serves a web app. The web
app lives in [`components/control/src/tasks/web/`](components/control/src/tasks/web/)
and is compiled into the firmware.

| Page | Purpose |
|------|---------|
| Info | Identity, FSM state, firmware details, pre-launch checklist, *Ready for launch* |
| Health | Sensors, hardware, FreeRTOS tasks, memory, HTTP/WebSocket and LoRa diagnostics |
| Live Data | Current sensor values, time series, acceleration and attitude (`/ws/live-data`) |
| Config | Edit and inspect `RuntimeConfig`, with validation and schema-driven forms |
| OTA | Upload and verify a firmware image, then reboot into it |
| Tests | Guided hardware tests from `TestRoutine`, with operator verdicts |
| Serial Monitor | Live device logs (`/ws/logs`) with pause, filter and copy |
| Files | List, stream-download and delete flight recorder files |

**Security.** Wi-Fi access and operator authentication are separate:

- Monitoring endpoints are read-only and open to anyone on the SoftAP.
- Mutating endpoints need a single-use nonce from `/api/auth/nonce`, plus an **HMAC-SHA256** signature computed with the shared secret `CONFIG_GROUND_SERVICES_AUTH_TOKEN`.
- Destructive actions also need a matching `X-Confirm` header, such as `READY_FOR_LAUNCH` or `REBOOT_TO_NEW_FIRMWARE`.
- An OTA upload blocks all other mutations until it finishes.

The REST API is registered in `GroundServicesTask::registerHandlers()`. Its
endpoints live under `/api/status`, `/api/health`, `/api/config/*`,
`/api/prelaunch/checklist`, `/api/fsm/ready-for-launch`, `/api/tests/*`,
`/api/ota/*` and `/api/files`.

---

## Telemetry

`TelemetryTask` sends a packed, little-endian `TelemetryPacket` (67 bytes) every
500 ms. The packet is defined in [`TelemetryTask.hpp`](components/control/src/tasks/TelemetryTask.hpp):

```cpp
#pragma pack(push, 1)
struct TelemetryPacket {
    uint32_t timestamp;                 // ms since boot
    bool     dataValid;
    struct { float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z; } imu;
    struct { float pressure, temperature; } baro1, baro2;   // hPa, °C
    float    baro_altitude;             // m above launch point
    struct { float latitude, longitude, altitude; } gps;
    float    velocity;                  // m/s
    uint8_t  flight_phase;              // RocketState index (0 = INACTIVE … 11 = RECOVERED)
    uint8_t  last_ack_command_id;       // last command received on the uplink
};
#pragma pack(pop)
```

The E220 driver is configured in [`E220LoRaTransmitter.cpp`](components/telemetry/radio_lora/src/E220LoRaTransmitter.cpp):

- Fixed transmission to the receiver address set in `config.h`
- 9.6 kbps air data rate
- 17 dBm transmit power
- RSSI enabled

The *E220 Configuration* test writes this configuration into the module.

---

## Flight Recorder

- **Format.** The recorder writes JSON Lines. `PayloadSerializers::toJson` turns each `RocketLogger` entry (sensor samples, estimator state, FSM transitions, events) into one line.
- **Storage.** Files go to external SPI flash (LittleFS, mounted at `/ext`) through `MirrorStorage`. If an SD card is present, the same data is also written there.
- **Per-flight files.** Each flight gets its own file (`flight_telemetry_NNNNNN.jsonl`). The file name is reserved and saved to NVS before the config is locked, so a reboot never overwrites a previous flight.
- **Recording window.** The recorder runs from `LAUNCH` through `LANDING` and stops in `RECOVERED`.
- **Retrieval.**
  - From the dashboard's **Files** page.
  - Over serial: run the *Dump JSONL telemetry from Flash* test and capture the output with `data_analysis/extract_flash_logs.py`.

---

## Configuration

Configuration lives in four places:

| Where | What | How to change |
|-------|------|---------------|
| `RuntimeConfig` (NVS) | Mission info, flight thresholds, altitude filter / apogee detector, recovery mode, actuator pulses, airbrakes. Calibration and telemetry values can be viewed but not edited | Dashboard **Config** page (before ready-for-launch) |
| [`components/global/src/config.h`](components/global/src/config.h) | Compiled defaults for the above, LoRa addresses/channel, GPS fix timeout, sensor addresses | Edit and rebuild |
| `main/Kconfig.projbuild` | Board selection, USB-JTAG driver, Ground Services secret, SoftAP settings | `idf.py menuconfig` |
| Root `CMakeLists.txt` | `AURORA_HIL_ENABLED` build profile | Edit and rebuild |

On a READY request, the FSM validates `RuntimeConfig`, saves it with a checksum and
**locks** it into a flight snapshot. All flight-critical tasks read that snapshot,
so dashboard edits cannot change parameters during a flight. After recovery, you can
unlock the config from the dashboard. The comment block in
[`RuntimeConfig.hpp`](components/control/src/tasks/RuntimeConfig.hpp) explains how
to add a new parameter.

---

## Hardware-in-the-Loop Simulation

To build the HIL firmware, set the switch in the root `CMakeLists.txt`:

```cmake
set(AURORA_HIL_ENABLED ON)   # OFF = flight firmware with no HIL code at all
```

In a HIL build, `HilSimulationTask` replaces the Sensor and GPS tasks. The task runs
a TCP server on port 5000 over the SoftAP. A RocketPy simulator on the PC sends it
IMU, barometer and GNSS samples in lockstep, and receives the FSM state and the
recovery and airbrake commands in return.

```bash
cd main/hil
python hil_rocketpy.py --rocket fred --sensor-profile clean      # or: nemesis, noisy, very_noisy
python hil_capture.py hil_captures/<capture>.json                # replot / 3D replay
python mock_manny_fc_server.py                                   # run without hardware
```

Rocket configurations are in `main/hil/config/` (`fred`, `nemesis`).
[`main/hil/README.md`](main/hil/README.md) covers the patched RocketPy setup, the
config format, the sensor profiles, reference frames, the wire protocol and the
capture format.

---

## Post-Flight Analysis

```bash
# Plots (altitude, velocity, acceleration, pressure, FSM timeline, ...) and interactive 3D replay
python data_analysis/analyze_flight_log.py flash_logs/flight_telemetry_000001.jsonl --replay

# Save the figures instead of showing them
python data_analysis/analyze_flight_log.py <file.jsonl> --save-dir plots --no-show

# Capture a flash dump streamed over serial by the "Dump JSONL telemetry from Flash" test
python data_analysis/extract_flash_logs.py --port COM5 --out flash_logs
```

`data_analysis/AnalyzeJSONs.ipynb` is a notebook for exploring the logs.

---

## Hardware Test Routine

[`TestRoutine`](components/tests/src/TestRoutine.cpp) provides guided checks. You can
run them from the dashboard's **Tests** page, or at boot by defining
`ENABLE_TEST_ROUTINE` in `main/main.cpp`. The checks are:

- Power and LEDs
- Sensors
- Actuators
- SD card
- I2C scan
- E220 configuration and connector checks
- Telemetry transmission
- LoRa command reception
- Flash memory test, format and dump
- BNO055 calibration saved to NVS

Tests that are destructive, such as flash format or actuator firing, need explicit confirmation.

---

## Contributing

1. Branch from `main`: `git checkout -b feature/<short-name>`.
2. Name commits with a bracketed scope, following the history: `[FSM] ...`, `[GROUND] ...`, `[HIL] ...`, `[STORAGE] ...`.
3. Open a pull request. The PR template asks for a test plan and a quality checklist.

Guidelines:

- Write all code, comments and strings in English.
- Add Doxygen comments to new classes and methods.
- Tasks must read flight-critical values from `runtime_config_get_flight_snapshot()`, not straight from `config.h`.
- Use bounded, static buffers in long-running tasks. The HTTP server and the flight tasks share a tight internal-RAM budget.
- Test on hardware or in HIL before merging changes to flight logic.

To report a bug or request a feature, use the issue templates.

---

## License

This project is released under the **MIT License**. See [LICENSE](LICENSE).

The vendored libraries in `components/third_party/` keep their own licenses:

- RadioLib
- EByte LoRa E220 library
- SX126x-Arduino
- Eigen
- TinyEKF
- SdFat
- Adafruit BusIO / Unified Sensor
- Bosch BNO055 SensorAPI
- nlohmann/json
- ArxTypeTraits
- NMEA0183 parser
- lis3dh

---

<div align="center">

**Aurora Rocketry Team**, Università di Bologna

</div>

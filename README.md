<p align="center">
  <h1 align="center">F1Tenth Low-Level ESP32 Controller</h1>
  <p align="center">
    Real-time low-level firmware for an F1Tenth autonomous racing platform — built with ESP32, FreeRTOS, PlatformIO & micro-ROS.
  </p>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS%202-Jazzy-blue?style=flat-square&logo=ros" alt="ROS 2 Jazzy" />
  <img src="https://img.shields.io/badge/Platform-ESP32-black?style=flat-square&logo=espressif" alt="ESP32" />
  <img src="https://img.shields.io/badge/Build-PlatformIO-orange?style=flat-square&logo=platformio" alt="PlatformIO" />
  <img src="https://img.shields.io/badge/RTOS-FreeRTOS-green?style=flat-square" alt="FreeRTOS" />
  <img src="https://img.shields.io/badge/micro--ROS-Serial-9cf?style=flat-square" alt="micro-ROS" />
  <img src="https://img.shields.io/badge/License-MIT-yellow?style=flat-square" alt="License" />
</p>

---

## Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [Hardware & Pin Configuration](#hardware--pin-configuration)
- [System Architecture](#system-architecture)
  - [Dual-Core Task Layout](#dual-core-task-layout)
  - [State Machine](#state-machine)
- [ROS 2 Interface](#ros-2-interface)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
  - [1. Host Setup](#1-host-setup)
  - [2. Build & Upload Firmware](#2-build--upload-firmware)
  - [3. Launch the micro-ROS Agent](#3-launch-the-micro-ros-agent)
- [Configuration & Tuning](#configuration--tuning)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

**F1Tenth Low-Level ESP32 Controller** is the real-time firmware layer for an [F1Tenth](https://f1tenth.org/) autonomous racing vehicle. Running on an **ESP32 (uPesy WROOM)**, it bridges the gap between a high-level ROS 2 navigation stack and the physical actuators — a **steering servo** and a **brushless DC motor** (via ESC).

The firmware communicates with a ROS 2 host over **micro-ROS serial transport**, receiving steering and throttle commands while publishing filtered steering-angle feedback. Deterministic control is achieved through **FreeRTOS dual-core task partitioning**, ensuring that the 100 Hz control loop never contends with the micro-ROS communication stack.

---

## Key Features

| Feature | Description |
|:--------|:------------|
| **micro-ROS Integration** | Real-time bidirectional communication with ROS 2 (Jazzy) via serial transport. Includes a robust auto-reconnect state machine that handles agent disconnections gracefully. |
| **Dual-Core Processing** | Core 1 runs the micro-ROS executors, timers, and subscriber callbacks. Core 0 runs the high-frequency control task (100 Hz / 10 ms period) for deterministic, jitter-free actuation. |
| **Thread-Safe Design** | Cross-core shared variables are protected by FreeRTOS Mutexes with dedicated logging utilities for debug traceability. |
| **Hybrid Signal Filtering** | A **3-Tap Median Filter** followed by a **1D Kalman Filter** (with angle + velocity state estimation) cleans noisy analog ADC feedback from the steering angle potentiometer. |
| **PID Control** | Custom PID controller with **anti-windup clamping**, **deadband compensation**, configurable dt guards, and output limiting for precise steering servo positioning. |
| **Automated Bringup** | A Python ROS 2 node (`bringup_robot.py`) auto-detects the ESP32 via USB-UART, launches the micro-ROS agent in a Docker container, monitors its health, and auto-restarts it on crash. |

---

## Hardware & Pin Configuration

| Component | GPIO Pin | Notes |
|:----------|:--------:|:------|
| Brushless Motor PWM (ESC) | `23` | PWM signal to ESC · Range: 1000–2000 µs · Neutral: 1500 µs |
| Steering Servo | `22` | PWM signal · Range: 500–2000 µs · Center: 1500 µs |
| Steering Angle Sensor (ADC) | `34` | Analog input · 12-bit resolution (0–4095) · 3.3 V range |

> **Board:** uPesy ESP32 WROOM · **Clock:** 240 MHz · **ADC Attenuation:** 11 dB (full 0–3.3 V range)

---

## System Architecture

### Dual-Core Task Layout

```
┌─────────────────────────────────────────────────────────────────┐
│                        ESP32 (240 MHz)                          │
├───────────────────────────┬─────────────────────────────────────┤
│        CORE 0             │            CORE 1                   │
│   (Control Task)          │   (micro-ROS / Arduino loop)        │
│                           │                                     │
│  ┌─────────────────────┐  │  ┌───────────────────────────────┐  │
│  │  100 Hz Control Loop │  │  │  micro-ROS State Machine      │  │
│  │  ─────────────────── │  │  │  ─────────────────────────    │  │
│  │  • Read steering cmd │◄─┼──│  • Subscriber callbacks       │  │
│  │  • Read throttle cmd │◄─┼──│    /steering_command          │  │
│  │  • PID compute       │  │  │    /throttle                  │  │
│  │  • Servo actuation   │  │  │  • Publisher timer (20 Hz)    │  │
│  │  • ESC actuation     │  │  │    /steering_angle            │  │
│  │  • vTaskDelayUntil   │  │  │  • Agent ping & reconnect    │  │
│  └─────────────────────┘  │  └───────────────────────────────┘  │
│                           │                                     │
│    FreeRTOS Mutexes ◄─────┼──────► FreeRTOS Mutexes             │
│  (servoSetPointMutex)     │  (steeringAngleReadingMutex)        │
│  (throttleCommandMutex)   │                                     │
└───────────────────────────┴─────────────────────────────────────┘
```

### State Machine

The micro-ROS connection lifecycle is managed by a four-state state machine running on Core 1:

```
 ┌──────────────┐   Agent found    ┌─────────────────┐
 │ WAITING_AGENT├─────────────────►│ AGENT_AVAILABLE  │
 └──────┬───────┘                  └────────┬─────────┘
        ▲                                   │
        │  Entities destroyed               │ create_entities()
        │                                   ▼
 ┌──────┴───────────┐  Ping fail   ┌────────────────┐
 │ AGENT_DISCONNECTED│◄────────────│ AGENT_CONNECTED │
 └──────────────────┘              └────────────────┘
                                    Spin executors
                                    Ping every 200ms
```

---

## ROS 2 Interface

### Subscriptions

| Topic | Type | Description |
|:------|:-----|:------------|
| `/steering_command` | `std_msgs/msg/Float32` | Desired steering angle setpoint (degrees) |
| `/throttle` | `std_msgs/msg/Float32` | Throttle command in range **[-1.0, 1.0]** (maps to 1000–2000 µs ESC pulse) |

### Publications

| Topic | Type | Rate | Description |
|:------|:-----|:----:|:------------|
| `/steering_angle` | `std_msgs/msg/Float32` | 20 Hz | Filtered steering angle feedback from the ADC sensor (degrees) |

### Node Name

```
micro_ros_platformio_node
```

---

## Project Structure

```
F1_Tenth_Low_level_ESP/
├── bringup_robot.py            # ROS 2 node: auto-detect ESP32 & launch micro-ROS agent
├── setup_micro_ros.sh          # Host setup script (pyudev, Docker image, USB permissions)
├── platformio.ini              # PlatformIO build configuration
│
├── config/                     # Tuning & hardware configuration macros
│   ├── esp32_config.h          #   Clock speed, ADC resolution, attenuation
│   ├── filter_config.h         #   Kalman filter process/measurement noise (Q, R)
│   ├── pins_config.h           #   GPIO pin assignments
│   ├── servo_config.h          #   PID gains, PWM limits, control frequency
│   └── uros_config.h           #   micro-ROS helper macros (RCCHECK, timing)
│
├── lib/                        # Modular firmware libraries
│   ├── brushless_lib/          #   ESC/brushless motor control
│   │   ├── brushless_control.h
│   │   ├── brushless_control.cpp
│   │   └── motor_config.h      #     Throttle PWM range (1000/1500/2000 µs)
│   ├── filter_lib/             #   Hybrid signal filter (Median + Kalman)
│   │   ├── filter_lib.h
│   │   └── filter_lib.cpp
│   ├── pid_lib/                #   PID controller with anti-windup & deadband
│   │   ├── pid_lib.h
│   │   └── pid_lib.cpp
│   ├── rtos_utils_lib/         #   Thread-safe mutex logging helpers
│   │   ├── rtos_utils_lib.h
│   │   └── rtos_utils_lib.c
│   └── Servo_lib/              #   Steering servo control & ADC feedback
│       ├── servo_control.h
│       └── servo_control.cpp
│
├── src/
│   └── main.cpp                # Application entry point, FreeRTOS tasks, micro-ROS setup
│
└── test/                       # Unit tests (PlatformIO test framework)
```

---

## Prerequisites

| Dependency | Purpose |
|:-----------|:--------|
| [PlatformIO](https://platformio.org/) | Build system for compiling and uploading firmware to the ESP32 |
| [Docker](https://docs.docker.com/get-docker/) | Runs the `microros/micro-ros-agent:jazzy` container |
| Python 3 + `rclpy` | Required by the `bringup_robot.py` launch node |
| ROS 2 Jazzy (host) | Needed for the bringup script and high-level stack |
| USB-UART driver | CP2102 driver (usually included in Linux kernel) |

---

## Quick Start

### 1. Host Setup

Run the provided setup script to install host dependencies, pull the micro-ROS Docker image, and configure USB permissions:

```bash
chmod +x setup_micro_ros.sh
./setup_micro_ros.sh
```

This script will:
- Install `python3-pyudev` (for USB device detection)
- Pull `microros/micro-ros-agent:jazzy` Docker image
- Set permissions on `/dev/ttyUSB*` devices

### 2. Build & Upload Firmware

Using PlatformIO CLI:

```bash
# Build the firmware
pio run

# Upload to the ESP32 (connect via USB first)
pio run --target upload

# (Optional) Open serial monitor to view debug output
pio device monitor -b 115200
```

Or using **VS Code** with the PlatformIO extension:
1. Open the project folder in VS Code.
2. Click the **PlatformIO: Build** button (✓) in the status bar.
3. Click the **PlatformIO: Upload** button (→) to flash the ESP32.

### 3. Launch the micro-ROS Agent

After uploading the firmware and ensuring the ESP32 is connected via USB:

```bash
# Source your ROS 2 workspace
source /opt/ros/jazzy/setup.bash

# Run the bringup script
python3 bringup_robot.py
```

The bringup node will:
1. **Auto-detect** the ESP32 on `/dev/ttyUSB*` or `/dev/ttyACM*`.
2. **Launch** the micro-ROS agent inside a Docker container with serial transport.
3. **Monitor** the agent process and **auto-restart** it if it crashes.
4. Handle graceful shutdown on `Ctrl+C`.

> **Tip:** You can override the device port and baud rate via ROS 2 parameters:
> ```bash
> python3 bringup_robot.py --ros-args -p device:=/dev/ttyUSB0 -p baud_rate:=115200
> ```

### Verify the Connection

Once the agent is running, you should see the topics in another terminal:

```bash
# List active topics
ros2 topic list

# Echo the steering angle feedback
ros2 topic echo /steering_angle

# Send a test throttle command
ros2 topic pub /throttle std_msgs/msg/Float32 "{data: 0.1}" --once

# Send a test steering command
ros2 topic pub /steering_command std_msgs/msg/Float32 "{data: 45.0}" --once
```

---

## Configuration & Tuning

All tunable parameters are centralized in the `config/` directory as preprocessor macros. Modify and re-upload to tune behavior.

### PID Gains (`config/servo_config.h`)

| Parameter | Default | Description |
|:----------|:-------:|:------------|
| `SERVO_KP` | 25.0 | Proportional gain |
| `SERVO_KI` | 5.0 | Integral gain |
| `SERVO_KD` | 0.1 | Derivative gain |
| `INTEGRAL_WINDUP_GUARD` | 1000.0 | Anti-windup clamp on integral term |
| `OUTPUT_LIMIT` | 2000.0 | Maximum PID output (µs offset) |
| `SERVO_CONTROL_FREQUENCY` | 100.0 | Control loop frequency (Hz) |

### Kalman Filter (`config/filter_config.h`)

| Parameter | Default | Description |
|:----------|:-------:|:------------|
| `FILTER_Q_ANGLE` | 0.1 | Process noise variance — angle |
| `FILTER_Q_VEL` | 0.1 | Process noise variance — velocity |
| `FILTER_R_MEAS` | 0.01 | Measurement noise variance |

### Motor/ESC (`lib/brushless_lib/motor_config.h`)

| Parameter | Default | Description |
|:----------|:-------:|:------------|
| `MIN_THROTTLE` | 1000 µs | Full reverse pulse width |
| `NEUTRAL` | 1500 µs | Neutral/stop pulse width |
| `MAX_THROTTLE` | 2000 µs | Full forward pulse width |

---

## Troubleshooting

| Issue | Solution |
|:------|:---------|
| **ESP32 not detected** | Check USB cable (must support data, not charge-only). Verify `/dev/ttyUSB*` exists. Run `setup_micro_ros.sh` to set permissions. |
| **micro-ROS agent won't connect** | Ensure `monitor_speed` in `platformio.ini` matches the baud rate (115200). Restart the ESP32 after uploading. |
| **Topics not appearing** | Verify the agent is running (`docker ps`). Check that the state machine reaches `AGENT_CONNECTED` via serial monitor. |
| **Noisy steering feedback** | Tune Kalman filter parameters in `config/filter_config.h`. Decrease `FILTER_R_MEAS` to trust measurements more, or increase it to smooth further. |
| **Servo oscillation** | Reduce `SERVO_KP` or increase `SERVO_KD` in `config/servo_config.h`. Check for mechanical backlash. |
| **Docker permission denied** | Add your user to the `docker` group: `sudo usermod -aG docker $USER`, then log out and back in. |

---

## Contributing

Contributions are welcome! Please:

1. Fork the repository.
2. Create a feature branch (`git checkout -b feature/my-feature`).
3. Commit your changes with clear messages.
4. Open a Pull Request describing your changes.

---

## License

This project is open-source. See the [LICENSE](LICENSE) file for details.

---

<p align="center">
  Built for speed. Built for autonomy. 🏎️
</p>

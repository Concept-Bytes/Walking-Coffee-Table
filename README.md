# 🪑 Walking Coffee Table

A robotic coffee table that *actually walks*.

Powered by two **24V DC brushless motors** and controlled via an **Arduino Nano RP2040**, this project brings a whimsical 12-legged (6 per side) walking mechanism to life. The table can be controlled remotely via Bluetooth or navigate autonomously using radar-based obstacle tracking.

---

## 🔧 Hardware Overview

- **Controller**: Arduino Nano RP2040 Connect
- **Motors**: 2x 24V brushless DC gear motors
- **Legs**: 12 total (6 per side) using a custom linkage system
- **Power**: 24V battery pack
- **Control Modes**:
  - **Bluetooth RC Mode** (default)
  - **Autonomous Radar Mode** (with Rd-03D FMCW sensor)

---

## 🕹️ Mode 1: Remote Control via Joystick

In this mode, the table is driven by a custom joystick controller using a second Arduino. It communicates wirelessly over Bluetooth.

### 📡 Communication:
- **Bluetooth UART** between two Arduino boards.
- Joystick position (X/Y) is sent from the controller to the table.

### 📁 Key Files:
- `Controller_V3.ino` — Code for the joystick controller.
- `Table_V3.ino` — Code running on the table receiving movement commands.

---

## 🤖 Mode 2: Autonomous Radar Navigation

Utilizes the **Rd-03D FMCW radar module** to detect motion and avoid obstacles. This millimeter-wave radar detects movement up to 8 meters with ±60° azimuth and ±30° pitch coverage.

### 🧠 Features:
- Tracks human presence and motion.
- Table halts or redirects when an object is detected.
- Perfect for demos or haunted house effects 🧟‍♂️

### 📁 Key Files:
- `radar_test.ino` — Basic radar parsing and visualization.
- `radar_clap.ino` — Detects claps and triggers movement.
- `clap_test.ino` — Calibrates for audio-based triggers using PDM mic.

---

## 🔬 Test & Calibration Scripts

These are standalone sketches to help calibrate or debug individual components:

| File               | Purpose                                  |
|--------------------|------------------------------------------|
| `gyro_test.ino`    | Yaw integration and drift correction     |
| `P-Control.ino`    | Simple proportional control motor test   |
| `rotation_test.ino`| Table turning/rotation testing           |

---

## 🧠 About the Rd-03D Radar Module

- **Frequency**: 24.00–24.25 GHz (ISM Band)
- **Max Range**: 8 meters
- **Detection Angle**: ±60° azimuth, ±30° pitch
- **Interface**: UART @ 256000 bps
- **Power**: 5V / ~92mA typical
- **Docs**: [Ai-Thinker Official Site](http://www.ai-thinker.com/)

This sensor allows for accurate tracking of multiple moving objects and is robust against Wi-Fi and environmental interference.

---

## 🔌 Wiring & Schematic

- **Bluetooth**: TX/RX between Arduinos (with logic level shifting if needed).
- **Radar**: UART RX/TX → RP2040 (5V input, use level shifter).
- **Motors**: Controlled via motor driver (H-bridge or similar) with PWM pins.

---

## 🛠️ Getting Started

1. Flash `Controller_V3.ino` to joystick Arduino.
2. Flash `Table_V3.ino` to Nano RP2040 on the table.
3. Power the motors and controller.
4. Move the joystick and watch the table walk.
5. To try autonomous mode, flash `radar_clap.ino` and clap 👏.

---

## 📸 Media

> Include demo videos or gifs here once uploaded!

---

## 📜 License

MIT License. Build, remix, and walk your coffee table into the future.

---

## 👷 Made by Concept Bytes

Follow along at [YouTube](https://youtube.com/@conceptbytes) and join the engineering chaos.

---

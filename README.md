<div align="center">

# ⚡ SmartServo V5.3 (56:1 Fast)

### Turn any brushed DC motor into an ultra-fast, high-precision closed-loop smart servo

**ESP32-S3 (N16R8) · AS5600 12-Bit Magnetic Encoder · BTS7960 43A H-Bridge · 400 kHz I2C · Dual Wi-Fi Dashboard**

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-ESP32--S3%20(N16R8)-orange.svg)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/)
[![Framework](https://img.shields.io/badge/Framework-Arduino%20%2F%20ESP--IDF-teal.svg)](https://www.arduino.cc/)
[![Control Loop](https://img.shields.io/badge/Loop-~1000%20Hz-brightgreen.svg)]()
[![I2C Bus](https://img.shields.io/badge/I2C-400%20kHz%20Fast%20Mode-blueviolet.svg)]()

---

*Features **Velocity Feedforward (Kff)**, **Acceleration-limited motion profiling**, **Silent 0-PWM idle lock**, and a **7-Stage 30-Second Fast Auto-PID Tuner**.*

</div>

---

## 📸 Real-Time Web Dashboard

<div align="center">

<img src="docs/dashboard-screenshot.svg" alt="SmartServo V5.3 Web Dashboard" width="100%"/>

<sub>Dark-themed, mobile-responsive web dashboard with real-time response graphing, one-tap 30s Auto-PID, and live telemetry</sub>

</div>

---

## 🔌 Visual Wiring Diagram

<div align="center">

<img src="docs/wiring-diagram.svg" alt="Visual Wiring Diagram — ESP32-S3, BTS7960, AS5600, 37GB-520 Motor, 12V Battery" width="100%"/>

</div>

> [!IMPORTANT]
> 1. **Common Ground:** Connect Battery (−), BTS7960 GND, and ESP32 GND together.
> 2. **Driver Enable:** The BTS7960's `R_EN` and `L_EN` pins must be tied to **3.3V** to enable the outputs.
> 3. **Magnet Placement:** Mount a 6mm diametric magnet on the motor shaft tip, centered 1–3 mm above the AS5600 IC.

---

## 🚀 What's New in V5.3

| Feature | V4 Pro | V5.3 (56:1 Fast) | Benefit |
|---|---|---|---|
| **Optimized Gear Ratio** | 220:1 | **56:1 (Default)** | 4x faster output rotation (supports any custom ratio) |
| **I2C Bus Speed** | 100 kHz | **400 kHz Fast Mode** | 4x faster angle sampling from AS5600 |
| **Control Loop Rate** | ~500 Hz | **~1,000 Hz (1 kHz)** | Sub-millisecond control updates on Core 1 |
| **Feedforward Control** | ❌ None | **Velocity Feedforward ($K_{ff}$)** | Eliminates tracking lag during acceleration & cruise |
| **Motion Profiler** | Velocity only | **Trapezoidal Accel-Limited** | Configurable acceleration limit (up to 900°/s²) |
| **Idle Motor State** | Potential dribble | **⚡ Silent 0-PWM Lock** | Hard zero cutoff: **zero hum, zero heat, zero power waste** |
| **Gear Protection** | Raw steps | **PWM Slew Rate Limiter** | Prevents mechanical shock and brush arcing |
| **Auto-Tuning** | Polarity only | **7-Stage 30s Fast Auto-PID** | Fully measures friction, backlash, $K_{ff}$, steps & locks |

---

## ✨ Core Features

<table>
<tr>
<td width="50%">

### 🎯 High-Performance Control
- **Velocity Feedforward ($K_{ff}$)** — Predictive torque injection for lag-free motion
- **Trapezoidal Motion Profile** — Smooth, acceleration-bounded position moves
- **Silent 0-PWM Lock** — Motor cuts power strictly to 0 when in deadzone or settled
- **Asymmetric Stiction Compensation** — Independent CW/CCW minimum PWM
- **PWM Slew Limiting** — Protects gear teeth from sudden reversals

</td>
<td width="50%">

### 🔬 7-Stage 30-Second Auto-PID
- **Stage 1:** CW & CCW breakaway friction measurement
- **Stage 2:** Gearbox backlash estimation
- **Stage 3:** Multi-speed $K_{ff}$, max acceleration & plant gain mapping
- **Stage 4:** Crisp micro-step step response tuning
- **Stage 5:** Fast $\pm60^\circ$ full-speed reversal testing
- **Stage 6:** Speed PID verification at $\pm90^\circ$/s
- **Stage 7:** Silent 0-PWM idle latch verification

</td>
</tr>
<tr>
<td>

### 🛡️ Hardware Safety & EMI Shielding
- **400 kHz Fast Mode I2C** — Minimal bus latency with clean STOP conditions
- **I2C Bus Auto-Recovery** — 9-cycle clock toggle recovers jammed bus automatically
- **Single-Step Spike Filter** — Rejects magnetic anomalies and noise glitches
- **Hardware E-STOP** — Instant emergency cutoff from dashboard or API
- **Stall Protection** — Detects physical jams and prevents motor burnout

</td>
<td>

### 🌐 Dual Wi-Fi & REST API
- **Simultaneous AP + STA** — Connect via home router or direct hotspot
- **Core 0 Web Server** — Zero interference with Core 1 control loop
- **High-Speed Telemetry** — Live JSON streaming (~8 Hz) to dashboard
- **Full NVS Persistence** — All tuning parameters and calibration saved to flash

</td>
</tr>
</table>

---

## 🧰 Hardware & Pinout

### Bill of Materials

| Component | Model | Purpose |
|---|---|---|
| **MCU** | ESP32-S3 DevKitC-1 (N16R8) | 240 MHz dual-core, 16 MB Flash, 8 MB PSRAM |
| **Motor Driver** | BTS7960 43A Dual H-Bridge | High-power motor drive (RPWM / LPWM) |
| **Position Sensor** | AS5600 Magnetic Rotary Encoder | 12-bit contactless angle sensing (0.088° res) |
| **Motor** | 37GB-520 12V DC Geared Motor | Fast 56:1 gearbox (or custom ratio) |
| **Magnet** | 6mm Diametrically Magnetized Ring/Disc | Mounted on motor output shaft |
| **Power** | 12V LiPo Battery or 12V 5A+ PSU | Main motor supply |

### Wiring Pinout

```
ESP32-S3 Pin      →   Peripheral Connection
─────────────────────────────────────────────
GPIO 14 (PWM)     →   BTS7960 RPWM (CW)
GPIO 13 (PWM)     →   BTS7960 LPWM (CCW)
GPIO 12 (I2C SDA) →   AS5600 SDA
GPIO 11 (I2C SCL) →   AS5600 SCL
3.3V OUT          →   AS5600 VCC, BTS7960 R_EN, BTS7960 L_EN
GND               →   Common Ground (ESP32, BTS7960, AS5600, Battery −)
```

---

## 🚀 Quick Start

### 1. Flash Firmware

Using [PlatformIO](https://platformio.org/):

```bash
git clone https://github.com/ssj1971/SmartServo-V4-Pro.git
cd SmartServo-V4-Pro

# Build and flash to ESP32-S3
pio run --target upload

# Open serial console
pio device monitor
```

### 2. Connect via Wi-Fi

The ESP32 boots in **dual AP + STA mode**:

| Mode | SSID | Password | IP Address |
|---|---|---|---|
| **Direct Access Point** | `SmartServo_V2` | `12345678` | `http://192.168.4.1` |
| **Station (Local Network)** | *(configurable)* | *(configurable)* | *(assigned by DHCP, check Serial)* |

*To set your local Wi-Fi credentials, edit `WIFI_SSID` and `WIFI_PASS` in `src/main.cpp`.*

### 3. One-Click Calibration

1. Open `http://192.168.4.1/` (or your local IP) in your browser.
2. Go to **Calibration** tab and tap **RUN AUTO CALIBRATION**. The motor will spin CW and CCW to verify H-bridge direction and set encoder polarity automatically.
3. Go to **PID & Auto-Tune** tab and tap **START 30s FAST AUTO-PID**. Sit back as the servo tests friction, backlash, $K_{ff}$, and step responses to compute optimal gains!

---

## 📡 REST API Reference

All commands accept standard HTTP GET requests:

| Endpoint | Purpose | Example |
|---|---|---|
| `GET /` | Serves dashboard UI | `http://192.168.4.1/` |
| `GET /cmd?mode=1&tDeg=90` | Set target position | `GET /cmd?tDeg=90` |
| `GET /cmd?mode=2&tVel=120` | Set target speed in °/s | `GET /cmd?mode=2&tVel=120` |
| `GET /cmd?mPwm=150` | Direct PWM drive | `GET /cmd?mode=3&mPwm=150` |
| `GET /cmd?autotune=1` | Trigger 7-Stage Auto-PID | `GET /cmd?autotune=1` |
| `GET /cmd?cal=1` | Trigger polarity/H-bridge cal | `GET /cmd?cal=1` |
| `GET /cmd?zero=1` | Zero current position | `GET /cmd?zero=1` |
| `GET /cmd?en=0` | Emergency Stop (E-STOP) | `GET /cmd?en=0` |
| `GET /resetpid` | Reset to default fast 56:1 gains | `GET /resetpid` |
| `GET /save?...` | Save parameters to NVS flash | `GET /save?p=7.5&ff=0.55&acc=900` |
| `GET /getparams` | Read all stored parameters (JSON) | `GET /getparams` |
| `GET /status` | Real-time telemetry (JSON) | `GET /status` |
| `GET /factory` | Full factory reset to defaults | `GET /factory` |

### Sample Telemetry JSON (`/status`)

```json
{
  "pos": 90.00,
  "tgt": 90.00,
  "tVel": 0.0,
  "vel": 0.0,
  "pwm": 0,
  "raw": 2048,
  "turns": 14.00,
  "mode": 1,
  "stall": false,
  "locked": true,
  "hz": 996,
  "ok": true,
  "rec": 0,
  "tuning": false,
  "prog": 100,
  "calMsg": "OK! Dir:+1 | CW:1830 CCW:-1815 | I2C Errs:0 (Idle 0 PWM)",
  "pidMsg": "FAST 30s TUNE OK! P:7.5 I:0.15 D:0.12 FF:0.55 | Rise:142ms | 0-PWM Lock"
}
```

---

## ⚙️ Default Tuning Parameters (56:1 Fast)

| Parameter | Default | Range | Description |
|---|---|---|---|
| **$K_p$** | `7.5` | 0.1 – 50.0 | Angle position proportional gain |
| **$K_i$** | `0.15` | 0.00 – 5.00 | Angle steady-state error elimination |
| **$K_d$** | `0.12` | 0.00 – 5.00 | Velocity damping |
| **$K_{ff}$** | `0.55` | 0.00 – 3.00 | Velocity feedforward (PWM per °/s) |
| **Max Accel** | `900` °/s² | 50 – 2000 | Trapezoidal profile acceleration ramp |
| **Max Velocity** | `360` °/s | 10 – 1000 | Speed limit |
| **PWM Slew** | `24` | 4 – 60 | Max PWM change per millisecond (gear protection) |
| **Tolerance** | `0.50`° | 0.1 – 5.0 | Position deadzone (triggers silent 0-PWM lock) |
| **Gear Ratio** | `56.0` | 0.1 – 1000.0 | Output gearbox reduction |
| **PWM Freq** | `7000` Hz | 1k – 25 kHz | Motor PWM frequency |
| **Min PWM CW/CCW** | `40` / `40` | 0 – 150 | Breakaway stiction compensation |

---

## 📁 Repository Structure

```
SmartServo-V4-Pro/
├── src/
│   └── main.cpp                  # Complete firmware (V5.3 Fast 56:1)
├── docs/
│   ├── dashboard-screenshot.svg  # Real-time web UI visual
│   └── wiring-diagram.svg        # Clean visual wiring diagram
├── platformio.ini                 # PlatformIO environment config
├── .gitignore
├── LICENSE                        # MIT License
└── README.md
```

---

<div align="center">

## 📜 License

Licensed under the [MIT License](LICENSE). Build whatever you want with it!

**[⬆ Back to Top](#-smartservo-v53-561-fast)**

</div>

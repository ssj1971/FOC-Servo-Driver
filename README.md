<div align="center">

# ⚡ FOC Servo Driver V5.3
### Industrial-Grade Closed-Loop Servo & Precision Motion Controller

**ESP32-S3 (240 MHz Dual-Core) · 12-Bit Contactless Magnetic Vector Sensing · 43A High-Current H-Bridge · 1,000 Hz Deterministic Loop · 400 kHz Fast-Mode I2C**

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-ESP32--S3%20(N16R8)-orange.svg)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/)
[![Framework](https://img.shields.io/badge/Framework-Arduino%20%2F%20ESP--IDF-teal.svg)](https://www.arduino.cc/)
[![Deterministic Loop](https://img.shields.io/badge/Core%201%20Loop-1%2C000%20Hz%20(1ms)-brightgreen.svg)]()
[![Feedback](https://img.shields.io/badge/Feedback-12--Bit%20Magnetic%20(0.088%C2%B0)-blueviolet.svg)]()
[![I2C Bus](https://img.shields.io/badge/I2C-400%20kHz%20Fast%20Mode-success.svg)]()

---

*Engineered for robotics, CNC, antenna rotators, pan-tilt gimbals, and automated machinery requiring **sub-degree position repeatability**, **active speed stabilization**, **velocity feedforward ($K_{ff}$)**, and **30-second autonomous system identification**.*

</div>

---

## 📸 Real-Time Oscilloscope Web Dashboard

<div align="center">

<img src="docs/dashboard-screenshot.svg" alt="FOC Servo Driver V5.3 Web Dashboard" width="100%"/>

<sub>Dark-themed, industrial control interface featuring dual-trace target vs. actual response plotting, 1-tap 30-second Auto-PID tuning, and sub-millisecond loop telemetry.</sub>

</div>

---

## 🔌 Industrial Wiring & Power Topology

<div align="center">

<img src="docs/wiring-diagram.svg" alt="FOC Servo Driver V5.3 — Industrial Wiring & Power Topology" width="100%"/>

</div>

> [!IMPORTANT]
> 1. **Common Star Ground:** Bond Battery (−), BTS7960 Power GND, and ESP32 Logic GND at a single star-point to prevent ground loops and inductive switching bounce.
> 2. **Driver Stage Enable:** Tie BTS7960 `R_EN` and `L_EN` to **3.3V** to activate output FET gates.
> 3. **Magnetic Alignment:** Mount a 6mm diametric magnet on the motor shaft tip, aligned 1.0–2.5 mm directly above the AS5600 hall sensor array.

---

## 🏛️ Cascaded Multi-Loop Control Architecture

The **FOC Servo Driver** implements an industry-standard cascaded control topology running deterministically on a dedicated FreeRTOS core:

```
[ Target Position / Velocity Command ]
                │
                ▼
┌───────────────────────────────────────────────┐
│       Trapezoidal Trajectory Generator        │
│  • Accel Ramp: a_max (up to 2000 °/s²)        │
│  • Velocity Bound: v_max (up to 1000 °/s)     │
└───────┬───────────────────────────────┬───────┘
        │ Profiled Position             │ Profiled Velocity
        ▼                               ▼
┌───────────────────────────────┐   ┌───────────────────────────────┐
│     Outer Position Loop       │   │   Velocity Feedforward (Kff)  │
│  • Proportional Gain: Kp      │   │  • Direct Torque Feedforward  │
│  • Anti-Windup Integral: Ki   │   │  • Phase-Lag Elimination      │
│  • Damping Derivative: Kd     │   └───────────────┬───────────────┘
└───────────────┬───────────────┘                   │
                │ Position Error PWM                │ Velocity Feedforward Effort
                └───────────────┬───────────────────┘
                                ▼
┌───────────────────────────────────────────────────────────────┐
│            Intermediate Velocity / Speed Loop                 │
│  • Closed-Loop Velocity PI Regulator: velKp / velKi           │
│  • Low-Lag Exponential Moving Average Velocity Filter (alpha) │
└───────────────────────────────┬───────────────────────────────┘
                                ▼
┌───────────────────────────────────────────────────────────────┐
│           Inner Effort & PWM Commutation Stage                │
│  • Asymmetric Stiction Breakaway Compensation (CW & CCW)     │
│  • PWM Slew-Rate Limiter (Gear & FET Protection)              │
│  • Silent 0-PWM Anti-Hunt Latched Holding State               │
└───────────────────────────────┬───────────────────────────────┘
                                │ Motor Drive Signals (RPWM / LPWM)
                                ▼
┌───────────────────────────────────────────────────────────────┐
│        BTS7960 43A High-Current Half-Bridge Driver            │
└───────────────────────────────┬───────────────────────────────┘
                                │ High-Current Armature Drive
                                ▼
┌───────────────────────────────────────────────────────────────┐
│        DC Actuator + High-Torque Gearbox (56:1 Fast)          │
└───────────────────────────────┬───────────────────────────────┘
                                │ Mechanical Shaft Output
                                ▼
┌───────────────────────────────────────────────────────────────┐
│    AS5600 12-Bit Contactless Absolute Magnetic Encoder        │
│  • 4096 Counts/Revolution (0.0879° Angular Quantization)      │
│  • 400 kHz Fast-Mode I2C Bus (0x0E Angle Caching, <65 µs)     │
│  • Real-Time Single-Step Anomaly & Noise Spike Filter         │
└───────────────────────────────────────────────────────────────┘
```

---

## 🌟 Industry-Grade Motion Capabilities

### 🎯 1. Sub-Degree Precision Position Control
- **12-Bit Contactless Angular Quantization:** 4,096 absolute counts per motor shaft turn ($0.0879^\circ$ raw resolution; scaled through the 56:1 gearbox to sub-millidegree output precision).
- **Trapezoidal Acceleration-Bounded Profiler:** Real-time calculation of smooth acceleration ramps ($a_{max}$) and velocity limits ($v_{max}$) prevents position overshoot and mechanical jerk.
- **Anti-Windup Position Integrator:** Dynamically clamps the position accumulator within stable thresholds and decays when tracking error widens, preventing actuator saturation.
- **⚡ Silent 0-PWM Anti-Hunt Latched Holding:** When the output settles within the deadzone tolerance ($\pm0.5^\circ$), the driver **cuts motor drive strictly to 0 PWM**. This eliminates idle motor heating, acoustic coil whine, and gear wear while holding load securely.

### 💨 2. High-Dynamic Precision Speed Control
- **Predictive Velocity Feedforward ($K_{ff}$):** Dynamically calculates expected armature drive effort proportional to commanded rotational speed, eliminating following error and phase lag during high-acceleration moves.
- **Closed-Loop Velocity Regulator:** Dedicated secondary PI velocity loop ensures constant rotational speed regardless of external load fluctuations.
- **Low-Lag Digital Observer (EMA Filter):** Filters encoder quantization jitter through an Exponential Moving Average filter ($\alpha$-tuned) to provide clean velocity derivatives without phase delay.
- **Bi-Directional Dynamic Range:** Precise control across extreme speed envelopes—from creeping speeds ($0.5^\circ/\text{s}$) up to rapid repositioning ($600^\circ/\text{s}+$).

### 🛡️ 3. Torque Slew & Hardware Protection
- **PWM Slew Rate Limiter ($S_{pwm}$):** Governs output change rate ($\Delta \text{PWM}/\text{ms}$) to absorb back-EMF spikes, protect gearbox teeth from shear stress, and extend commutator brush life.
- **Asymmetric Stiction Compensation:** Independent CW and CCW minimum torque injection ensures smooth breakaway without deadzone stick-slip motion.
- **Stall & Overcurrent Watchdog:** Automatically cuts motor power if PWM effort saturates while zero velocity is detected, safeguarding the actuator against physical jams and thermal destruction.

---

## 🔬 7-Stage Autonomous 30-Second System Identification

Tap **START 30s FAST AUTO-PID** in the web dashboard or trigger `/cmd?autotune=1` via REST API. The driver executes a complete system identification sequence to identify mechanical and electrical parameters:

```
[ Stage 1 ] Breakaway Friction  ──► Characterizes static CW & CCW stiction thresholds
[ Stage 2 ] Backlash Estimation ──► Measures 56:1 gear train mechanical play
[ Stage 3 ] Plant Gain & Kff    ──► Maps speed curve, motor gain (Kv), and max accel
[ Stage 4 ] Micro-Step Tuning   ──► Tunes Kp and Kd damping on small 10° steps
[ Stage 5 ] Dynamic Reversals   ──► Stress-tests ±60° high-speed inertia transitions
[ Stage 6 ] Velocity PID        ──► Calibrates speed-mode PI loop at ±90°/s
[ Stage 7 ] 0-PWM Idle Lock     ──► Validates zero-power quiet latching & saves to NVS
```

| Stage | Duration | Measurement & Optimization Goal |
|---|---|---|
| **1. Friction Characterization** | ~3.5s | Ramps PWM in both directions until motion is detected. Stores optimal `minPWM_CW` and `minPWM_CCW`. |
| **2. Backlash Estimation** | ~2.5s | Applies gentle forward torque, reverses direction, and records deadband angle before shaft rotates. Automatically sets position deadzone `tolerance`. |
| **3. Plant Gain & $K_{ff}$** | ~4.5s | Drives motor at two speeds, measures achieved velocities, and calculates velocity feedforward ($K_{ff}$) and plant gain ($K_v$). |
| **4. Micro-Step Damping** | ~5.5s | Executes $\pm10^\circ$ micro-steps, checking rise time, overshoot, and PWM reversals to set initial $K_p$, $K_i$, and $K_d$. |
| **5. High-Speed Reversals** | ~6.5s | Commands full-acceleration $\pm60^\circ$ direction reversals. Adjusts slew rate and damping to eliminate overshoot under high rotational inertia. |
| **6. Speed Loop Verification** | ~3.5s | Operates in pure velocity closed-loop at $\pm90^\circ/\text{s}$ to fine-tune `velKp` and `velKi`. |
| **7. 0-PWM Latch Verification** | ~2.5s | Settles actuator at target angle and verifies that drive current drops completely to 0. Saves all calibrated parameters to NVS flash memory. |

---

## 🎛️ Multi-Mode Industrial Operation

| Mode | Identifier | Function | Primary Use Case |
|---|---|---|---|
| **Position Mode** | `MODE_ANGLE (1)` | Closed-loop absolute angle servo with motion profiling and 0-PWM lock | Robotic joints, CNC axis positioning, camera pan-tilt, antenna tracking |
| **Speed Mode** | `MODE_SPEED (2)` | Closed-loop velocity regulator with feedforward torque compensation | Conveyor belts, pumps, constant-velocity scanning, feed rollers |
| **Effort / Torque Mode** | `MODE_MANUAL (3)` | Direct PWM drive with active slew rate limiting | Open-loop testing, end-stop calibration, external torque control |
| **Standby / Idle Mode** | `MODE_IDLE (0)` | Complete motor bridge disable (0 PWM) | Power saving, emergency halt, manual back-driving |

---

## 🧰 Hardware Specifications & Pinout

### System Bill of Materials

| Component | Specification | Function |
|---|---|---|
| **Controller** | ESP32-S3 DevKitC-1 (N16R8) | 240 MHz dual-core Xtensa LX7, 16 MB Flash, 8 MB PSRAM |
| **Power Stage** | BTS7960 Dual Half-Bridge Module | 43A peak current, 6–28V DC operating range |
| **Rotary Sensor** | AS5600 Magnetic Angle Sensor | 12-bit contactless hall array (4096 positions/rev) |
| **Actuator** | 37GB-520 High-Torque DC Motor | 12V DC, 56:1 metal planetary/spur gearbox |
| **Magnetic Target** | 6mm $\times$ 2.5mm Diametric Magnet | Mounted axially on motor tail shaft |
| **Power Supply** | 12V 5A–10A PSU or 3S/4S LiPo | Main power bus for BTS7960 high-current stage |

### Controller Pinout Mapping

```
ESP32-S3 Pin       →   Peripheral Connection
────────────────────────────────────────────────────────────────
GPIO 14 (LEDC PWM) →   BTS7960 RPWM (Forward / CW Effort)
GPIO 13 (LEDC PWM) →   BTS7960 LPWM (Reverse / CCW Effort)
GPIO 12 (I2C SDA)  →   AS5600 SDA (Data line, internal 45k pull-up)
GPIO 11 (I2C SCL)  →   AS5600 SCL (Clock line, internal 45k pull-up)
3.3V OUT           →   AS5600 VCC, BTS7960 R_EN, BTS7960 L_EN
GND                →   Common Star Ground (ESP32, BTS7960, AS5600, PSU −)
```

---

## 🚀 Quick Start Guide

### 1. Build and Flash Firmware

Clone the repository and flash using [PlatformIO](https://platformio.org/):

```bash
git clone https://github.com/ssj1971/FOC-Servo-Driver.git
cd FOC-Servo-Driver

# Build and flash to ESP32-S3
pio run --target upload

# Open high-speed serial monitor
pio device monitor
```

### 2. Connect to Web Telemetry

The driver automatically initializes dual **AP + Station** networking:

| Network Interface | SSID | Password | IP Address |
|---|---|---|---|
| **Direct Access Point** | `FOC_Servo_Driver` | `12345678` | `http://192.168.4.1` |
| **Station (Local Network)** | *(configured in code)* | *(configured in code)* | *(assigned by DHCP, check Serial)* |

*To connect to your existing Wi-Fi network, modify `WIFI_SSID` and `WIFI_PASS` in `src/main.cpp`.*

### 3. One-Touch Auto-Commissioning

1. Navigate to `http://192.168.4.1/` (or your local IP address).
2. Open the **Calibration** tab and click **RUN AUTO CALIBRATION**. The actuator will spin CW and CCW to verify bridge continuity, detect phase polarity, and check for I2C EMI errors.
3. Open the **PID & Auto-Tune** tab and click **START 30s FAST AUTO-PID**. The controller will perform automated system identification and lock in optimal gains.

---

## 📡 REST API & Machine Telemetry

All parameters and motion commands are accessible over high-speed HTTP GET requests for integration with ROS, Python, LabVIEW, PLC, or custom controllers:

### Command Endpoints (`/cmd`)

| Parameter | Type | Description | Example Request |
|---|---|---|---|
| `mode` | `int` | Operating mode (`0`: Idle, `1`: Position, `2`: Speed, `3`: Manual) | `GET /cmd?mode=1` |
| `tDeg` | `float` | Target angular position in degrees | `GET /cmd?tDeg=180.0` |
| `tVel` | `float` | Target velocity in degrees per second | `GET /cmd?mode=2&tVel=90.0` |
| `mPwm` | `int` | Direct effort PWM (`-255` to `+255`) | `GET /cmd?mode=3&mPwm=120` |
| `vel` | `float` | Dynamic speed limit in position mode (°/s) | `GET /cmd?vel=360` |
| `maxpwm` | `int` | Output torque clamp (`60` to `255`) | `GET /cmd?maxpwm=200` |
| `zero` | `int` | Zeros current position as home offset | `GET /cmd?zero=1` |
| `en` | `int` | Emergency Stop (`0`: E-STOP, `1`: Enable) | `GET /cmd?en=0` |
| `autotune` | `int` | Triggers 7-stage autonomous PID tuning | `GET /cmd?autotune=1` |
| `cal` | `int` | Triggers polarity and H-bridge continuity test | `GET /cmd?cal=1` |
| `recI2C` | `int` | Forces hardware 9-cycle I2C bus recovery | `GET /cmd?recI2C=1` |

### System Telemetry Endpoint (`/status`)

Returns instantaneous controller states at up to 20 Hz:

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
  "hz": 998,
  "ok": true,
  "rec": 0,
  "tuning": false,
  "prog": 100,
  "calMsg": "OK! Dir:+1 | CW:1830 CCW:-1815 | I2C Errs:0 (Idle 0 PWM)",
  "pidMsg": "FAST 30s TUNE OK! P:7.5 I:0.15 D:0.12 FF:0.55 | Rise:142ms | 0-PWM Lock"
}
```

---

## ⚙️ Default Tuning Parameters (56:1 Industrial Actuator)

| Parameter | Symbol | Default | Range | Description |
|---|---|---|---|---|
| **Position Proportional** | $K_p$ | `7.5` | 0.1 – 50.0 | Angular stiffness and error response |
| **Position Integral** | $K_i$ | `0.15` | 0.00 – 5.00 | Steady-state load offset elimination |
| **Position Derivative** | $K_d$ | `0.12` | 0.00 – 5.00 | Velocity damping and overshoot suppression |
| **Velocity Feedforward** | $K_{ff}$ | `0.55` | 0.00 – 3.00 | Predictive torque injection per °/s of velocity |
| **Velocity Proportional** | $vK_p$ | `1.4` | 0.1 – 20.0 | Speed-mode tracking stiffness |
| **Velocity Integral** | $vK_i$ | `2.5` | 0.1 – 30.0 | Speed-mode steady-state torque regulator |
| **Acceleration Limit** | $a_{max}$ | `900` °/s² | 50 – 2000 | Trapezoidal profile acceleration ramp |
| **Velocity Limit** | $v_{max}$ | `360` °/s | 10 – 1000 | Maximum rotational speed in position mode |
| **PWM Slew Rate** | $S_{pwm}$ | `24` | 4 – 60 | Maximum PWM change per ms (gearbox protection) |
| **Deadzone Tolerance** | $\epsilon_{pos}$ | `0.50`° | 0.1 – 5.0 | Position window triggering silent 0-PWM lock |
| **Gearbox Reduction** | $R$ | `56.0` | 0.1 – 1000.0 | Actuator reduction ratio (customizable) |
| **PWM Frequency** | $f_{pwm}$ | `7000` Hz | 1k – 25 kHz | Motor drive switching frequency |
| **Stiction CW / CCW** | $P_{min}$ | `40` / `40` | 0 – 150 | Minimum breakaway PWM threshold |

---

## 📁 Repository Structure

```
FOC-Servo-Driver/
├── src/
│   └── main.cpp                  # Complete firmware (1kHz Control Loop & Web Dashboard)
├── docs/
│   ├── dashboard-screenshot.svg  # Real-time web UI visual
│   └── wiring-diagram.svg        # Industrial wiring and power topology
├── platformio.ini                 # PlatformIO build configuration
├── .gitignore
├── LICENSE                        # MIT License
└── README.md
```

---

<div align="center">

## 📜 License

Licensed under the [MIT License](LICENSE). Free for commercial, industrial, and hobby applications.

**[⬆ Back to Top](#-foc-servo-driver-v53)**

</div>

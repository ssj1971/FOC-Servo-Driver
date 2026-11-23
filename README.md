# SmartServo V4 Pro (I2C)

**Turn any brushed DC motor + AS5600 magnetic encoder into a closed-loop smart servo with Wi-Fi control, live telemetry, and a full web dashboard — all on a single ESP32-S3.**

---

## Features

| Category | Details |
|---|---|
| **MCU** | ESP32-S3 N16R8 (dual-core, 16 MB flash, 8 MB PSRAM) |
| **Encoder** | AS5600 12-bit magnetic encoder over I2C @ 100 kHz |
| **Motor Driver** | BTS7960 43A H-Bridge (RPWM / LPWM) |
| **Control Modes** | Angle Lock (PID) · Speed PID · Manual PWM · Idle |
| **Motion Profile** | Trapezoidal velocity-limited ramp for smooth moves |
| **Networking** | Simultaneous AP + STA Wi-Fi; built-in async HTTP server |
| **Dashboard** | Dark-themed, mobile-responsive, multi-tab web UI |
| **Persistence** | All tunable parameters saved to NVS (ESP32 flash) |
| **Safety** | E-Stop, stall detection, I2C auto-recovery, spike rejection |
| **Control Loop** | ~500 Hz RTOS task pinned to Core 1 |

---

## Hardware

### Bill of Materials

| Component | Description |
|---|---|
| ESP32-S3-DevKitC-1 (N16R8) | Main controller |
| AS5600 | 12-bit contactless magnetic position sensor |
| BTS7960 | 43A dual H-bridge motor driver module |
| Brushed DC Motor | With 220:1 (or any) gearbox |
| 6mm Diametrically-Magnetised Magnet | Mounted on motor shaft for AS5600 |
| 12V Power Supply | Sized for your motor's stall current |

### Wiring

```
ESP32-S3              BTS7960
────────              ───────
GPIO 14  ──────────►  RPWM
GPIO 13  ──────────►  LPWM
GND      ──────────►  GND
                      VCC ◄── 12V Supply
                      R_EN ── 3.3V (or GPIO for enable)
                      L_EN ── 3.3V (or GPIO for enable)

ESP32-S3              AS5600
────────              ──────
GPIO 12 (SDA) ─────►  SDA
GPIO 11 (SCL) ─────►  SCL
3.3V          ─────►  VCC
GND           ─────►  GND
```

> [!IMPORTANT]
> The AS5600's diametric magnet must be centered above the IC with ~1-3 mm air gap. Misalignment causes noisy or missing readings.

---

## Software Setup

### Prerequisites

- [PlatformIO](https://platformio.org/) (CLI or VS Code extension)
- USB-C cable for the ESP32-S3 DevKit

### Build & Upload

```bash
# Clone the repo
git clone https://github.com/<your-username>/SmartServo-V4-Pro.git
cd SmartServo-V4-Pro

# Build
pio run

# Upload (hold BOOT on first flash if needed)
pio run --target upload

# Monitor serial output
pio device monitor
```

### Wi-Fi Configuration

Edit the credentials at the top of `src/main.cpp`:

```cpp
const char* WIFI_SSID = "iot";
const char* WIFI_PASS = "111222333";
```

The ESP32 runs in **AP + STA** mode simultaneously:

| Network | SSID | Password | Default IP |
|---|---|---|---|
| **Access Point** | `SmartServo_V2` | `12345678` | `192.168.4.1` |
| **Station** | *(your Wi-Fi)* | *(your password)* | *(DHCP assigned — see serial)* |

---

## Web Dashboard

Open `http://<ESP32-IP>/` in any browser. The dashboard has four tabs:

### Control Tab
- **Mode selector**: Idle / Angle Lock / Speed PID / Manual PWM
- **Angle mode**: Slider + presets (-180°, -90°, 0°, +90°, +180°, +360°)
- **Speed mode**: Target velocity in °/s with presets
- **Manual mode**: Direct PWM drive with CW/CCW test buttons
- **Speed limit** and **max torque** sliders

### PID & Motion Tab
- Angle PID gains: Kp, Ki, Kd
- Speed PID gains: Kp, Ki
- Velocity filter alpha (low-pass smoothing)
- Deadzone tolerance
- **Apply & Save to Flash** persists all values to NVS

### Calibration Tab
- **Auto-Calibration**: Spins the motor CW then CCW to verify both H-bridge directions and automatically set encoder polarity
- Live AS5600 raw value + I2C health status
- Encoder polarity toggle
- Spike rejection threshold
- Force I2C bus recovery

### Hardware Tab
- Gear ratio (supports any gearbox)
- Min PWM for CW / CCW (asymmetric stiction compensation)
- PWM frequency (1–25 kHz)
- Stall protection enable + timeout
- **Factory Reset** button

---

## Control Architecture

```
                        ┌──────────────────────────────────────────────┐
   Browser (Wi-Fi)      │              ESP32-S3                        │
  ┌──────────────┐      │                                              │
  │  Web Dashboard│◄────►│ Core 0: AsyncWebServer + Wi-Fi              │
  │  (HTML/JS)   │ HTTP │                                              │
  └──────────────┘      │ Core 1: RTOS Control Task (~500 Hz)          │
                        │  ┌─────────┐    ┌─────────┐    ┌─────────┐  │
                        │  │ Encoder  │───►│  PID    │───►│ Motor   │  │
                        │  │ AS5600   │    │ Control │    │ BTS7960 │  │
                        │  │ (I2C)    │    │         │    │ (PWM)   │  │
                        │  └─────────┘    └─────────┘    └─────────┘  │
                        │        ▲              │                      │
                        │        │    NVS Flash Storage                │
                        │        └──── (Preferences)                   │
                        └──────────────────────────────────────────────┘
```

- **Core 0** handles Wi-Fi, HTTP requests, and the web dashboard
- **Core 1** runs the real-time control loop at ~500 Hz with I2C encoder reads, PID computation, and PWM output
- Communication between cores uses `volatile` shared variables (safe for single-writer patterns)

---

## API Endpoints

All endpoints accept HTTP GET requests.

| Endpoint | Description |
|---|---|
| `GET /` | Serves the web dashboard |
| `GET /cmd?...` | Real-time commands (mode, target, enable, zero, etc.) |
| `GET /save?...` | Apply + persist tuning parameters to NVS |
| `GET /getparams` | Returns current parameters as JSON |
| `GET /status` | Returns live telemetry as JSON (~8 Hz poll) |
| `GET /factory` | Factory reset all NVS settings |

### Example: Set target angle to 90°
```
GET /cmd?tDeg=90
```

### Example: Switch to speed mode at 45°/s
```
GET /cmd?mode=2&tVel=45
```

### Status JSON response
```json
{
  "pos": 89.72,
  "tgt": 90.0,
  "tVel": 0.0,
  "vel": 0.3,
  "pwm": 12,
  "raw": 2048,
  "turns": 54.8,
  "mode": 1,
  "stall": false,
  "hz": 498,
  "ok": true,
  "rec": 0,
  "calMsg": "OK! Dir:+1 (CW:1832 ticks, CCW:-1791 ticks)"
}
```

---

## Default Tuning Parameters

| Parameter | Default | Description |
|---|---|---|
| Kp | 5.0 | Angle PID proportional |
| Ki | 0.08 | Angle PID integral |
| Kd | 0.08 | Angle PID derivative |
| velKp | 1.2 | Speed PID proportional |
| velKi | 2.0 | Speed PID integral |
| Tolerance | 0.5° | Deadzone for angle lock |
| Max Velocity | 180°/s | Motion profile speed limit |
| Vel Alpha | 0.20 | Velocity low-pass filter (0.05–1.0) |
| PWM Freq | 7000 Hz | Motor PWM frequency |
| Max PWM | 255 | Torque limit |
| Min PWM CW | 45 | Minimum PWM to overcome CW stiction |
| Min PWM CCW | 45 | Minimum PWM to overcome CCW stiction |
| Gear Ratio | 220:1 | Gearbox reduction ratio |
| Spike Limit | 1600 | Reject encoder jumps > N ticks |
| Stall Enable | false | Stall detection off by default |
| Stall Timeout | 2000 ms | Time before stall fault triggers |

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| Dashboard won't load | Not connected to AP or STA | Connect to `SmartServo_V2` Wi-Fi, navigate to `192.168.4.1` |
| Motor oscillates wildly | PID gains too high | Reduce Kp, increase Kd |
| Motor won't move at low angles | Stiction | Increase Min PWM CW/CCW |
| I2C FAULT badge | Wiring / magnet issue | Check SDA/SCL wiring; check magnet alignment; press "Force I2C Reset" |
| Auto-cal says "FAULT: No movement" | Motor not powered or driver wiring wrong | Check 12V supply, BTS7960 enable pins, motor connections |
| Position drifts over time | Encoder polarity wrong | Run Auto-Calibration, or manually toggle encoder polarity |

---

## Project Structure

```
SmartServo-V4-Pro/
├── src/
│   └── main.cpp          # All firmware code (single-file for easy Arduino IDE compat)
├── platformio.ini         # PlatformIO project configuration
├── .gitignore
├── LICENSE
└── README.md
```

---

## License

This project is released under the [MIT License](LICENSE).

---

## Contributing

1. Fork this repository
2. Create a feature branch (`git checkout -b feature/my-improvement`)
3. Commit your changes
4. Push and open a Pull Request

---

*Built with ❤️ for the ESP32 robotics & maker community.*

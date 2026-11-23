#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Preferences.h>

// ================= WI-FI CREDENTIALS =================
const char* WIFI_SSID = "iot";
const char* WIFI_PASS = "111222333";

// ================= PINS (ESP32-S3 N16R8 SAFE) =================
const int motorRPWM = 14; // BTS7960 RPWM
const int motorLPWM = 13; // BTS7960 LPWM
const int I2C_SDA   = 12; // AS5600 SDA
const int I2C_SCL   = 11; // AS5600 SCL

// ================= CONTROL MODES =================
enum CtrlMode : uint8_t {
  MODE_IDLE   = 0,
  MODE_ANGLE  = 1,
  MODE_SPEED  = 2,
  MODE_MANUAL = 3
};

// ================= TUNABLE PARAMETERS (SAVED TO NVS) =================
float Kp          = 5.0f;    // Angle Lock P
float Ki          = 0.08f;   // Angle Lock I
float Kd          = 0.08f;   // Angle Lock D
float velKp       = 1.2f;    // Speed Mode P
float velKi       = 2.0f;    // Speed Mode I
float tolerance   = 0.5f;    // Angle lock deadzone (deg)
float maxDeg      = 360.0f;  // Slider bounds (±deg)
float gearRatio   = 220.0f;  // 220:1 gearbox
float maxVelocity = 180.0f;  // Motion profile speed limit (deg/s)
float velAlpha    = 0.20f;   // Velocity low-pass smoothing (0.05 - 1.0)

int   pwmFreq     = 7000;    // 7 kHz default PWM
int   maxPWM      = 255;     // Torque limit (60 - 255)
int   minPWM_CW   = 45;      // Min PWM for CW stiction
int   minPWM_CCW  = 45;      // Min PWM for CCW stiction
int   encDir      = 1;       // +1 or -1
int   spikeLimit  = 1600;    // Reject impossible single-step tick jumps
bool  stallEnable = false;   // OFF by default so hand-hold tests don't trip
int   stallMs     = 2000;    // Stall timeout (ms)

// ================= LIVE STATE & TELEMETRY =================
volatile uint8_t  g_mode        = MODE_ANGLE;
volatile bool     servoEnabled  = true;
volatile bool     isStalled     = false;
volatile bool     isCalibrating = false;
volatile bool     reqAutoCal    = false;
volatile bool     reqI2CRecover = false;

volatile float    targetDeg     = 0.0f;
volatile float    targetVel     = 0.0f;
volatile int      manualPWM     = 0;

volatile float    currentDeg    = 0.0f;
volatile float    currentVel    = 0.0f;
volatile int      currentPWM   = 0;
volatile int      liveRaw       = 0;
volatile int      loopHz        = 0;
volatile bool     encHealthy    = true;
volatile uint32_t i2cRecoveries = 0;

char calStatusMsg[96] = "Ready (Run Auto-Cal to verify CW/CCW & Polarity)";

// ================= INTERNAL CONTROL VARS =================
bool  pwmInitialized = false;
float integral = 0.0f, lastError = 0.0f;
float velIntegral = 0.0f;
float currentProfiledTarget = 0.0f;
float prevDeg = 0.0f;

int     lastRaw      = 0;
long    totalTicks   = 0;
long    homeOffset   = 0;
uint8_t i2cFailCount = 0;

unsigned long lastLoopTime   = 0;
unsigned long stallStartTime = 0;

TaskHandle_t controlTaskHandle;
Preferences prefs;
AsyncWebServer server(80);

// ================= PWM SETUP & OUTPUT =================
void setupPWM(int freq) {
  freq = constrain(freq, 1000, 25000);
  pwmFreq = freq;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (pwmInitialized) {
    ledcDetach(motorRPWM);
    ledcDetach(motorLPWM);
  }
  ledcAttach(motorRPWM, pwmFreq, 8);
  ledcAttach(motorLPWM, pwmFreq, 8);
  ledcWrite(motorRPWM, 0);
  ledcWrite(motorLPWM, 0);
#else
  ledcSetup(0, pwmFreq, 8);
  ledcSetup(1, pwmFreq, 8);
  ledcAttachPin(motorRPWM, 0);
  ledcAttachPin(motorLPWM, 1);
  ledcWrite(0, 0);
  ledcWrite(1, 0);
#endif
  pwmInitialized = true;
}

inline void writeMotorRaw(int pwm) {
  pwm = constrain(pwm, -maxPWM, maxPWM);
  currentPWM = pwm;
  uint8_t rVal = (pwm > 0) ? (uint8_t)pwm    : 0;
  uint8_t lVal = (pwm < 0) ? (uint8_t)(-pwm) : 0;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(motorRPWM, rVal);
  ledcWrite(motorLPWM, lVal);
#else
  ledcWrite(0, rVal);
  ledcWrite(1, lVal);
#endif
}

// ================= AS5600 I2C (100 kHz, 0x0E ONLY, CLEAN STOP) =================
void recoverI2C() {
  writeMotorRaw(0);
  Wire.end();
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, OUTPUT);

  for (int i = 0; i < 9; i++) {
    digitalWrite(I2C_SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(I2C_SCL, HIGH);
    delayMicroseconds(10);
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000); // 100 kHz for strong EMI immunity
  Wire.setTimeOut(5);
  i2cRecoveries++;
}

inline bool readAS5600Raw(int &outRaw) {
  Wire.beginTransmission(0x36);
  Wire.write(0x0E); // Angle register ONLY
  if (Wire.endTransmission(true) != 0) return false; // Clean STOP condition
  if (Wire.requestFrom((uint8_t)0x36, (uint8_t)2, (uint8_t)true) != 2) return false;
  outRaw = ((Wire.read() & 0x0F) << 8) | Wire.read();
  return true;
}

inline void updateEncoder(float dt) {
  int raw = 0;
  if (!readAS5600Raw(raw)) {
    if (++i2cFailCount >= 15) {
      encHealthy = false;
      recoverI2C();
      i2cFailCount = 0;
    }
    return;
  }

  i2cFailCount = 0;
  encHealthy = true;
  liveRaw = raw;

  int diff = raw - lastRaw;
  if (diff > 2048)  diff -= 4096;
  if (diff < -2048) diff += 4096;

  // Reject impossible single-step magnetic/I2C glitch
  if (abs(diff) < spikeLimit) {
    totalTicks += diff;
    lastRaw = raw;
  }

  float ratio = (gearRatio > 0.01f) ? gearRatio : 1.0f;
  float newDeg = (((float)(totalTicks - homeOffset) * (float)encDir) * (360.0f / 4096.0f)) / ratio;

  if (dt > 0.0001f) {
    float instVel = (newDeg - prevDeg) / dt;
    currentVel = (1.0f - velAlpha) * currentVel + velAlpha * instVel;
  }
  prevDeg = newDeg;
  currentDeg = newDeg;
}

void resetControllers() {
  integral = 0.0f;
  lastError = 0.0f;
  velIntegral = 0.0f;
  isStalled = false;
  stallStartTime = 0;
  currentProfiledTarget = currentDeg;
}

// ================= ASYMMETRIC-SAFE MOTOR DRIVE =================
void drivePIDOutput(float output, bool checkDeadzone) {
  if (!servoEnabled || isStalled || !encHealthy) {
    writeMotorRaw(0);
    return;
  }

  if (checkDeadzone && fabsf(targetDeg - currentDeg) < tolerance) {
    writeMotorRaw(0);
    return;
  }

  int minP = (output >= 0.0f) ? minPWM_CW : minPWM_CCW;
  int mag  = (int)roundf(fabsf(output));
  if (mag < minP) mag = minP;
  if (mag > maxPWM) mag = maxPWM;

  writeMotorRaw((output >= 0.0f) ? mag : -mag);
}

// ================= AUTO CALIBRATION (CW/CCW H-BRIDGE + POLARITY) =================
void runAutoCalibration() {
  isCalibrating = true;
  snprintf(calStatusMsg, sizeof(calStatusMsg), "Calibrating: Spinning CW...");
  writeMotorRaw(0);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  int testPWM = constrain(max(130, max(minPWM_CW, minPWM_CCW) + 60), 100, maxPWM);
  int prevR = lastRaw;
  readAS5600Raw(prevR);

  // 1. Spin Positive (RPWM) for 900 ms
  long cwTicks = 0;
  writeMotorRaw(testPWM);
  unsigned long t0 = millis();
  while (millis() - t0 < 900) {
    int r = prevR;
    if (readAS5600Raw(r)) {
      liveRaw = r;
      int d = r - prevR;
      if (d > 2048) d -= 4096;
      if (d < -2048) d += 4096;
      if (abs(d) < spikeLimit) { cwTicks += d; prevR = r; }
    }
    vTaskDelay(2);
  }
  writeMotorRaw(0);
  vTaskDelay(250 / portTICK_PERIOD_MS);

  // 2. Spin Negative (LPWM) for 900 ms
  snprintf(calStatusMsg, sizeof(calStatusMsg), "Calibrating: Spinning CCW...");
  long ccwTicks = 0;
  readAS5600Raw(prevR);
  writeMotorRaw(-testPWM);
  t0 = millis();
  while (millis() - t0 < 900) {
    int r = prevR;
    if (readAS5600Raw(r)) {
      liveRaw = r;
      int d = r - prevR;
      if (d > 2048) d -= 4096;
      if (d < -2048) d += 4096;
      if (abs(d) < spikeLimit) { ccwTicks += d; prevR = r; }
    }
    vTaskDelay(2);
  }
  writeMotorRaw(0);
  vTaskDelay(150 / portTICK_PERIOD_MS);

  // 3. Evaluate Results
  bool cwMoved  = (abs(cwTicks)  > 400);
  bool ccwMoved = (abs(ccwTicks) > 400);

  if (cwMoved && ccwMoved) {
    encDir = (cwTicks > 0) ? 1 : -1;
    prefs.putInt("dir", encDir);
    snprintf(calStatusMsg, sizeof(calStatusMsg),
             "OK! Dir:%+d (CW:%ld ticks, CCW:%ld ticks)",
             encDir, cwTicks, ccwTicks);
  } else if (cwMoved && !ccwMoved) {
    snprintf(calStatusMsg, sizeof(calStatusMsg),
             "FAULT: LPWM (GPIO13 / L_EN) did NOT move! Check BTS7960.");
  } else if (!cwMoved && ccwMoved) {
    snprintf(calStatusMsg, sizeof(calStatusMsg),
             "FAULT: RPWM (GPIO14 / R_EN) did NOT move! Check BTS7960.");
  } else {
    snprintf(calStatusMsg, sizeof(calStatusMsg),
             "FAULT: No movement! Check motor power & I2C (SDA12/SCL11).");
  }

  readAS5600Raw(lastRaw);
  homeOffset = totalTicks;
  currentDeg = 0.0f;
  prevDeg = 0.0f;
  targetDeg = 0.0f;
  currentProfiledTarget = 0.0f;
  resetControllers();
  isCalibrating = false;
}

// ================= CORE 1 RTOS CONTROL TASK (~500 Hz, 100 kHz I2C SAFE) =================
void controlLoopTask(void * pvParameters) {
  lastLoopTime = micros();
  unsigned long hzTimer = millis();
  int hzCounter = 0;

  for (;;) {
    if (reqI2CRecover) {
      reqI2CRecover = false;
      recoverI2C();
    }

    if (reqAutoCal) {
      reqAutoCal = false;
      runAutoCalibration();
      lastLoopTime = micros();
      continue;
    }

    unsigned long now = micros();
    float dt = (now - lastLoopTime) * 1e-6f;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.002f;
    lastLoopTime = now;

    hzCounter++;
    if (millis() - hzTimer >= 500) {
      loopHz = hzCounter * 2;
      hzCounter = 0;
      hzTimer = millis();
    }

    updateEncoder(dt);

    if (!servoEnabled || isStalled || !encHealthy || g_mode == MODE_IDLE) {
      writeMotorRaw(0);
      integral = 0.0f;
      velIntegral = 0.0f;
      currentProfiledTarget = currentDeg;
      vTaskDelay(2);
      continue;
    }

    if (g_mode == MODE_MANUAL) {
      writeMotorRaw(manualPWM);
      currentProfiledTarget = currentDeg;
    }
    else if (g_mode == MODE_ANGLE) {
      float maxChange = maxVelocity * dt;
      if (targetDeg > currentProfiledTarget + maxChange)      currentProfiledTarget += maxChange;
      else if (targetDeg < currentProfiledTarget - maxChange) currentProfiledTarget -= maxChange;
      else                                                    currentProfiledTarget = targetDeg;

      float error = currentProfiledTarget - currentDeg;

      if (fabsf(error) < 35.0f) {
        integral = constrain(integral + error * dt, -150.0f, 150.0f);
      } else {
        integral = 0.0f;
      }

      float deriv = (error - lastError) / dt;
      float out = (Kp * error) + (Ki * integral) + (Kd * deriv);
      lastError = error;

      if (stallEnable && fabsf(out) >= (maxPWM - 5) && fabsf(currentVel) < 1.0f) {
        if (stallStartTime == 0) stallStartTime = millis();
        else if (millis() - stallStartTime > (unsigned long)stallMs) isStalled = true;
      } else {
        stallStartTime = 0;
      }

      drivePIDOutput(out, true);
    }
    else if (g_mode == MODE_SPEED) {
      if (fabsf(targetVel) < 0.5f) {
        velIntegral = 0.0f;
        writeMotorRaw(0);
      } else {
        float vErr = targetVel - currentVel;
        velIntegral = constrain(velIntegral + vErr * dt, -255.0f, 255.0f);
        float out = (velKp * vErr) + (velKi * velIntegral);
        drivePIDOutput(out, false);
      }
      currentProfiledTarget = currentDeg;
    }

    vTaskDelay(2); // ~500 Hz, stable for 100 kHz I2C & Wi-Fi
  }
}

// ================= NVS LOAD / SAVE =================
void loadSettings() {
  prefs.begin("servo-v4i2c", false);
  homeOffset  = prefs.getLong("offset", 0);
  Kp          = prefs.getFloat("kp", 5.0f);
  Ki          = prefs.getFloat("ki", 0.08f);
  Kd          = prefs.getFloat("kd", 0.08f);
  velKp       = prefs.getFloat("vKp", 1.2f);
  velKi       = prefs.getFloat("vKi", 2.0f);
  tolerance   = prefs.getFloat("tol", 0.5f);
  maxDeg      = prefs.getFloat("maxDeg", 360.0f);
  gearRatio   = prefs.getFloat("ratio", 220.0f);
  maxVelocity = prefs.getFloat("vel", 180.0f);
  velAlpha    = prefs.getFloat("alpha", 0.20f);
  pwmFreq     = prefs.getInt("freq", 7000);
  maxPWM      = prefs.getInt("maxpwm", 255);
  minPWM_CW   = prefs.getInt("minCW", 45);
  minPWM_CCW  = prefs.getInt("minCCW", 45);
  encDir      = prefs.getInt("dir", 1);
  spikeLimit  = prefs.getInt("spike", 1600);
  stallEnable = prefs.getBool("stEn", false);
  stallMs     = prefs.getInt("stMs", 2000);
}

void saveSettingsToNVS() {
  prefs.putFloat("kp", Kp); prefs.putFloat("ki", Ki); prefs.putFloat("kd", Kd);
  prefs.putFloat("vKp", velKp); prefs.putFloat("vKi", velKi);
  prefs.putFloat("tol", tolerance); prefs.putFloat("maxDeg", maxDeg);
  prefs.putFloat("ratio", gearRatio); prefs.putFloat("vel", maxVelocity);
  prefs.putFloat("alpha", velAlpha); prefs.putInt("freq", pwmFreq);
  prefs.putInt("maxpwm", maxPWM); prefs.putInt("minCW", minPWM_CW);
  prefs.putInt("minCCW", minPWM_CCW); prefs.putInt("dir", encDir);
  prefs.putInt("spike", spikeLimit);
  prefs.putBool("stEn", stallEnable); prefs.putInt("stMs", stallMs);
}

// ================= ADVANCED MULTI-TAB HTML DASHBOARD =================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SmartServo V4 Pro (I2C)</title>
<style>
  :root{--bg:#0b0f17;--card:#141b29;--bdr:#24324c;--txt:#e6edf7;--mut:#8b9bb4;--acc:#38bdf8;--ok:#22c55e;--warn:#f59e0b;--err:#ef4444;}
  *{box-sizing:border-box;font-family:system-ui,-apple-system,sans-serif;}
  body{margin:0;padding:12px;background:var(--bg);color:var(--txt);}
  .wrap{max-width:760px;margin:0 auto;display:grid;gap:10px;}
  .top{display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:8px;}
  h2{margin:0;font-size:1.2rem;letter-spacing:.3px;}
  .badge{padding:3px 9px;border-radius:99px;font-size:.73rem;font-weight:700;background:#1e293b;border:1px solid var(--bdr);}
  .grid4{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:8px;}
  .card{background:var(--card);border:1px solid var(--bdr);border-radius:12px;padding:13px;}
  .lbl{font-size:.72rem;color:var(--mut);text-transform:uppercase;letter-spacing:.4px;}
  .val{font-size:1.35rem;font-weight:700;margin-top:3px;font-variant-numeric:tabular-nums;}
  .nav{display:flex;gap:6px;flex-wrap:wrap;}
  .ntab{flex:1;min-width:85px;padding:9px;border-radius:8px;border:1px solid var(--bdr);background:#0f172a;color:var(--txt);font-weight:700;cursor:pointer;font-size:.83rem;}
  .ntab.active{background:var(--acc);color:#06121f;border-color:var(--acc);}
  .panel{display:none;}
  .panel.active{display:grid;gap:10px;}
  .row{display:flex;align-items:center;justify-content:space-between;gap:8px;margin:7px 0;flex-wrap:wrap;font-size:.88rem;}
  .row label{color:var(--mut);min-width:125px;}
  input[type=range]{flex:1;min-width:120px;accent-color:var(--acc);}
  input[type=number]{width:86px;padding:6px 8px;border-radius:6px;border:1px solid var(--bdr);background:#090e18;color:var(--txt);font-weight:700;text-align:center;}
  .btn{padding:8px 12px;border-radius:8px;border:1px solid var(--bdr);background:#1e293b;color:var(--txt);font-weight:700;cursor:pointer;font-size:.82rem;}
  .btn:hover{border-color:var(--acc);}
  .btn.pri{background:var(--acc);color:#06121f;border-color:var(--acc);}
  .btn.ok{background:var(--ok);color:#041a0d;border-color:var(--ok);}
  .btn.warn{background:var(--warn);color:#1f1300;border-color:var(--warn);}
  .btn.err{background:var(--err);color:#fff;border-color:var(--err);}
  .alert{background:var(--err);color:#fff;padding:10px 12px;border-radius:8px;font-weight:700;display:none;justify-content:space-between;align-items:center;}
  canvas{width:100%;height:175px;background:#070a10;border-radius:8px;border:1px solid var(--bdr);display:block;}
  .bar-bg{width:100%;height:14px;background:#090e18;border-radius:7px;overflow:hidden;border:1px solid var(--bdr);}
  .bar-fill{height:100%;background:linear-gradient(90deg,#38bdf8,#22c55e);width:0%;}
  .calbox{background:#090e18;border:1px solid var(--bdr);padding:10px;border-radius:8px;font-family:monospace;font-size:.82rem;color:#38bdf8;}
</style>
</head>
<body>
<div class="wrap">
  <div class="top">
    <div>
      <h2>SmartServo V4 Pro (I2C 100kHz)</h2>
      <span id="encBadge" class="badge">I2C: OK</span>
      <span id="hzBadge" class="badge">Loop: -- Hz</span>
      <span id="modeBadge" class="badge">Mode: ANGLE</span>
    </div>
    <div style="display:flex;gap:6px;">
      <button class="btn ok" onclick="cmd({zero:1})">Set Zero</button>
      <button class="btn err" id="enBtn" onclick="toggleEnable()">E-STOP</button>
    </div>
  </div>

  <div id="stallAlert" class="alert">
    <span>MOTOR STALL DETECTED</span>
    <button class="btn" onclick="cmd({resetstall:1})">CLEAR FAULT</button>
  </div>

  <div class="grid4">
    <div class="card"><div class="lbl">Output Angle</div><div class="val" id="vPos" style="color:var(--ok)">0.0°</div></div>
    <div class="card"><div class="lbl">Target</div><div class="val" id="vTgt" style="color:var(--acc)">0.0°</div></div>
    <div class="card"><div class="lbl">Speed (°/s)</div><div class="val" id="vVel">0.0</div></div>
    <div class="card"><div class="lbl">PWM / I2C Raw</div><div class="val" id="vPwmRaw">0 / 0</div></div>
  </div>

  <div class="card">
    <div style="display:flex;justify-content:space-between;margin-bottom:5px;font-size:.78rem;color:var(--mut);">
      <span>Live Graph (<span style="color:#38bdf8">Target</span> vs <span style="color:#22c55e">Actual</span>)</span>
      <span id="vTurns">Motor Turns: 0.0</span>
    </div>
    <canvas id="chart" width="700" height="175"></canvas>
  </div>

  <div class="nav">
    <button class="ntab active" onclick="showTab(0,this)">Control</button>
    <button class="ntab" onclick="showTab(1,this)">PID & Motion</button>
    <button class="ntab" onclick="showTab(2,this)">Calibration</button>
    <button class="ntab" onclick="showTab(3,this)">Hardware</button>
  </div>

  <!-- TAB 0: CONTROL -->
  <div class="panel active" id="tab0">
    <div class="card">
      <div class="lbl" style="margin-bottom:8px;">Operating Mode</div>
      <div class="nav">
        <button class="ntab" id="m0" onclick="setMode(0)">IDLE</button>
        <button class="ntab active" id="m1" onclick="setMode(1)">ANGLE LOCK</button>
        <button class="ntab" id="m2" onclick="setMode(2)">SPEED PID</button>
        <button class="ntab" id="m3" onclick="setMode(3)">MANUAL PWM</button>
      </div>

      <div id="boxAngle" style="margin-top:12px;">
        <div class="row">
          <label>Target Angle (°)</label>
          <input type="range" id="sAng" min="-360" max="360" step="1" value="0" oninput="nAng.value=this.value;cmd({tDeg:this.value})">
          <input type="number" id="nAng" value="0" step="5" onchange="sAng.value=this.value;cmd({tDeg:this.value})">
        </div>
        <div class="row" style="justify-content:center;">
          <button class="btn" onclick="setAng(-180)">-180°</button>
          <button class="btn" onclick="setAng(-90)">-90°</button>
          <button class="btn pri" onclick="setAng(0)">0°</button>
          <button class="btn" onclick="setAng(90)">+90°</button>
          <button class="btn" onclick="setAng(180)">+180°</button>
          <button class="btn" onclick="setAng(360)">+360°</button>
        </div>
      </div>

      <div id="boxSpeed" style="display:none;margin-top:12px;">
        <div class="row">
          <label>Target Speed (°/s)</label>
          <input type="range" id="sSpd" min="-200" max="200" step="5" value="0" oninput="nSpd.value=this.value;cmd({tVel:this.value})">
          <input type="number" id="nSpd" value="0" step="5" onchange="sSpd.value=this.value;cmd({tVel:this.value})">
        </div>
        <div class="row" style="justify-content:center;">
          <button class="btn" onclick="setSpd(-90)">-90°/s</button>
          <button class="btn" onclick="setSpd(-30)">-30°/s</button>
          <button class="btn pri" onclick="setSpd(0)">STOP</button>
          <button class="btn" onclick="setSpd(30)">+30°/s</button>
          <button class="btn" onclick="setSpd(90)">+90°/s</button>
        </div>
      </div>

      <div id="boxManual" style="display:none;margin-top:12px;">
        <div class="row">
          <label>Direct PWM</label>
          <input type="range" id="sMan" min="-255" max="255" step="5" value="0" oninput="nMan.value=this.value;cmd({mPwm:this.value})">
          <input type="number" id="nMan" value="0" step="5" onchange="sMan.value=this.value;cmd({mPwm:this.value})">
        </div>
        <div class="row" style="justify-content:center;">
          <button class="btn warn" onclick="setMan(-160)">Test CCW (-160)</button>
          <button class="btn err" onclick="setMan(0)">STOP (0)</button>
          <button class="btn ok" onclick="setMan(160)">Test CW (+160)</button>
        </div>
      </div>

      <div class="row" style="margin-top:12px;">
        <label>Speed Limit (°/s)</label>
        <input type="range" id="sVelLim" min="10" max="500" step="5" value="180" oninput="nVelLim.value=this.value;cmd({vel:this.value})">
        <input type="number" id="nVelLim" value="180" step="5" onchange="sVelLim.value=this.value;cmd({vel:this.value})">
      </div>
      <div class="row">
        <label>Max PWM (Torque)</label>
        <input type="range" id="sMaxPwm" min="60" max="255" step="5" value="255" oninput="nMaxPwm.value=this.value;cmd({maxpwm:this.value})">
        <input type="number" id="nMaxPwm" value="255" step="5" onchange="sMaxPwm.value=this.value;cmd({maxpwm:this.value})">
      </div>
    </div>
  </div>

  <!-- TAB 1: PID & MOTION -->
  <div class="panel" id="tab1">
    <div class="card">
      <div class="lbl" style="margin-bottom:8px;">Angle Lock PID</div>
      <div class="row"><label>Pos Kp (Stiffness)</label><input type="number" id="inKp" step="0.2"></div>
      <div class="row"><label>Pos Ki (Steady Lock)</label><input type="number" id="inKi" step="0.02"></div>
      <div class="row"><label>Pos Kd (Damping)</label><input type="number" id="inKd" step="0.02"></div>
      <div class="row"><label>Deadzone Tol (°)</label><input type="number" id="inTol" step="0.1"></div>
      <div class="lbl" style="margin:12px 0 8px;">Speed Mode PID & Filtering</div>
      <div class="row"><label>Speed Kp</label><input type="number" id="inVKp" step="0.1"></div>
      <div class="row"><label>Speed Ki</label><input type="number" id="inVKi" step="0.1"></div>
      <div class="row"><label>Vel Filter Alpha</label><input type="number" id="inAlpha" step="0.05" min="0.05" max="1"></div>
      <button class="btn ok" style="width:100%;margin-top:8px;padding:11px;" onclick="saveAll()">APPLY & SAVE TO FLASH</button>
    </div>
  </div>

  <!-- TAB 2: CALIBRATION -->
  <div class="panel" id="tab2">
    <div class="card">
      <div class="lbl" style="margin-bottom:8px;">Auto Calibration (CW/CCW H-Bridge Check + Polarity)</div>
      <div class="calbox" id="calMsg">Ready</div>
      <button class="btn warn" style="width:100%;margin:10px 0;padding:11px;" onclick="runAutoCal()">RUN AUTO CALIBRATION (SPINS CW & CCW)</button>

      <div class="lbl" style="margin-top:10px;">Live AS5600 I2C (0x0E)</div>
      <div class="row"><span>Raw Angle: <b id="cRaw">0</b> / 4095</span><span>I2C Recoveries: <b id="cRec">0</b></span></div>
      <div class="bar-bg"><div class="bar-fill" id="cBar"></div></div>

      <div class="row" style="margin-top:12px;">
        <label>Encoder Polarity</label>
        <button class="btn" id="dirBtn" onclick="toggleDir()">Normal (+1)</button>
      </div>
      <div class="row">
        <label>Spike Reject (ticks)</label>
        <input type="number" id="inSpike" min="400" max="2040" step="100">
      </div>
      <div style="display:flex;gap:8px;margin-top:8px;">
        <button class="btn" style="flex:1;" onclick="cmd({recI2C:1})">Force I2C Reset</button>
        <button class="btn pri" style="flex:2;" onclick="saveAll()">SAVE CALIBRATION</button>
      </div>
    </div>
  </div>

  <!-- TAB 3: HARDWARE -->
  <div class="panel" id="tab3">
    <div class="card">
      <div class="lbl" style="margin-bottom:8px;">Gearbox, Asymmetric Friction & PWM</div>
      <div class="row"><label>Gear Ratio (1:X)</label><input type="number" id="inRatio" step="1"></div>
      <div class="row"><label>Slider Max Angle (±°)</label><input type="number" id="inMaxDeg" step="10"></div>
      <div class="row"><label>Min PWM CW (+)</label><input type="number" id="inMinCW" min="0" max="150"></div>
      <div class="row"><label>Min PWM CCW (-)</label><input type="number" id="inMinCCW" min="0" max="150"></div>
      <div class="row"><label>PWM Freq (Hz)</label><input type="number" id="inFreq" step="500" min="1000" max="25000"></div>
      <div class="lbl" style="margin:12px 0 8px;">Safety</div>
      <div class="row"><label>Stall Protection</label><input type="checkbox" id="inStallEn" style="width:22px;height:22px;"></div>
      <div class="row"><label>Stall Timeout (ms)</label><input type="number" id="inStallMs" step="200" min="500" max="10000"></div>
      <div style="display:flex;gap:8px;margin-top:10px;">
        <button class="btn ok" style="flex:2;padding:11px;" onclick="saveAll()">APPLY & SAVE ALL</button>
        <button class="btn err" style="flex:1;padding:11px;" onclick="factoryReset()">FACTORY RESET</button>
      </div>
    </div>
  </div>
</div>

<script>
let curMode=1, enabled=true, encDir=1;
const histT=[], histA=[], MAX_PTS=120;
const ctx=document.getElementById('chart').getContext('2d');
const modeNames=['IDLE','ANGLE LOCK','SPEED PID','MANUAL PWM'];

function cmd(o){ fetch('/cmd?'+new URLSearchParams(o).toString()).catch(()=>{}); }
function showTab(idx,el){
  document.querySelectorAll('.panel').forEach((p,i)=>p.classList.toggle('active',i===idx));
  el.parentElement.querySelectorAll('.ntab').forEach(b=>b.classList.remove('active'));
  el.classList.add('active');
}
function syncModeUI(m){
  curMode=m;
  for(let i=0;i<4;i++) document.getElementById('m'+i).classList.toggle('active',i===m);
  boxAngle.style.display=(m===1)?'block':'none';
  boxSpeed.style.display=(m===2)?'block':'none';
  boxManual.style.display=(m===3)?'block':'none';
  modeBadge.textContent='Mode: '+modeNames[m];
}
function setMode(m){
  syncModeUI(m);
  histT.length=0; histA.length=0;
  cmd({mode:m});
}
function setAng(v){ sAng.value=v; nAng.value=v; cmd({tDeg:v}); }
function setSpd(v){ sSpd.value=v; nSpd.value=v; cmd({tVel:v}); }
function setMan(v){ sMan.value=v; nMan.value=v; cmd({mPwm:v}); }
function toggleEnable(){
  enabled=!enabled;
  cmd({en:enabled?1:0});
  enBtn.textContent=enabled?'E-STOP':'ENABLE';
  enBtn.className=enabled?'btn err':'btn ok';
}
function toggleDir(){
  encDir=(encDir===1)?-1:1;
  dirBtn.textContent=(encDir===1)?'Normal (+1)':'Inverted (-1)';
  cmd({dir:encDir});
}
function runAutoCal(){
  if(confirm('Motor will spin CW then CCW for ~2 seconds to check both H-bridge directions & set polarity. Continue?')){
    cmd({cal:1});
  }
}
function factoryReset(){
  if(confirm('Reset all settings to defaults?')) fetch('/factory').then(()=>location.reload());
}
function saveAll(){
  const q=new URLSearchParams({
    p:inKp.value, i:inKi.value, d:inKd.value, vP:inVKp.value, vI:inVKi.value,
    tol:inTol.value, max:inMaxDeg.value, r:inRatio.value, vel:nVelLim.value,
    maxpwm:nMaxPwm.value, minCW:inMinCW.value, minCCW:inMinCCW.value,
    dir:encDir, freq:inFreq.value, spike:inSpike.value,
    alpha:inAlpha.value, stallEn:inStallEn.checked?1:0, stallMs:inStallMs.value
  }).toString();
  fetch('/save?'+q).then(()=>{
    const mx=parseFloat(inMaxDeg.value)||360;
    sAng.min=-mx; sAng.max=mx;
    alert('Settings Applied & Saved to Flash!');
  });
}

function drawChart(){
  const w=ctx.canvas.width, h=ctx.canvas.height;
  ctx.clearRect(0,0,w,h);
  if(histA.length<2) return;
  let mn=Infinity, mx=-Infinity;
  for(let k=0;k<histA.length;k++){ mn=Math.min(mn,histA[k],histT[k]); mx=Math.max(mx,histA[k],histT[k]); }
  if(mx-mn<5){ mx+=2.5; mn-=2.5; }
  const pad=(mx-mn)*0.12; mn-=pad; mx+=pad;
  const mapY=v=>h-((v-mn)/(mx-mn))*h;

  ctx.strokeStyle='#1e293b'; ctx.lineWidth=1;
  ctx.beginPath(); ctx.moveTo(0,h/2); ctx.lineTo(w,h/2); ctx.stroke();

  ctx.strokeStyle='#38bdf8'; ctx.lineWidth=2; ctx.beginPath();
  histT.forEach((v,i)=>{ const x=(i/(MAX_PTS-1))*w; i?ctx.lineTo(x,mapY(v)):ctx.moveTo(x,mapY(v)); });
  ctx.stroke();

  ctx.strokeStyle='#22c55e'; ctx.lineWidth=2.2; ctx.beginPath();
  histA.forEach((v,i)=>{ const x=(i/(MAX_PTS-1))*w; i?ctx.lineTo(x,mapY(v)):ctx.moveTo(x,mapY(v)); });
  ctx.stroke();
}

function loadParams(){
  fetch('/getparams').then(r=>r.json()).then(d=>{
    inKp.value=d.p; inKi.value=d.i; inKd.value=d.d; inVKp.value=d.vP; inVKi.value=d.vI;
    inTol.value=d.tol; inMaxDeg.value=d.max; inRatio.value=d.r;
    sVelLim.value=d.vel; nVelLim.value=d.vel;
    sMaxPwm.value=d.maxpwm; nMaxPwm.value=d.maxpwm;
    inMinCW.value=d.minCW; inMinCCW.value=d.minCCW;
    inFreq.value=d.freq; inSpike.value=d.spike;
    inAlpha.value=d.alpha; inStallEn.checked=d.stallEn; inStallMs.value=d.stallMs;
    encDir=d.dir; dirBtn.textContent=(encDir===1)?'Normal (+1)':'Inverted (-1)';
    sAng.min=-d.max; sAng.max=d.max;
  });
}
loadParams();

setInterval(()=>{
  fetch('/status').then(r=>r.json()).then(d=>{
    if(d.mode !== curMode) syncModeUI(d.mode);
    vPos.textContent=d.pos.toFixed(1)+'°';
    vTgt.textContent=(d.mode===2)?(d.tVel.toFixed(1)+'°/s'):(d.tgt.toFixed(1)+'°');
    vVel.textContent=d.vel.toFixed(1);
    vPwmRaw.textContent=d.pwm+' / '+d.raw;
    vTurns.textContent='Motor Turns: '+d.turns.toFixed(1);
    hzBadge.textContent='Loop: '+d.hz+' Hz';
    stallAlert.style.display=d.stall?'flex':'none';

    encBadge.textContent=d.ok?('I2C: OK (Rec '+d.rec+')'):'I2C FAULT!';
    encBadge.style.borderColor=d.ok?'#22c55e':'#ef4444';

    cRaw.textContent=d.raw;
    cRec.textContent=d.rec;
    cBar.style.width=((d.raw/4095)*100).toFixed(1)+'%';
    calMsg.textContent=d.calMsg;

    const tVal=(d.mode===2)?d.tVel:d.tgt;
    const aVal=(d.mode===2)?d.vel:d.pos;
    histT.push(tVal); histA.push(aVal);
    if(histT.length>MAX_PTS){ histT.shift(); histA.shift(); }
    drawChart();
  }).catch(()=>{});
}, 130);
</script>
</body>
</html>
)rawliteral";

// ================= SETUP & ROUTES =================
void setup() {
  Serial.begin(115200);
  delay(400);

  // Noise-immune 100 kHz I2C setup (0x0E only)
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(5);

  loadSettings();
  setupPWM(pwmFreq);

  if (readAS5600Raw(lastRaw)) {
    totalTicks = lastRaw;
    liveRaw = lastRaw;
  }
  updateEncoder(0.01f);
  targetDeg = currentDeg;
  currentProfiledTarget = currentDeg;

  // Wi-Fi AP + STA (connects to 'iot' AND keeps 'SmartServo_V2' AP)
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_AP_STA);
  WiFi.setTxPower(WIFI_POWER_17dBm);
  WiFi.softAP("SmartServo_V2", "12345678");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.printf("Connecting to '%s'", WIFI_SSID);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("STA ('iot') -> http://");
    Serial.println(WiFi.localIP());
  }
  Serial.print("AP ('SmartServo_V2', pass 12345678) -> http://");
  Serial.println(WiFi.softAPIP());

  // Start ~500 Hz Control Loop on Core 1
  xTaskCreatePinnedToCore(controlLoopTask, "ServoV4_I2C", 4096, NULL, 1, &controlTaskHandle, 1);

  // HTTP Routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send_P(200, "text/html", index_html); });

  server.on("/cmd", HTTP_GET, [](AsyncWebServerRequest *r){
    if (r->hasParam("mode"))       { g_mode = (uint8_t)r->getParam("mode")->value().toInt(); resetControllers(); }
    if (r->hasParam("tDeg"))       targetDeg = r->getParam("tDeg")->value().toFloat();
    if (r->hasParam("tVel"))       targetVel = r->getParam("tVel")->value().toFloat();
    if (r->hasParam("mPwm"))       manualPWM = constrain(r->getParam("mPwm")->value().toInt(), -255, 255);
    if (r->hasParam("vel"))        maxVelocity = constrain(r->getParam("vel")->value().toFloat(), 5.0f, 1000.0f);
    if (r->hasParam("maxpwm"))     maxPWM = constrain(r->getParam("maxpwm")->value().toInt(), 60, 255);
    if (r->hasParam("en"))         { servoEnabled = (r->getParam("en")->value().toInt() == 1); if (servoEnabled) resetControllers(); }
    if (r->hasParam("dir"))        { encDir = (r->getParam("dir")->value().toInt() >= 0) ? 1 : -1; resetControllers(); }
    if (r->hasParam("resetstall")) resetControllers();
    if (r->hasParam("cal"))        reqAutoCal = true;
    if (r->hasParam("recI2C"))     reqI2CRecover = true;
    if (r->hasParam("zero")) {
      homeOffset = totalTicks;
      targetDeg = 0; currentProfiledTarget = 0; currentDeg = 0; prevDeg = 0;
      prefs.putLong("offset", homeOffset);
    }
    r->send(200, "text/plain", "OK");
  });

  server.on("/save", HTTP_GET, [](AsyncWebServerRequest *r){
    if (r->hasParam("p"))       Kp          = r->getParam("p")->value().toFloat();
    if (r->hasParam("i"))       Ki          = r->getParam("i")->value().toFloat();
    if (r->hasParam("d"))       Kd          = r->getParam("d")->value().toFloat();
    if (r->hasParam("vP"))      velKp       = r->getParam("vP")->value().toFloat();
    if (r->hasParam("vI"))      velKi       = r->getParam("vI")->value().toFloat();
    if (r->hasParam("tol"))     tolerance   = r->getParam("tol")->value().toFloat();
    if (r->hasParam("max"))     maxDeg      = r->getParam("max")->value().toFloat();
    if (r->hasParam("r"))       gearRatio   = r->getParam("r")->value().toFloat();
    if (r->hasParam("vel"))     maxVelocity = r->getParam("vel")->value().toFloat();
    if (r->hasParam("maxpwm"))  maxPWM      = constrain(r->getParam("maxpwm")->value().toInt(), 60, 255);
    if (r->hasParam("minCW"))   minPWM_CW   = constrain(r->getParam("minCW")->value().toInt(), 0, 150);
    if (r->hasParam("minCCW"))  minPWM_CCW  = constrain(r->getParam("minCCW")->value().toInt(), 0, 150);
    if (r->hasParam("dir"))     encDir      = (r->getParam("dir")->value().toInt() >= 0) ? 1 : -1;
    if (r->hasParam("spike"))   spikeLimit  = constrain(r->getParam("spike")->value().toInt(), 400, 2040);
    if (r->hasParam("alpha"))   velAlpha    = constrain(r->getParam("alpha")->value().toFloat(), 0.05f, 1.0f);
    if (r->hasParam("stallEn")) stallEnable = (r->getParam("stallEn")->value().toInt() == 1);
    if (r->hasParam("stallMs")) stallMs     = constrain(r->getParam("stallMs")->value().toInt(), 500, 10000);
    if (r->hasParam("freq")) {
      int newFreq = constrain(r->getParam("freq")->value().toInt(), 1000, 25000);
      if (newFreq != pwmFreq) setupPWM(newFreq);
    }
    saveSettingsToNVS();
    resetControllers();
    r->send(200, "text/plain", "OK");
  });

  server.on("/factory", HTTP_GET, [](AsyncWebServerRequest *r){
    prefs.clear();
    loadSettings();
    setupPWM(pwmFreq);
    resetControllers();
    r->send(200, "text/plain", "OK");
  });

  server.on("/getparams", HTTP_GET, [](AsyncWebServerRequest *r){
    char json[384];
    snprintf(json, sizeof(json),
             "{\"p\":%.3f,\"i\":%.3f,\"d\":%.3f,\"vP\":%.2f,\"vI\":%.2f,\"tol\":%.2f,"
             "\"max\":%.1f,\"r\":%.1f,\"vel\":%.1f,\"maxpwm\":%d,\"minCW\":%d,\"minCCW\":%d,"
             "\"dir\":%d,\"freq\":%d,\"spike\":%d,\"alpha\":%.2f,\"stallEn\":%s,\"stallMs\":%d}",
             Kp, Ki, Kd, velKp, velKi, tolerance, maxDeg, gearRatio, maxVelocity, maxPWM,
             minPWM_CW, minPWM_CCW, encDir, pwmFreq, spikeLimit, velAlpha,
             stallEnable ? "true" : "false", stallMs);
    r->send(200, "application/json", json);
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *r){
    char json[340];
    snprintf(json, sizeof(json),
             "{\"pos\":%.2f,\"tgt\":%.2f,\"tVel\":%.1f,\"vel\":%.1f,\"pwm\":%d,\"raw\":%d,"
             "\"turns\":%.2f,\"mode\":%u,\"stall\":%s,\"hz\":%d,\"ok\":%s,\"rec\":%lu,\"calMsg\":\"%s\"}",
             currentDeg, targetDeg, targetVel, currentVel, currentPWM, liveRaw,
             (float)(totalTicks - homeOffset) / 4096.0f, g_mode,
             isStalled ? "true" : "false", loopHz,
             encHealthy ? "true" : "false", (unsigned long)i2cRecoveries, calStatusMsg);
    r->send(200, "application/json", json);
  });

  server.begin();
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

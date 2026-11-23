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

// ================= DEFAULT PID & MOTION CONSTANTS (56:1 FAST) =================
const float DEF_GEAR_RATIO = 56.0f;   // 56:1 gearbox
const float DEF_KP         = 7.5f;    // Crisp stiffness
const float DEF_KI         = 0.15f;   // Fast steady-state correction
const float DEF_KD         = 0.12f;   // Velocity damping
const float DEF_KFF        = 0.55f;   // Velocity Feedforward (PWM per deg/s)
const float DEF_VEL_KP     = 1.4f;
const float DEF_VEL_KI     = 2.5f;
const float DEF_TOL        = 0.50f;
const float DEF_ALPHA      = 0.30f;   // Low-lag velocity filter
const float DEF_MAX_VEL    = 360.0f;  // Fast output speed limit (deg/s)
const float DEF_ACCEL      = 900.0f;  // Snappy acceleration (deg/s^2)
const int   DEF_SLEW       = 24;      // Fast response while still protecting gears

// ================= CONTROL MODES =================
enum CtrlMode : uint8_t {
  MODE_IDLE   = 0,
  MODE_ANGLE  = 1,
  MODE_SPEED  = 2,
  MODE_MANUAL = 3
};

// ================= TUNABLE PARAMETERS (SAVED TO NVS) =================
float Kp          = DEF_KP;
float Ki          = DEF_KI;
float Kd          = DEF_KD;
float Kff         = DEF_KFF;
float velKp       = DEF_VEL_KP;
float velKi       = DEF_VEL_KI;
float tolerance   = DEF_TOL;
float maxDeg      = 360.0f;
float gearRatio   = DEF_GEAR_RATIO;
float maxVelocity = DEF_MAX_VEL;
float maxAccel    = DEF_ACCEL;
float velAlpha    = DEF_ALPHA;

int   pwmFreq     = 7000;
int   maxPWM      = 255;
int   minPWM_CW   = 40;
int   minPWM_CCW  = 40;
int   pwmSlew     = DEF_SLEW;
int   encDir      = 1;
int   spikeLimit  = 1600;
bool  stallEnable = false;
int   stallMs     = 2000;

// ================= LIVE STATE & TELEMETRY =================
volatile uint8_t  g_mode           = MODE_ANGLE;
volatile bool     servoEnabled     = true;
volatile bool     isStalled        = false;
volatile bool     isPositionLocked = true;
volatile bool     isCalibrating    = false;
volatile bool     isAutoTuning     = false;
volatile bool     reqAutoCal       = false;
volatile bool     reqAutoPid       = false;
volatile bool     reqI2CRecover    = false;
volatile int      tuneProgress     = 0;

volatile float    targetDeg        = 0.0f;
volatile float    targetVel        = 0.0f;
volatile int      manualPWM        = 0;

volatile float    currentDeg       = 0.0f;
volatile float    currentVel       = 0.0f;
volatile int      currentPWM       = 0;
volatile int      liveRaw          = 0;
volatile int      loopHz           = 0;
volatile bool     encHealthy       = true;
volatile uint32_t i2cRecoveries    = 0;

char calStatusMsg[112] = "Ready (Run Auto-Cal to verify CW/CCW, Polarity & I2C EMI)";
char pidStatusMsg[128] = "Ready (Tap 30s Auto-PID for 7-stage fast & smooth tuning)";

// ================= INTERNAL CONTROL VARS =================
bool  pwmInitialized = false;
float integral       = 0.0f;
float velIntegral    = 0.0f;
float profPos        = 0.0f;
float profVel        = 0.0f;
float lastTargetDeg  = 0.0f;
float prevDeg        = 0.0f;
float slewedPWM      = 0.0f;

int     lastRaw      = 0;
long    totalTicks   = 0;
long    homeOffset   = 0;
uint8_t i2cFailCount = 0;

unsigned long lastLoopTime   = 0;
unsigned long stallStartTime = 0;
unsigned long settleStartMs  = 0;

TaskHandle_t controlTaskHandle;
Preferences prefs;
AsyncWebServer server(80);

// ================= FORWARD DECLARATIONS =================
void saveSettingsToNVS();

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
  slewedPWM  = (float)pwm;
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

// Fast slew-limited PWM writer with hard zero cutoff (never dribbles idle PWM)
inline void writeMotorSmooth(float targetPwm, float dt) {
  targetPwm = constrain(targetPwm, (float)(-maxPWM), (float)maxPWM);

  if (targetPwm == 0.0f) {
    if (fabsf(slewedPWM) <= 24.0f) {
      writeMotorRaw(0);
      return;
    }
  }

  float maxStep = (float)max(pwmSlew, 2) * (dt * 1000.0f);
  if (maxStep < 1.5f) maxStep = 1.5f;

  if (targetPwm > slewedPWM + maxStep)      slewedPWM += maxStep;
  else if (targetPwm < slewedPWM - maxStep) slewedPWM -= maxStep;
  else                                      slewedPWM = targetPwm;

  int pwm = (int)roundf(slewedPWM);
  if (targetPwm == 0.0f && abs(pwm) <= 20) {
    pwm = 0;
    slewedPWM = 0.0f;
  }

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

// ================= AS5600 I2C (400 kHz, 0x0E ONLY, CLEAN STOP) =================
void recoverI2C() {
  writeMotorRaw(0);
  Wire.end();
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, OUTPUT);

  for (int i = 0; i < 9; i++) {
    digitalWrite(I2C_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL, HIGH);
    delayMicroseconds(5);
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(5);
  i2cRecoveries++;
}

inline bool readAS5600Raw(int &outRaw) {
  Wire.beginTransmission(0x36);
  Wire.write(0x0E); // Angle register ONLY
  if (Wire.endTransmission(true) != 0) return false;
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

void resetControllers(bool lockNow = false) {
  integral         = 0.0f;
  velIntegral      = 0.0f;
  isStalled        = false;
  stallStartTime   = 0;
  settleStartMs    = 0;
  profPos          = currentDeg;
  profVel          = 0.0f;
  lastTargetDeg    = targetDeg;
  isPositionLocked = lockNow;
  writeMotorRaw(0);
}

void resetPidDefaults() {
  Kp          = DEF_KP;
  Ki          = DEF_KI;
  Kd          = DEF_KD;
  Kff         = DEF_KFF;
  velKp       = DEF_VEL_KP;
  velKi       = DEF_VEL_KI;
  tolerance   = DEF_TOL;
  velAlpha    = DEF_ALPHA;
  maxVelocity = DEF_MAX_VEL;
  maxAccel    = DEF_ACCEL;
  pwmSlew     = DEF_SLEW;
  saveSettingsToNVS();
  resetControllers(true);
  snprintf(pidStatusMsg, sizeof(pidStatusMsg), "PID reset to fast 56:1 defaults (P:%.1f D:%.2f FF:%.2f)", Kp, Kd, Kff);
}

// ================= FAST ANGLE CONTROL + HARD 0-PWM IDLE LATCH =================
void runAngleLockStep(float dt, bool allowStallCheck) {
  float totalErr = targetDeg - currentDeg;

  // Unlock if target moved
  if (fabsf(targetDeg - lastTargetDeg) > 0.05f) {
    isPositionLocked = false;
    settleStartMs    = 0;
    lastTargetDeg    = targetDeg;
  }

  // Hysteresis band for waking up from silent 0-PWM lock
  float unlockBand = max(tolerance * 2.2f, tolerance + 0.55f);
  if (isPositionLocked) {
    if (fabsf(totalErr) > unlockBand) {
      isPositionLocked = false;
      settleStartMs    = 0;
      profPos          = currentDeg;
      profVel          = 0.0f;
    } else {
      // Strictly 0 PWM at idle
      integral = 0.0f;
      profPos  = targetDeg;
      profVel  = 0.0f;
      stallStartTime = 0;
      if (currentPWM != 0 || slewedPWM != 0.0f) writeMotorRaw(0);
      return;
    }
  }

  // 1. Fast Trapezoidal Acceleration-Limited Motion Profiler
  float rem  = targetDeg - profPos;
  float dist = fabsf(rem);
  float dir  = (rem >= 0.0f) ? 1.0f : -1.0f;
  float aMax = max(maxAccel, 50.0f);

  float desiredVel = 0.0f;
  if (dist > 0.02f) {
    float vBrake = sqrtf(2.0f * aMax * dist);
    desiredVel = dir * min(maxVelocity, vBrake);
  }

  float dvMax = aMax * dt;
  if (desiredVel > profVel + dvMax)      profVel += dvMax;
  else if (desiredVel < profVel - dvMax) profVel -= dvMax;
  else                                   profVel = desiredVel;

  profPos += profVel * dt;

  if (fabsf(targetDeg - profPos) < 0.04f && fabsf(profVel) < 2.0f) {
    profPos = targetDeg;
    profVel = 0.0f;
  }

  // 2A. Immediate silent 0-PWM lock when inside tolerance
  if (fabsf(totalErr) <= tolerance && fabsf(profVel) < 2.0f && fabsf(currentVel) < 5.0f) {
    isPositionLocked = true;
    settleStartMs    = 0;
    integral         = 0.0f;
    stallStartTime   = 0;
    writeMotorRaw(0);
    return;
  }

  // 2B. Settle-Timeout Latch: if friction holds shaft motionless right beside target for 220ms -> hard 0 PWM
  if (fabsf(profVel) < 2.0f && fabsf(totalErr) <= unlockBand && fabsf(currentVel) < 1.5f) {
    if (settleStartMs == 0) settleStartMs = millis();
    else if (millis() - settleStartMs >= 220) {
      isPositionLocked = true;
      settleStartMs    = 0;
      integral         = 0.0f;
      stallStartTime   = 0;
      writeMotorRaw(0);
      return;
    }
  } else {
    settleStartMs = 0;
  }

  // 3. Feedforward + Crisp PID with Velocity Damping
  float trackErr = profPos - currentDeg;
  if (fabsf(trackErr) < 30.0f) {
    integral = constrain(integral + trackErr * dt, -140.0f, 140.0f);
  } else {
    integral *= 0.85f;
  }

  float velErr = profVel - currentVel;
  float out = (Kff * profVel) + (Kp * trackErr) + (Ki * integral) + (Kd * velErr);

  // Stiction compensation (always enough to actually overcome friction, or 0)
  float minP = (out >= 0.0f) ? (float)minPWM_CW : (float)minPWM_CCW;
  float errScale = constrain(fabsf(totalErr) / max(tolerance * 2.5f, 0.8f), 0.25f, 1.0f);
  float velScale = constrain(fabsf(profVel) / 20.0f, 0.0f, 1.0f);
  float blend = max(errScale, velScale);

  float cmd = 0.0f;
  if (fabsf(out) > 1.5f) {
    float cmdMag = constrain(fabsf(out) + (minP * blend), 0.0f, (float)maxPWM);
    cmd = (out >= 0.0f) ? cmdMag : -cmdMag;
  }

  if (allowStallCheck && stallEnable && fabsf(cmd) >= (maxPWM - 5) && fabsf(currentVel) < 1.0f) {
    if (stallStartTime == 0) stallStartTime = millis();
    else if (millis() - stallStartTime > (unsigned long)stallMs) isStalled = true;
  } else {
    stallStartTime = 0;
  }

  writeMotorSmooth(cmd, dt);
}

// ================= AUTO CALIBRATION (CW/CCW H-BRIDGE + POLARITY + EMI CHECK) =================
void runAutoCalibration() {
  isCalibrating = true;
  snprintf(calStatusMsg, sizeof(calStatusMsg), "Calibrating: Spinning CW & checking I2C...");
  writeMotorRaw(0);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  int testPWM = constrain(max(135, max(minPWM_CW, minPWM_CCW) + 60), 100, maxPWM);
  int prevR = lastRaw;
  readAS5600Raw(prevR);
  int loadI2cErrs = 0;

  long cwTicks = 0;
  writeMotorRaw(testPWM);
  unsigned long t0 = millis();
  while (millis() - t0 < 800) {
    int r = prevR;
    if (readAS5600Raw(r)) {
      liveRaw = r;
      int d = r - prevR;
      if (d > 2048) d -= 4096;
      if (d < -2048) d += 4096;
      if (abs(d) < spikeLimit) { cwTicks += d; prevR = r; }
    } else {
      loadI2cErrs++;
    }
    vTaskDelay(1);
  }
  writeMotorRaw(0);
  vTaskDelay(220 / portTICK_PERIOD_MS);

  snprintf(calStatusMsg, sizeof(calStatusMsg), "Calibrating: Spinning CCW & checking I2C...");
  long ccwTicks = 0;
  readAS5600Raw(prevR);
  writeMotorRaw(-testPWM);
  t0 = millis();
  while (millis() - t0 < 800) {
    int r = prevR;
    if (readAS5600Raw(r)) {
      liveRaw = r;
      int d = r - prevR;
      if (d > 2048) d -= 4096;
      if (d < -2048) d += 4096;
      if (abs(d) < spikeLimit) { ccwTicks += d; prevR = r; }
    } else {
      loadI2cErrs++;
    }
    vTaskDelay(1);
  }
  writeMotorRaw(0);
  vTaskDelay(150 / portTICK_PERIOD_MS);

  bool cwMoved  = (abs(cwTicks)  > 400);
  bool ccwMoved = (abs(ccwTicks) > 400);

  if (cwMoved && ccwMoved) {
    encDir = (cwTicks > 0) ? 1 : -1;
    prefs.putInt("dir", encDir);
    snprintf(calStatusMsg, sizeof(calStatusMsg),
             "OK! Dir:%+d | CW:%ld CCW:%ld | I2C Errs:%d (Idle 0 PWM)",
             encDir, cwTicks, ccwTicks, loadI2cErrs);
  } else if (cwMoved && !ccwMoved) {
    snprintf(calStatusMsg, sizeof(calStatusMsg),
             "FAULT: LPWM (GPIO13 / L_EN) did NOT move! Check BTS7960.");
  } else if (!cwMoved && ccwMoved) {
    snprintf(calStatusMsg, sizeof(calStatusMsg),
             "FAULT: RPWM (GPIO14 / R_EN) did NOT move! Check BTS7960.");
  } else {
    snprintf(calStatusMsg, sizeof(calStatusMsg),
             "FAULT: No movement! Check motor power & I2C (Errs:%d).", loadI2cErrs);
  }

  readAS5600Raw(lastRaw);
  homeOffset = totalTicks;
  currentDeg = 0.0f;
  prevDeg    = 0.0f;
  targetDeg  = 0.0f;
  resetControllers(true); // Hard 0 PWM & locked
  isCalibrating = false;
}

// ================= OPTIMIZED 7-STAGE 30-SECOND FAST AUTO-PID =================
void runAutoPidTune() {
  isAutoTuning = true;
  tuneProgress = 2;
  writeMotorRaw(0);
  vTaskDelay(150 / portTICK_PERIOD_MS);

  unsigned long prevUs = micros();
  auto stepEnc = [&]() -> float {
    unsigned long nowUs = micros();
    float dt = (nowUs - prevUs) * 1e-6f;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.001f;
    prevUs = nowUs;
    updateEncoder(dt);
    return dt;
  };

  stepEnc();
  float centerDeg = currentDeg;
  float abortCode = 0.0f;

  struct StepMetrics {
    float maxOvershoot;
    float maxTrackLag;
    float steadyErr;
    int   riseMs;
    int   pwmFlips;
  };

  auto runStepTest = [&](float destDeg, unsigned long durationMs) -> StepMetrics {
    StepMetrics m = {0.0f, 0.0f, 0.0f, (int)durationMs, 0};
    float startDeg = currentDeg;
    float stepSpan = fabsf(destDeg - startDeg);

    targetDeg        = destDeg;
    lastTargetDeg    = destDeg;
    isPositionLocked = false;
    settleStartMs    = 0;
    profPos          = currentDeg;
    profVel          = 0.0f;
    integral         = 0.0f;

    float stepDir = (destDeg >= startDeg) ? 1.0f : -1.0f;
    bool reached90 = false;
    int lastSign = 0;
    float errSum = 0.0f;
    int errCnt = 0;

    unsigned long tStart = millis();
    while (millis() - tStart < durationMs) {
      if (!servoEnabled) { abortCode = 1.0f; break; }
      float dt = stepEnc();

      if (fabsf(currentDeg - centerDeg) > 120.0f) {
        snprintf(pidStatusMsg, sizeof(pidStatusMsg), "ABORTED: Runaway (>120 deg)! Flip Encoder Polarity.");
        abortCode = 2.0f;
        break;
      }

      runAngleLockStep(dt, false);

      unsigned long elap = millis() - tStart;
      if (!reached90 && stepSpan > 3.0f && fabsf(destDeg - currentDeg) <= stepSpan * 0.10f) {
        m.riseMs = (int)elap;
        reached90 = true;
      }

      float past = (currentDeg - destDeg) * stepDir;
      if (past > m.maxOvershoot) m.maxOvershoot = past;

      float lag = fabsf(profPos - currentDeg);
      if (lag > m.maxTrackLag) m.maxTrackLag = lag;

      if (elap > 250) {
        int s = (currentPWM > 22) ? 1 : (currentPWM < -22) ? -1 : 0;
        if (s != 0 && lastSign != 0 && s != lastSign) m.pwmFlips++;
        if (s != 0) lastSign = s;
      }
      if (elap > durationMs - 450) {
        errSum += fabsf(destDeg - currentDeg);
        errCnt++;
      }
      vTaskDelay(1);
    }
    m.steadyErr = (errCnt > 0) ? (errSum / errCnt) : 0.0f;
    return m;
  };

  // ---------- STAGE 1/7: CW & CCW BREAKAWAY FRICTION (~3.5s) ----------
  snprintf(pidStatusMsg, sizeof(pidStatusMsg), "[1/7] Measuring CW & CCW breakaway friction...");
  tuneProgress = 8;

  int foundCW = 40, foundCCW = 40;
  float sA = currentDeg;
  for (int p = 14; p <= min(maxPWM, 130); p += 3) {
    if (!servoEnabled) { abortCode = 1.0f; break; }
    writeMotorRaw(p);
    unsigned long tW = millis();
    while (millis() - tW < 22) { stepEnc(); vTaskDelay(1); }
    if (fabsf(currentDeg - sA) > 0.35f) { foundCW = p; break; }
  }
  writeMotorRaw(0);
  vTaskDelay(160 / portTICK_PERIOD_MS);

  sA = currentDeg;
  for (int p = 14; p <= min(maxPWM, 130); p += 3) {
    if (abortCode != 0.0f || !servoEnabled) { abortCode = 1.0f; break; }
    writeMotorRaw(-p);
    unsigned long tW = millis();
    while (millis() - tW < 22) { stepEnc(); vTaskDelay(1); }
    if (fabsf(currentDeg - sA) > 0.35f) { foundCCW = p; break; }
  }
  writeMotorRaw(0);
  vTaskDelay(160 / portTICK_PERIOD_MS);

  if (abortCode != 0.0f) {
    snprintf(pidStatusMsg, sizeof(pidStatusMsg), "Auto-Tune Stopped by E-STOP.");
    resetControllers(true); isAutoTuning = false; tuneProgress = 0; return;
  }

  minPWM_CW  = constrain((int)(foundCW  * 0.78f), 16, 95);
  minPWM_CCW = constrain((int)(foundCCW * 0.78f), 16, 95);

  // ---------- STAGE 2/7: GEARBOX BACKLASH CHECK (~2.5s) ----------
  snprintf(pidStatusMsg, sizeof(pidStatusMsg), "[2/7] Measuring 56:1 gearbox backlash...");
  tuneProgress = 18;

  int gentlePWM = constrain(max(foundCW, foundCCW) + 28, 65, 135);
  unsigned long t0 = millis();
  while (millis() - t0 < 300 && abortCode == 0.0f) {
    if (!servoEnabled) { abortCode = 1.0f; break; }
    stepEnc(); writeMotorRaw(gentlePWM); vTaskDelay(1);
  }
  writeMotorRaw(0);
  vTaskDelay(140 / portTICK_PERIOD_MS);

  float revStart = currentDeg;
  float backlashEst = 0.4f;
  t0 = millis();
  while (millis() - t0 < 400 && abortCode == 0.0f) {
    if (!servoEnabled) { abortCode = 1.0f; break; }
    stepEnc(); writeMotorRaw(-gentlePWM);
    if (fabsf(currentVel) > 12.0f) {
      backlashEst = fabsf(currentDeg - revStart);
      break;
    }
    vTaskDelay(1);
  }
  writeMotorRaw(0);
  vTaskDelay(140 / portTICK_PERIOD_MS);

  tolerance = constrain(max(0.35f, backlashEst * 0.55f), 0.35f, 1.0f);

  // ---------- STAGE 3/7: MULTI-SPEED Kff & MAX ACCEL MAP (~4.5s) ----------
  snprintf(pidStatusMsg, sizeof(pidStatusMsg), "[3/7] Mapping speed curve (Kff), accel & inertia...");
  tuneProgress = 30;

  int pLow  = constrain(max(foundCW, foundCCW) + 55, 100, min(maxPWM, 155));
  int pHigh = constrain(max(foundCW, foundCCW) + 125, 165, min(maxPWM, 235));
  if (pHigh <= pLow + 20) pHigh = min(maxPWM, pLow + 35);

  float vLow = 0.0f, vHigh = 0.0f, coastDeg = 0.0f;

  t0 = millis();
  while (millis() - t0 < 700 && abortCode == 0.0f) {
    if (!servoEnabled) { abortCode = 1.0f; break; }
    float dt = stepEnc();
    writeMotorSmooth((float)pLow, dt);
    if (millis() - t0 > 350) vLow = 0.85f * vLow + 0.15f * fabsf(currentVel);
    vTaskDelay(1);
  }
  writeMotorRaw(0);
  vTaskDelay(160 / portTICK_PERIOD_MS);

  t0 = millis();
  while (millis() - t0 < 700 && abortCode == 0.0f) {
    if (!servoEnabled) { abortCode = 1.0f; break; }
    float dt = stepEnc();
    writeMotorSmooth((float)(-pHigh), dt);
    if (millis() - t0 > 350) vHigh = 0.85f * vHigh + 0.15f * fabsf(currentVel);
    vTaskDelay(1);
  }
  float coastStart = currentDeg;
  writeMotorRaw(0);
  t0 = millis();
  while (millis() - t0 < 280) { stepEnc(); vTaskDelay(1); }
  coastDeg = fabsf(currentDeg - coastStart);

  if (abortCode != 0.0f || vHigh < 5.0f) {
    snprintf(pidStatusMsg, sizeof(pidStatusMsg),
             (abortCode == 1.0f) ? "Auto-Tune Stopped by E-STOP." : "FAULT: Motor too slow. Check power & polarity.");
    resetControllers(true); isAutoTuning = false; tuneProgress = 0; return;
  }

  float dVel      = max(vHigh - vLow, 5.0f);
  float dPwm      = (float)(pHigh - pLow);
  float plantGain = vHigh / (float)pHigh; // (deg/s) per PWM

  // Snappy, high-response starting parameters scaled for 56:1
  Kff         = constrain((dPwm / dVel) * 0.88f, 0.10f, 3.0f);
  Kp          = constrain(2.20f / max(plantGain, 0.08f), 2.5f, 22.0f);
  Ki          = constrain(Kp * 0.04f, 0.04f, 1.2f);
  Kd          = constrain((0.14f + coastDeg * 0.03f) / max(plantGain, 0.08f), 0.04f, 1.2f);
  velKp       = constrain(0.65f / max(plantGain, 0.08f), 0.5f, 10.0f);
  velKi       = constrain(velKp * 1.45f, 0.4f, 16.0f);
  maxVelocity = constrain(vHigh * 1.15f, 120.0f, 600.0f);
  maxAccel    = constrain(vHigh * 4.8f, 350.0f, 1400.0f);
  pwmSlew     = constrain((int)(28.0f - coastDeg * 0.8f), 14, 32);

  centerDeg = currentDeg;

  // ---------- STAGE 4/7: CRISP MICRO-STEP TEST (~5.5s) ----------
  float microTargets[3] = { centerDeg + 10.0f, centerDeg - 10.0f, centerDeg };
  for (int i = 0; i < 3 && abortCode == 0.0f; i++) {
    snprintf(pidStatusMsg, sizeof(pidStatusMsg),
             "[4/7] Crisp Micro-Step %d/3 (P:%.1f D:%.2f)...", i + 1, Kp, Kd);
    tuneProgress = 42 + i * 5;
    StepMetrics m = runStepTest(microTargets[i], 1700);
    if (abortCode != 0.0f) break;

    if (m.pwmFlips >= 3) {
      Kp *= 0.85f; Kd *= 1.22f; pwmSlew = max(pwmSlew - 2, 10);
    } else if (m.maxOvershoot > 1.2f) {
      Kd *= 1.22f;
    } else if (m.steadyErr > tolerance * 1.25f || m.riseMs > 380) {
      Kp *= 1.18f; Ki *= 1.25f;
    }
    Kp = constrain(Kp, 2.0f, 28.0f);
    Ki = constrain(Ki, 0.03f, 2.5f);
    Kd = constrain(Kd, 0.03f, 2.0f);
  }

  // ---------- STAGE 5/7: FAST LARGE-STEP REVERSALS (~6.5s) ----------
  float largeTargets[3] = { centerDeg + 60.0f, centerDeg - 60.0f, centerDeg };
  int lastRiseMs = 0;
  for (int i = 0; i < 3 && abortCode == 0.0f; i++) {
    snprintf(pidStatusMsg, sizeof(pidStatusMsg),
             "[5/7] Fast Reversal %d/3 (FF:%.2f Acc:%.0f)...", i + 1, Kff, maxAccel);
    tuneProgress = 58 + i * 7;
    StepMetrics m = runStepTest(largeTargets[i], 2000);
    lastRiseMs = m.riseMs;
    if (abortCode != 0.0f) break;

    if (m.pwmFlips >= 3) {
      Kp *= 0.84f; Kd *= 1.25f; pwmSlew = max(pwmSlew - 2, 10);
    } else if (m.maxOvershoot > 1.8f) {
      Kff *= 0.92f; Kd *= 1.25f; maxAccel = max(maxAccel * 0.90f, 250.0f);
    } else if (m.maxTrackLag > 4.5f) {
      Kff *= 1.12f; Kp *= 1.12f;
    } else {
      // Clean fast step -> push response slightly snappier
      Kp *= 1.06f; maxAccel = min(maxAccel * 1.08f, 1500.0f);
    }
    Kff = constrain(Kff, 0.10f, 3.5f);
    Kp  = constrain(Kp, 2.0f, 30.0f);
    Ki  = constrain(Ki, 0.03f, 2.5f);
    Kd  = constrain(Kd, 0.03f, 2.2f);
  }

  // ---------- STAGE 6/7: SPEED-MODE PID VERIFICATION (~3.5s) ----------
  if (abortCode == 0.0f) {
    snprintf(pidStatusMsg, sizeof(pidStatusMsg), "[6/7] Refining Speed PID (+90 / -90 deg/s)...");
    tuneProgress = 82;

    float testSpeeds[2] = { 90.0f, -90.0f };
    for (int s = 0; s < 2 && abortCode == 0.0f; s++) {
      float tV = testSpeeds[s];
      velIntegral = 0.0f;
      float vErrSum = 0.0f;
      int vCnt = 0;
      t0 = millis();
      while (millis() - t0 < 1300) {
        if (!servoEnabled) { abortCode = 1.0f; break; }
        float dt = stepEnc();
        float vErr = tV - currentVel;
        velIntegral = constrain(velIntegral + vErr * dt, -255.0f, 255.0f);
        float out = (Kff * tV) + (velKp * vErr) + (velKi * velIntegral);
        float minP = (out >= 0.0f) ? (float)minPWM_CW : (float)minPWM_CCW;
        float cmdMag = constrain(fabsf(out) + minP * 0.45f, 0.0f, (float)maxPWM);
        writeMotorSmooth((out >= 0.0f) ? cmdMag : -cmdMag, dt);

        if (millis() - t0 > 500) { vErrSum += fabsf(vErr); vCnt++; }
        vTaskDelay(1);
      }
      if (vCnt > 0 && (vErrSum / vCnt) > 10.0f) {
        velKp = constrain(velKp * 1.18f, 0.4f, 12.0f);
        velKi = constrain(velKi * 1.22f, 0.3f, 18.0f);
      }
      writeMotorRaw(0);
      vTaskDelay(150 / portTICK_PERIOD_MS);
      tuneProgress = 86 + s * 4;
    }
  }

  // ---------- STAGE 7/7: HARD 0-PWM IDLE LOCK CHECK (~2.5s) ----------
  if (abortCode == 0.0f) {
    snprintf(pidStatusMsg, sizeof(pidStatusMsg), "[7/7] Verifying Silent 0-PWM Idle Lock...");
    tuneProgress = 94;

    targetDeg        = centerDeg;
    lastTargetDeg    = centerDeg;
    isPositionLocked = false;
    settleStartMs    = 0;
    profPos          = currentDeg;
    profVel          = 0.0f;

    int nonZeroIdleCount = 0;
    t0 = millis();
    while (millis() - t0 < 2300) {
      if (!servoEnabled) { abortCode = 1.0f; break; }
      float dt = stepEnc();
      runAngleLockStep(dt, false);
      if (millis() - t0 > 1300 && currentPWM != 0) nonZeroIdleCount++;
      vTaskDelay(1);
    }

    if (abortCode == 0.0f) {
      if (nonZeroIdleCount > 40) {
        tolerance = constrain(tolerance + 0.15f, 0.4f, 1.2f);
        Kp *= 0.92f;
      }
      saveSettingsToNVS();
      tuneProgress = 100;
      snprintf(pidStatusMsg, sizeof(pidStatusMsg),
               "FAST 30s TUNE OK! P:%.1f I:%.2f D:%.2f FF:%.2f | Rise:%dms | 0-PWM Lock",
               Kp, Ki, Kd, Kff, lastRiseMs);
    }
  }

  if (abortCode == 1.0f) {
    snprintf(pidStatusMsg, sizeof(pidStatusMsg), "Auto-Tune Stopped by E-STOP.");
    tuneProgress = 0;
  }

  // Strictly 0 PWM & locked at current position when finished
  targetDeg = currentDeg;
  resetControllers(true);
  isAutoTuning = false;
}

// ================= CORE 1 RTOS CONTROL TASK (~1 kHz, 400 kHz I2C) =================
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

    if (reqAutoPid) {
      reqAutoPid = false;
      runAutoPidTune();
      lastLoopTime = micros();
      continue;
    }

    unsigned long now = micros();
    float dt = (now - lastLoopTime) * 1e-6f;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.001f;
    lastLoopTime = now;

    hzCounter++;
    if (millis() - hzTimer >= 500) {
      loopHz = hzCounter * 2;
      hzCounter = 0;
      hzTimer = millis();
    }

    updateEncoder(dt);

    if (!servoEnabled || isStalled || !encHealthy || g_mode == MODE_IDLE) {
      if (currentPWM != 0 || slewedPWM != 0.0f) writeMotorRaw(0);
      integral = 0.0f;
      velIntegral = 0.0f;
      profPos = currentDeg;
      profVel = 0.0f;
      vTaskDelay(1);
      continue;
    }

    if (g_mode == MODE_MANUAL) {
      if (manualPWM == 0) writeMotorRaw(0);
      else writeMotorSmooth((float)manualPWM, dt);
      profPos = currentDeg;
      profVel = 0.0f;
    }
    else if (g_mode == MODE_ANGLE) {
      runAngleLockStep(dt, true);
    }
    else if (g_mode == MODE_SPEED) {
      if (fabsf(targetVel) < 0.5f) {
        velIntegral = 0.0f;
        writeMotorRaw(0);
      } else {
        float vErr = targetVel - currentVel;
        velIntegral = constrain(velIntegral + vErr * dt, -255.0f, 255.0f);
        float out = (Kff * targetVel) + (velKp * vErr) + (velKi * velIntegral);
        float minP = (out >= 0.0f) ? (float)minPWM_CW : (float)minPWM_CCW;
        float cmdMag = constrain(fabsf(out) + minP * 0.45f, 0.0f, (float)maxPWM);
        writeMotorSmooth((out >= 0.0f) ? cmdMag : -cmdMag, dt);
      }
      profPos = currentDeg;
      profVel = currentVel;
    }

    vTaskDelay(1);
  }
}

// ================= NVS LOAD / SAVE =================
void loadSettings() {
  prefs.begin("servo-v53i2c", false);
  homeOffset  = prefs.getLong("offset", 0);
  Kp          = prefs.getFloat("kp", DEF_KP);
  Ki          = prefs.getFloat("ki", DEF_KI);
  Kd          = prefs.getFloat("kd", DEF_KD);
  Kff         = prefs.getFloat("kff", DEF_KFF);
  velKp       = prefs.getFloat("vKp", DEF_VEL_KP);
  velKi       = prefs.getFloat("vKi", DEF_VEL_KI);
  tolerance   = prefs.getFloat("tol", DEF_TOL);
  maxDeg      = prefs.getFloat("maxDeg", 360.0f);
  gearRatio   = prefs.getFloat("ratio", DEF_GEAR_RATIO);
  maxVelocity = prefs.getFloat("vel", DEF_MAX_VEL);
  maxAccel    = prefs.getFloat("acc", DEF_ACCEL);
  velAlpha    = prefs.getFloat("alpha", DEF_ALPHA);
  pwmFreq     = prefs.getInt("freq", 7000);
  maxPWM      = prefs.getInt("maxpwm", 255);
  minPWM_CW   = prefs.getInt("minCW", 40);
  minPWM_CCW  = prefs.getInt("minCCW", 40);
  pwmSlew     = prefs.getInt("slew", DEF_SLEW);
  encDir      = prefs.getInt("dir", 1);
  spikeLimit  = prefs.getInt("spike", 1600);
  stallEnable = prefs.getBool("stEn", false);
  stallMs     = prefs.getInt("stMs", 2000);
}

void saveSettingsToNVS() {
  prefs.putFloat("kp", Kp); prefs.putFloat("ki", Ki); prefs.putFloat("kd", Kd);
  prefs.putFloat("kff", Kff); prefs.putFloat("vKp", velKp); prefs.putFloat("vKi", velKi);
  prefs.putFloat("tol", tolerance); prefs.putFloat("maxDeg", maxDeg);
  prefs.putFloat("ratio", gearRatio); prefs.putFloat("vel", maxVelocity);
  prefs.putFloat("acc", maxAccel); prefs.putFloat("alpha", velAlpha);
  prefs.putInt("freq", pwmFreq); prefs.putInt("maxpwm", maxPWM);
  prefs.putInt("minCW", minPWM_CW); prefs.putInt("minCCW", minPWM_CCW);
  prefs.putInt("slew", pwmSlew); prefs.putInt("dir", encDir);
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
<title>SmartServo V5.3 (56:1 Fast)</title>
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
  .row label{color:var(--mut);min-width:140px;}
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
  .bar-bg{width:100%;height:14px;background:#090e18;border-radius:7px;overflow:hidden;border:1px solid var(--bdr);margin:6px 0;}
  .bar-fill{height:100%;background:linear-gradient(90deg,#38bdf8,#22c55e);width:0%;transition:width .2s;}
  .calbox{background:#090e18;border:1px solid var(--bdr);padding:10px;border-radius:8px;font-family:monospace;font-size:.82rem;color:#38bdf8;}
</style>
</head>
<body>
<div class="wrap">
  <div class="top">
    <div>
      <h2>SmartServo V5.3 (56:1 Fast @ 400kHz)</h2>
      <span id="encBadge" class="badge">I2C: OK</span>
      <span id="hzBadge" class="badge">Loop: -- Hz</span>
      <span id="lockBadge" class="badge">LOCKED (0 PWM)</span>
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
    <button class="ntab" onclick="showTab(1,this)">PID & Auto-Tune</button>
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
          <input type="range" id="sSpd" min="-250" max="250" step="5" value="0" oninput="nSpd.value=this.value;cmd({tVel:this.value})">
          <input type="number" id="nSpd" value="0" step="5" onchange="sSpd.value=this.value;cmd({tVel:this.value})">
        </div>
        <div class="row" style="justify-content:center;">
          <button class="btn" onclick="setSpd(-120)">-120°/s</button>
          <button class="btn" onclick="setSpd(-45)">-45°/s</button>
          <button class="btn pri" onclick="setSpd(0)">STOP</button>
          <button class="btn" onclick="setSpd(45)">+45°/s</button>
          <button class="btn" onclick="setSpd(120)">+120°/s</button>
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
        <input type="range" id="sVelLim" min="20" max="600" step="10" value="360" oninput="nVelLim.value=this.value;cmd({vel:this.value})">
        <input type="number" id="nVelLim" value="360" step="10" onchange="sVelLim.value=this.value;cmd({vel:this.value})">
      </div>
      <div class="row">
        <label>Max PWM (Torque)</label>
        <input type="range" id="sMaxPwm" min="60" max="255" step="5" value="255" oninput="nMaxPwm.value=this.value;cmd({maxpwm:this.value})">
        <input type="number" id="nMaxPwm" value="255" step="5" onchange="sMaxPwm.value=this.value;cmd({maxpwm:this.value})">
      </div>
    </div>
  </div>

  <!-- TAB 1: PID & AUTO-TUNE -->
  <div class="panel" id="tab1">
    <div class="card">
      <div class="lbl" style="margin-bottom:6px;">7-Stage Fast Auto-PID (~30s: Friction + Backlash + Kff + Crisp Steps + Speed + 0-PWM Lock)</div>
      <div class="calbox" id="pidMsg">Ready</div>
      <div class="bar-bg"><div class="bar-fill" id="tuneBar"></div></div>
      <div style="display:flex;gap:8px;margin:10px 0 14px;">
        <button class="btn warn" style="flex:2;padding:11px;" onclick="runAutoPid()">START 30s FAST AUTO-PID</button>
        <button class="btn err" style="flex:1;padding:11px;" onclick="resetPidDef()">RESET PID DEFAULT</button>
      </div>

      <div class="lbl" style="margin-bottom:8px;">Angle Lock PID + Feedforward</div>
      <div class="row"><label>Pos Kp (Stiffness)</label><input type="number" id="inKp" step="0.2"></div>
      <div class="row"><label>Pos Ki (Steady Lock)</label><input type="number" id="inKi" step="0.02"></div>
      <div class="row"><label>Pos Kd (Vel Damping)</label><input type="number" id="inKd" step="0.02"></div>
      <div class="row"><label>Feedforward Kff</label><input type="number" id="inKff" step="0.05"></div>
      <div class="row"><label>Accel Limit (°/s²)</label><input type="number" id="inAcc" step="50" min="50" max="2000"></div>
      <div class="row"><label>PWM Slew (Response)</label><input type="number" id="inSlew" min="4" max="60" step="2"></div>
      <div class="row"><label>Deadzone Tol (°)</label><input type="number" id="inTol" step="0.05"></div>

      <div class="lbl" style="margin:12px 0 8px;">Speed Mode PID & Filtering</div>
      <div class="row"><label>Speed Kp</label><input type="number" id="inVKp" step="0.1"></div>
      <div class="row"><label>Speed Ki</label><input type="number" id="inVKi" step="0.1"></div>
      <div class="row"><label>Vel Filter Alpha</label><input type="number" id="inAlpha" step="0.02" min="0.05" max="1"></div>
      <button class="btn ok" style="width:100%;margin-top:8px;padding:11px;" onclick="saveAll()">APPLY & SAVE TO FLASH</button>
    </div>
  </div>

  <!-- TAB 2: CALIBRATION -->
  <div class="panel" id="tab2">
    <div class="card">
      <div class="lbl" style="margin-bottom:8px;">Auto Calibration (CW/CCW H-Bridge + Polarity + I2C EMI Check)</div>
      <div class="calbox" id="calMsg">Ready</div>
      <button class="btn warn" style="width:100%;margin:10px 0;padding:11px;" onclick="runAutoCal()">RUN AUTO CALIBRATION (SPINS CW & CCW)</button>

      <div class="lbl" style="margin-top:10px;">Live AS5600 I2C (0x0E @ 400kHz)</div>
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
      <div class="lbl" style="margin-bottom:8px;">Gearbox (Default 56:1), Friction & PWM</div>
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
let curMode=1, enabled=true, encDir=1, wasTuning=false;
const histT=[], histA=[], MAX_PTS=140;
const ctx=document.getElementById('chart').getContext('2d');

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
  if(confirm('Motor will spin CW then CCW for ~2 seconds to check both H-bridge directions, polarity & I2C EMI. Continue?')){
    cmd({cal:1});
  }
}
function runAutoPid(){
  if(confirm('Starts a 30-second 7-stage fast calibration (friction, backlash, Kff/accel, micro-steps, fast ±60° reversals, speed PID & 0-PWM lock). Ensure output shaft can move ±70° freely. Continue?')){
    cmd({autotune:1});
  }
}
function resetPidDef(){
  if(confirm('Reset PID, Feedforward, Accel & Slew gains to fast 56:1 defaults?')){
    fetch('/resetpid').then(()=>loadParams());
  }
}
function factoryReset(){
  if(confirm('Reset ALL settings (including 56:1 ratio & calibration) to defaults?')) fetch('/factory').then(()=>location.reload());
}
function saveAll(){
  const q=new URLSearchParams({
    p:inKp.value, i:inKi.value, d:inKd.value, ff:inKff.value, acc:inAcc.value,
    vP:inVKp.value, vI:inVKi.value, tol:inTol.value, slew:inSlew.value,
    max:inMaxDeg.value, r:inRatio.value, vel:nVelLim.value,
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
    inKp.value=d.p; inKi.value=d.i; inKd.value=d.d; inKff.value=d.ff; inAcc.value=d.acc;
    inVKp.value=d.vP; inVKi.value=d.vI; inTol.value=d.tol; inSlew.value=d.slew;
    inMaxDeg.value=d.max; inRatio.value=d.r;
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
    if(wasTuning && !d.tuning) loadParams();
    wasTuning = d.tuning;

    vPos.textContent=d.pos.toFixed(1)+'°';
    vTgt.textContent=(d.mode===2)?(d.tVel.toFixed(1)+'°/s'):(d.tgt.toFixed(1)+'°');
    vVel.textContent=d.vel.toFixed(1);
    vPwmRaw.textContent=d.pwm+' / '+d.raw;
    vTurns.textContent='Motor Turns: '+d.turns.toFixed(1);
    hzBadge.textContent='Loop: '+d.hz+' Hz';
    stallAlert.style.display=d.stall?'flex':'none';

    encBadge.textContent=d.ok?('I2C: OK (Rec '+d.rec+')'):'I2C FAULT!';
    encBadge.style.borderColor=d.ok?'#22c55e':'#ef4444';

    if(d.mode===1){
      lockBadge.textContent=d.locked?'LOCKED (0 PWM)':'MOVING / HOLDING';
      lockBadge.style.borderColor=d.locked?'#22c55e':'#38bdf8';
    } else {
      lockBadge.textContent=(d.mode===0)?'IDLE':(d.mode===2?'SPEED PID':'MANUAL PWM');
      lockBadge.style.borderColor='#8b9bb4';
    }

    cRaw.textContent=d.raw;
    cRec.textContent=d.rec;
    cBar.style.width=((d.raw/4095)*100).toFixed(1)+'%';
    tuneBar.style.width=d.prog+'%';
    calMsg.textContent=d.calMsg;
    pidMsg.textContent=d.pidMsg;

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

  // 400 kHz Fast Mode I2C (0x0E only, clean STOP)
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(5);

  loadSettings();
  setupPWM(pwmFreq);

  if (readAS5600Raw(lastRaw)) {
    totalTicks = lastRaw;
    liveRaw = lastRaw;
  }
  updateEncoder(0.01f);
  targetDeg = currentDeg;
  resetControllers(true);

  // Wi-Fi AP + STA
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

  // Start ~1 kHz Control Loop on Core 1
  xTaskCreatePinnedToCore(controlLoopTask, "ServoV53_56", 6144, NULL, 1, &controlTaskHandle, 1);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send_P(200, "text/html", index_html); });

  server.on("/cmd", HTTP_GET, [](AsyncWebServerRequest *r){
    if (r->hasParam("mode"))       { g_mode = (uint8_t)r->getParam("mode")->value().toInt(); resetControllers(g_mode == MODE_ANGLE); }
    if (r->hasParam("tDeg"))       { targetDeg = r->getParam("tDeg")->value().toFloat(); isPositionLocked = false; settleStartMs = 0; }
    if (r->hasParam("tVel"))       targetVel = r->getParam("tVel")->value().toFloat();
    if (r->hasParam("mPwm"))       manualPWM = constrain(r->getParam("mPwm")->value().toInt(), -255, 255);
    if (r->hasParam("vel"))        maxVelocity = constrain(r->getParam("vel")->value().toFloat(), 10.0f, 1000.0f);
    if (r->hasParam("maxpwm"))     maxPWM = constrain(r->getParam("maxpwm")->value().toInt(), 60, 255);
    if (r->hasParam("en"))         { servoEnabled = (r->getParam("en")->value().toInt() == 1); if (servoEnabled) resetControllers(true); }
    if (r->hasParam("dir"))        { encDir = (r->getParam("dir")->value().toInt() >= 0) ? 1 : -1; resetControllers(true); }
    if (r->hasParam("resetstall")) resetControllers(true);
    if (r->hasParam("cal"))        reqAutoCal = true;
    if (r->hasParam("autotune"))   reqAutoPid = true;
    if (r->hasParam("recI2C"))     reqI2CRecover = true;
    if (r->hasParam("zero")) {
      homeOffset = totalTicks;
      targetDeg = 0; currentDeg = 0; prevDeg = 0;
      resetControllers(true);
      prefs.putLong("offset", homeOffset);
    }
    r->send(200, "text/plain", "OK");
  });

  server.on("/resetpid", HTTP_GET, [](AsyncWebServerRequest *r){
    resetPidDefaults();
    r->send(200, "text/plain", "OK");
  });

  server.on("/save", HTTP_GET, [](AsyncWebServerRequest *r){
    if (r->hasParam("p"))       Kp          = r->getParam("p")->value().toFloat();
    if (r->hasParam("i"))       Ki          = r->getParam("i")->value().toFloat();
    if (r->hasParam("d"))       Kd          = r->getParam("d")->value().toFloat();
    if (r->hasParam("ff"))      Kff         = r->getParam("ff")->value().toFloat();
    if (r->hasParam("acc"))     maxAccel    = constrain(r->getParam("acc")->value().toFloat(), 50.0f, 2000.0f);
    if (r->hasParam("vP"))      velKp       = r->getParam("vP")->value().toFloat();
    if (r->hasParam("vI"))      velKi       = r->getParam("vI")->value().toFloat();
    if (r->hasParam("tol"))     tolerance   = r->getParam("tol")->value().toFloat();
    if (r->hasParam("slew"))    pwmSlew     = constrain(r->getParam("slew")->value().toInt(), 4, 60);
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
    resetControllers(true);
    r->send(200, "text/plain", "OK");
  });

  server.on("/factory", HTTP_GET, [](AsyncWebServerRequest *r){
    prefs.clear();
    loadSettings();
    setupPWM(pwmFreq);
    resetControllers(true);
    r->send(200, "text/plain", "OK");
  });

  server.on("/getparams", HTTP_GET, [](AsyncWebServerRequest *r){
    char json[512];
    snprintf(json, sizeof(json),
             "{\"p\":%.3f,\"i\":%.3f,\"d\":%.3f,\"ff\":%.3f,\"acc\":%.1f,\"vP\":%.2f,\"vI\":%.2f,"
             "\"tol\":%.2f,\"slew\":%d,\"max\":%.1f,\"r\":%.1f,\"vel\":%.1f,\"maxpwm\":%d,"
             "\"minCW\":%d,\"minCCW\":%d,\"dir\":%d,\"freq\":%d,\"spike\":%d,\"alpha\":%.2f,"
             "\"stallEn\":%s,\"stallMs\":%d}",
             Kp, Ki, Kd, Kff, maxAccel, velKp, velKi, tolerance, pwmSlew, maxDeg, gearRatio,
             maxVelocity, maxPWM, minPWM_CW, minPWM_CCW, encDir, pwmFreq, spikeLimit, velAlpha,
             stallEnable ? "true" : "false", stallMs);
    r->send(200, "application/json", json);
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *r){
    char json[640];
    snprintf(json, sizeof(json),
             "{\"pos\":%.2f,\"tgt\":%.2f,\"tVel\":%.1f,\"vel\":%.1f,\"pwm\":%d,\"raw\":%d,"
             "\"turns\":%.2f,\"mode\":%u,\"stall\":%s,\"locked\":%s,\"hz\":%d,\"ok\":%s,\"rec\":%lu,"
             "\"tuning\":%s,\"prog\":%d,\"calMsg\":\"%s\",\"pidMsg\":\"%s\"}",
             currentDeg, targetDeg, targetVel, currentVel, currentPWM, liveRaw,
             (float)(totalTicks - homeOffset) / 4096.0f, g_mode,
             isStalled ? "true" : "false", isPositionLocked ? "true" : "false",
             loopHz, encHealthy ? "true" : "false", (unsigned long)i2cRecoveries,
             isAutoTuning ? "true" : "false", tuneProgress, calStatusMsg, pidStatusMsg);
    r->send(200, "application/json", json);
  });

  server.begin();
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

/*
  Robot Car Heading Control with MPU6050 (Arduino Uno)
  ----------------------------------------------------
  Features:
  - Calibrates gyro Z bias at startup.
  - Estimates yaw (heading) by integrating gyro Z rate.
  - Drives straight using heading-hold (PD controller).
  - Performs precise in-place turns to a target angle (PD controller).
  --------------------------------------------------------------------
  This code is for PD tuning only
  --------------------------------------------------------------------

  Wiring:
  MPU6050:
    SDA -> A4
    SCL -> A5
    VCC -> 5V (or 3.3V module dependent)
    GND -> GND

  H-Bridge:
    ENA -> 10 (PWM)
    IN1 -> 9
    IN2 -> 8
    IN3 -> 7
    IN4 -> 6
    ENB -> 5  (PWM)

  Notes:
  - This example uses relative yaw (no magnetometer).
  - Tune the constants in the "TUNING" section for your robot.

  Quick tuning guide:
  - KP_STRAIGHT: Higher = stronger heading correction in straight driving (too high can oscillate).
  - BASE_SPEED_STRAIGHT: Base forward speed (too low may stall, too high may reduce straight accuracy).
  - KP_TURN: Higher = turns reach target angle faster (too high can overshoot).
  - KD_TURN: Higher = more turn damping (reduces overshoot, too high can feel sluggish).
  - TURN_TOL_DEG: Smaller = tighter final angle accuracy, but may take longer to settle.
  - TURN_RATE_TOL_DPS: Smaller = robot must rotate slower before stop, improving final precision.
*/

#include <Wire.h>

// ============================
// MPU6050 register addresses
// ============================
const uint8_t MPU_ADDR       = 0x68;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_GYRO_XOUT_H = 0x43;

// ============================
// Motor pin mapping
// ============================
const int ENA = 10;
const int IN1 = 9;
const int IN2 = 8;
const int IN3 = 7;
const int IN4 = 6;
const int ENB = 5;

// Motor direction polarity:
// Set true if that side moves backward when commanded forward.
const bool INVERT_LEFT_MOTOR  = true;
const bool INVERT_RIGHT_MOTOR = true;

// ============================
// TUNING
// ============================
// Gyro configuration assumption: +/-250 deg/s => 131 LSB/(deg/s)
const float GYRO_SENS_250DPS = 131.0f;

// Control loop timing
const unsigned long LOOP_DT_MS = 10; // 100 Hz control loop

// Straight driving controller (PD)
const float KP_STRAIGHT = 2.2f;      // proportional correction per degree heading error (OG=2.2)
const float KD_STRAIGHT = 8.0f;      // damping using gyro Z rate (deg/s)  (OG=0.8)
const int   BASE_SPEED_STRAIGHT = 190; // nominal PWM while driving straight (OG=135)

// Turn controller (PD)
const float KP_TURN = 5.2f;          // proportional gain on angle error (OG=3.2)
const float KD_TURN = 0.35f;          // damping on gyro rate (deg/s) (OG=0.9)
const int   TURN_MIN_PWM = 125;       // minimum pwm to overcome stiction
const int   TURN_MAX_PWM = 190;      // max pwm while turning
const float TURN_TOL_DEG = 1.5f;     // stop tolerance in degrees
const float TURN_RATE_TOL_DPS = 4.0f;// and near-zero rate for clean stop 

// Optional small dead-zone for straight correction (helps reduce jitter)
const float STRAIGHT_ERR_DEADBAND_DEG = 0.4f;

// Before first movement, keep robot still and settle heading estimate.
const unsigned long FIRST_MOVE_HEADING_SETTLE_MS = 300;

// ============================
// Global state
// ============================
float gyroZBiasRaw = 0.0f;       // raw LSB bias for gyro Z
float yawDeg = 0.0f;             // integrated heading in degrees
float gyroZRateDps = 0.0f;       // latest angular rate (deg/s)

unsigned long lastLoopMs = 0;

// Simple mode state machine for demonstration
enum Mode {
  MODE_IDLE,
  MODE_DRIVE_STRAIGHT,
  MODE_TURNING
};

Mode mode = MODE_IDLE;

// Straight mode target heading
float targetYawStraight = 0.0f;

// Turn mode target heading
float targetYawTurn = 0.0f;

// Demo sequence timing
unsigned long modeStartMs = 0;

// Last motor commands actually applied (signed: + forward, - reverse after INVERT_*).
int lastLeftPwmCmd = 0;
int lastRightPwmCmd = 0;

// ============================
// Utility
// ============================
int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float wrapAngle180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// ============================
// MPU6050 low-level helpers
// ============================
void mpuWriteByte(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

bool mpuReadGyroRaw(int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_GYRO_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;

  // Read 6 bytes: GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L
  uint8_t toRead = 6;
  uint8_t got = Wire.requestFrom((int)MPU_ADDR, (int)toRead, (int)true);
  if (got != toRead) return false;

  gx = (int16_t)((Wire.read() << 8) | Wire.read());
  gy = (int16_t)((Wire.read() << 8) | Wire.read());
  gz = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

bool initMPU6050() {
  Wire.begin();
  // Wake sensor (clear sleep bit)
  mpuWriteByte(REG_PWR_MGMT_1, 0x00);
  delay(100);
  return true;
}

void calibrateGyroZBias(uint16_t samples = 1000) {
  long sum = 0;
  uint16_t valid = 0;

  Serial.println(F("Calibrating gyro Z bias... Keep robot still."));
  delay(500);

  for (uint16_t i = 0; i < samples; i++) {
    int16_t gx, gy, gz;
    if (mpuReadGyroRaw(gx, gy, gz)) {
      sum += gz;
      valid++;
    }
    delay(2); // ~500 Hz sampling during calibration
  }

  if (valid > 0) {
    gyroZBiasRaw = (float)sum / (float)valid;
  } else {
    gyroZBiasRaw = 0.0f;
  }

  Serial.print(F("Gyro Z bias (raw): "));
  Serial.println(gyroZBiasRaw, 3);
}

// ============================
// Motor control
// ============================
void setMotorLeft(int pwm) {
  // pwm > 0: forward, pwm < 0: reverse
  if (INVERT_LEFT_MOTOR) pwm = -pwm;
  lastLeftPwmCmd = pwm;
  int mag = clampInt(abs(pwm), 0, 255);

  if (pwm >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, mag);
}

void setMotorRight(int pwm) {
  if (INVERT_RIGHT_MOTOR) pwm = -pwm;
  lastRightPwmCmd = pwm;
  int mag = clampInt(abs(pwm), 0, 255);

  if (pwm >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  analogWrite(ENB, mag);
}

void stopMotors() {
  lastLeftPwmCmd = 0;
  lastRightPwmCmd = 0;
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  // Coast (both low)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ============================
// IMU update
// ============================
bool updateYaw(float dtSec) {
  int16_t gx, gy, gz;
  if (!mpuReadGyroRaw(gx, gy, gz)) return false;

  float gzCorrectedRaw = (float)gz - gyroZBiasRaw;
  // Inverted so clockwise physical rotation reports positive yaw.
  gyroZRateDps = -(gzCorrectedRaw / GYRO_SENS_250DPS); // deg/s

  // Integrate yaw
  yawDeg += gyroZRateDps * dtSec;
  yawDeg = wrapAngle180(yawDeg);

  return true;
}

// ============================
// Controllers
// ============================
void controlDriveStraight() {
  float err = wrapAngle180(targetYawStraight - yawDeg);

  // Optional deadband to reduce small oscillations
  if (abs(err) < STRAIGHT_ERR_DEADBAND_DEG) err = 0.0f;

  // PD correction:
  // - P term pulls heading back toward target.
  // - D term damps rotation using gyro Z rate (reduces oscillation/weaving).
  float correctionF = (KP_STRAIGHT * err) - (KD_STRAIGHT * gyroZRateDps);
  int correction = (int)correctionF;

  int leftPWM  = BASE_SPEED_STRAIGHT + correction;
  int rightPWM = BASE_SPEED_STRAIGHT - correction;

  leftPWM  = clampInt(leftPWM,  0, 255);
  rightPWM = clampInt(rightPWM, 0, 255);

  setMotorLeft(leftPWM);
  setMotorRight(rightPWM);
}

bool controlTurnToAngle() {
  float err = wrapAngle180(targetYawTurn - yawDeg);

  // PD turn command:
  // +P pushes toward target angle, -D damps using actual angular rate.
  float cmd = (KP_TURN * err) - (KD_TURN * gyroZRateDps);

  // Clamp and enforce minimum effort when not near zero.
  int pwm = (int)cmd;
  if (pwm > 0) {
    pwm = clampInt(pwm, TURN_MIN_PWM, TURN_MAX_PWM);
  } else if (pwm < 0) {
    pwm = -clampInt(-pwm, TURN_MIN_PWM, TURN_MAX_PWM);
  }

  // For in-place turn:
  // positive pwm => turn one direction, negative => opposite.
  setMotorLeft(pwm);
  setMotorRight(-pwm);

  // Stop condition: small angle error and low angular rate.
  bool angleOk = abs(err) <= TURN_TOL_DEG;
  bool rateOk  = abs(gyroZRateDps) <= TURN_RATE_TOL_DPS;
  if (angleOk && rateOk) {
    stopMotors();
    return true;
  }
  return false;
}

// ============================
// Demo sequence
// ============================
// This demonstrates:
// 1) Drive straight for 3s
// 2) Turn +90 degrees
// 3) Drive straight for 3s
// 4) Idle
void startDriveStraight(unsigned long nowMs) {
  targetYawStraight = yawDeg; // hold current heading
  mode = MODE_DRIVE_STRAIGHT;
  modeStartMs = nowMs;
  Serial.print(F("Mode: DRIVE_STRAIGHT, targetYaw="));
  Serial.println(targetYawStraight, 2);
}

void startTurnRelative(float deltaDeg, unsigned long nowMs) {
  targetYawTurn = wrapAngle180(yawDeg + deltaDeg);
  mode = MODE_TURNING;
  modeStartMs = nowMs;
  Serial.print(F("Mode: TURNING, targetYaw="));
  Serial.println(targetYawTurn, 2);
}

void settleAndZeroHeadingBeforeFirstMove() {
  stopMotors();

  // Let gyro updates run while stationary so heading settles before first drive.
  unsigned long settleStartMs = millis();
  unsigned long settleLastMs = settleStartMs;
  while (millis() - settleStartMs < FIRST_MOVE_HEADING_SETTLE_MS) {
    unsigned long now = millis();
    if (now - settleLastMs >= LOOP_DT_MS) {
      float dt = (now - settleLastMs) / 1000.0f;
      settleLastMs = now;
      updateYaw(dt);
    }
  }

  // Re-zero just before movement starts.
  yawDeg = 0.0f;
  gyroZRateDps = 0.0f;
}

void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  stopMotors();

  if (!initMPU6050()) {
    Serial.println(F("MPU6050 init failed."));
    while (1) {}
  }

  calibrateGyroZBias(1000);

  yawDeg = 0.0f;
  gyroZRateDps = 0.0f;
  lastLoopMs = millis();

  Serial.println(F("Starting demo in 1 second..."));
  delay(1000);

  settleAndZeroHeadingBeforeFirstMove();
  startDriveStraight(millis());
}

void loop() {
  unsigned long now = millis();
  if (now - lastLoopMs < LOOP_DT_MS) return;

  float dt = (now - lastLoopMs) / 1000.0f;
  lastLoopMs = now;

  if (!updateYaw(dt)) {
    // If read fails, safest action is to stop.
    stopMotors();
    Serial.println(F("IMU read failed, motors stopped."));
    delay(100);
    return;
  }

  // State machine
  switch (mode) {
    case MODE_DRIVE_STRAIGHT: {
      controlDriveStraight();

      // In demo: each straight segment lasts 3 seconds
      if (now - modeStartMs >= 3000) {
        // First straight segment turns +90, second straight ends demo.
        static bool didFirstTurn = false;
        if (!didFirstTurn) {
          didFirstTurn = true;
          startTurnRelative(+90.0f, now);
        } else {
          mode = MODE_IDLE;
          stopMotors();
          Serial.println(F("Mode: IDLE (demo done)"));
        }
      }
      break;
    }

    case MODE_TURNING: {
      bool done = controlTurnToAngle();
      if (done) {
        // After turn, do second straight segment
        startDriveStraight(now);
      }
      break;
    }

    case MODE_IDLE:
    default:
      stopMotors();
      break;
  }

  // Debug print at ~10 Hz
  static uint8_t decim = 0;
  decim++;
  if (decim >= 10) {
    decim = 0;
    Serial.print(F("yaw="));
    Serial.print(yawDeg, 2);
    Serial.print(F(" deg, rate="));
    Serial.print(gyroZRateDps, 2);
    Serial.print(F(" dps, mode="));
    Serial.print((int)mode);
    Serial.print(F(", Lpwm="));
    Serial.print(lastLeftPwmCmd);
    Serial.print(F(", Rpwm="));
    Serial.println(lastRightPwmCmd);
  }
}

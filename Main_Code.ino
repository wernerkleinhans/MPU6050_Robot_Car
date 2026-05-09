/*
  Robot Car Heading Control with MPU6050 (Arduino Uno)
  ----------------------------------------------------
  Features:
  - Calibrates gyro Z bias at startup.
  - Estimates yaw (heading) by integrating gyro Z rate.
  - Drives straight using heading-hold (PD controller).
  - Uses ultrasonic obstacle detection to trigger turns.
  - Performs in-place turns to random target angles (PD controller).
  - Adds a turn timeout fail-safe to prevent getting stuck.
  --------------------------------------------------------------------
  This sketch is the active obstacle-avoidance behavior version.
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

  Ultrasonic sensor:
    Echo -> A1
    Trig -> A2

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
const int US_ECHO_PIN = A1;
const int US_TRIG_PIN = A2;

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
const unsigned long TURN_MAX_DURATION_MS = 3000;

// Optional small dead-zone for straight correction (helps reduce jitter)
const float STRAIGHT_ERR_DEADBAND_DEG = 0.4f;
// Obstacle threshold: start turn when measured distance is <= this value.
const float OBSTACLE_STOP_DISTANCE_CM = 30.0f;
// pulseIn timeout for ultrasonic echo waiting.
const unsigned long ULTRASONIC_ECHO_TIMEOUT_US = 30000;
// Minimum gap between ultrasonic pings for stable readings.
const unsigned long ULTRASONIC_SAMPLE_INTERVAL_MS = 60;

// Allowed random turn angles in degrees.
const int TURN_ANGLE_CHOICES_DEG[] = {20, 40, 60, 80, 90, 100, 120, 140, 180};
const uint8_t TURN_ANGLE_CHOICES_COUNT = sizeof(TURN_ANGLE_CHOICES_DEG) / sizeof(TURN_ANGLE_CHOICES_DEG[0]);

// Before first movement, keep robot still and settle heading estimate.
const unsigned long FIRST_MOVE_HEADING_SETTLE_MS = 300;
// Estimate residual gyro Z drift (deg/s) while robot is stationary.
const unsigned long RESIDUAL_DRIFT_ESTIMATE_MS = 1500;
const float RESIDUAL_DRIFT_MAX_SAMPLE_DPS = 8.0f;

// ============================
// Global state
// ============================
float gyroZBiasRaw = 0.0f;       // raw LSB bias for gyro Z
float yawDeg = 0.0f;             // integrated heading in degrees
float gyroZRateRawDps = 0.0f;    // latest corrected raw rate before residual compensation
float gyroZRateDps = 0.0f;       // latest angular rate used by control (deg/s)
float gyroZResidualDriftDps = 0.0f; // estimated stationary residual drift rate (deg/s)

unsigned long lastLoopMs = 0;

// Runtime mode state machine
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

// Last motor commands actually applied (signed: + forward, - reverse after INVERT_*).
int lastLeftPwmCmd = 0;
int lastRightPwmCmd = 0;

float ultrasonicDistanceCm = -1.0f;
unsigned long lastUltrasonicSampleMs = 0;
// Timestamp used by turning timeout fail-safe.
unsigned long turnStartMs = 0;

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

float readUltrasonicDistanceCm() {
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);

  unsigned long echoDurationUs = pulseIn(US_ECHO_PIN, HIGH, ULTRASONIC_ECHO_TIMEOUT_US);
  if (echoDurationUs == 0) {
    // Timeout -> no echo in range, treat as clear path.
    return -1.0f;
  }
  return (echoDurationUs * 0.0343f) * 0.5f;
}

void updateUltrasonicDistance(unsigned long nowMs) {
  // Sample at a fixed interval to avoid over-triggering HC-SR04 style sensors.
  if (nowMs - lastUltrasonicSampleMs < ULTRASONIC_SAMPLE_INTERVAL_MS) return;
  lastUltrasonicSampleMs = nowMs;
  ultrasonicDistanceCm = readUltrasonicDistanceCm();
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
  gyroZRateRawDps = -(gzCorrectedRaw / GYRO_SENS_250DPS); // deg/s

  // Subtract estimated residual drift from runtime rate.
  gyroZRateDps = gyroZRateRawDps - gyroZResidualDriftDps;

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

void startDriveStraight() {
  targetYawStraight = yawDeg; // hold current heading
  mode = MODE_DRIVE_STRAIGHT;
  Serial.print(F("Mode: DRIVE_STRAIGHT, targetYaw="));
  Serial.println(targetYawStraight, 2);
}

void startTurnRelative(float deltaDeg) {
  targetYawTurn = wrapAngle180(yawDeg + deltaDeg);
  mode = MODE_TURNING;
  turnStartMs = millis();
  Serial.print(F("Mode: TURNING, targetYaw="));
  Serial.println(targetYawTurn, 2);
}

void startRandomTurn() {
  int angleIdx = random(TURN_ANGLE_CHOICES_COUNT);
  int turnSign = (random(2) == 0) ? -1 : 1; // -1 left, +1 right
  float deltaDeg = (float)(turnSign * TURN_ANGLE_CHOICES_DEG[angleIdx]);
  Serial.print(F("Obstacle detected. Random turn delta="));
  Serial.println(deltaDeg, 1);
  startTurnRelative(deltaDeg);
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

void estimateResidualGyroZDrift() {
  stopMotors();

  Serial.println(F("Estimating residual gyro Z drift... Keep robot still."));

  unsigned long startMs = millis();
  unsigned long lastMs = startMs;
  float sumRate = 0.0f;
  uint16_t validSamples = 0;

  while (millis() - startMs < RESIDUAL_DRIFT_ESTIMATE_MS) {
    unsigned long now = millis();
    if (now - lastMs >= LOOP_DT_MS) {
      lastMs = now;

      int16_t gx, gy, gz;
      if (!mpuReadGyroRaw(gx, gy, gz)) continue;

      float gzCorrectedRaw = (float)gz - gyroZBiasRaw;
      float rateDps = -(gzCorrectedRaw / GYRO_SENS_250DPS);

      // Reject samples that imply the robot is moving during drift estimate.
      if (abs(rateDps) <= RESIDUAL_DRIFT_MAX_SAMPLE_DPS) {
        sumRate += rateDps;
        validSamples++;
      }
    }
  }

  if (validSamples > 0) {
    gyroZResidualDriftDps = sumRate / (float)validSamples;
  } else {
    gyroZResidualDriftDps = 0.0f;
  }

  Serial.print(F("Residual gyro Z drift (dps): "));
  Serial.println(gyroZResidualDriftDps, 4);
}

void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);

  stopMotors();
  digitalWrite(US_TRIG_PIN, LOW);

  if (!initMPU6050()) {
    Serial.println(F("MPU6050 init failed."));
    while (1) {}
  }

  calibrateGyroZBias(1000);

  yawDeg = 0.0f;
  gyroZRateRawDps = 0.0f;
  gyroZRateDps = 0.0f;
  gyroZResidualDriftDps = 0.0f;
  lastLoopMs = millis();
  // Seed random turn selection from floating analog input noise and timer.
  randomSeed((unsigned long)analogRead(A0) + (unsigned long)micros());

  Serial.println(F("Starting obstacle-avoidance run in 1 second..."));
  delay(1000);

  settleAndZeroHeadingBeforeFirstMove();
  estimateResidualGyroZDrift();
  yawDeg = 0.0f;
  gyroZRateRawDps = 0.0f;
  gyroZRateDps = 0.0f;
  startDriveStraight();
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

  // Do not ping ultrasonic while turning: pulseIn() can block and hurt turn control timing.
  if (mode != MODE_TURNING) {
    updateUltrasonicDistance(now);
  }

  // State machine
  switch (mode) {
    case MODE_DRIVE_STRAIGHT: {
      controlDriveStraight();
      if (ultrasonicDistanceCm > 0.0f && ultrasonicDistanceCm <= OBSTACLE_STOP_DISTANCE_CM) {
        stopMotors();
        startRandomTurn();
      }
      break;
    }

    case MODE_TURNING: {
      bool done = controlTurnToAngle();
      if (done) {
        startDriveStraight();
      } else if (now - turnStartMs >= TURN_MAX_DURATION_MS) {
        stopMotors();
        Serial.println(F("Turn timeout -> resume straight"));
        startDriveStraight();
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
    Serial.print(F(" dps(raw="));
    Serial.print(gyroZRateRawDps, 2);
    Serial.print(F(", drift="));
    Serial.print(gyroZResidualDriftDps, 3);
    Serial.print(F("), mode="));
    Serial.print((int)mode);
    Serial.print(F(", Lpwm="));
    Serial.print(lastLeftPwmCmd);
    Serial.print(F(", Rpwm="));
    Serial.print(lastRightPwmCmd);
    Serial.print(F(", distCm="));
    if (ultrasonicDistanceCm > 0.0f) {
      Serial.println(ultrasonicDistanceCm, 1);
    } else {
      Serial.println(F("NA"));
    }
  }
}

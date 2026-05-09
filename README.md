# MPU6050_Robot_car

Arduino robot car project that uses an `MPU6050` gyro for heading control and an ultrasonic sensor for obstacle-triggered random turns.

## What this project does

- Keeps the car driving straight using a PD heading controller.
- Detects obstacles with an ultrasonic sensor.
- When an obstacle is within `30 cm`, the car performs an in-place random turn:
  - random direction: left or right
  - random angle from: `20, 40, 60, 80, 90, 100, 120, 140, 180`
- Returns to straight driving after each turn.

## Important workflow (do this first)

You should **always tune your own robot first** before running the main behavior.

1. Open and upload `PID_Tuning_code.ino`
2. Tune the PD constants for your specific robot
3. Copy the tuned values into `Main_Code.ino`
4. Upload `Main_Code.ino`

This is required because every robot differs in motor strength, wheel friction, battery voltage, and chassis balance.

## Sketch overview

### `PID_Tuning_code.ino`

Used for tuning only:
- gyro-based yaw estimation
- straight PD tuning
- turn PD tuning
- simple, controlled motion sequence for repeatable tuning

### `Main_Code.ino`

Main autonomous behavior:
- same IMU/PD control core as tuning sketch
- ultrasonic obstacle detection (`Echo=A1`, `Trig=A2`)
- obstacle threshold handling (`<= 30 cm`)
- random turn selection from predefined angle list
- turn timeout fail-safe to avoid getting stuck in a turn

## Wiring

### MPU6050 (I2C)

- `SDA -> A4`
- `SCL -> A5`
- `VCC -> 5V` (or module-specific supply requirement)
- `GND -> GND`

### Motor driver (H-bridge)

- `ENA -> 10` (PWM)
- `IN1 -> 9`
- `IN2 -> 8`
- `IN3 -> 7`
- `IN4 -> 6`
- `ENB -> 5` (PWM)

### Ultrasonic sensor

- `Echo -> A1`
- `Trig -> A2`
- `VCC -> 5V`
- `GND -> GND`

## PD tuning guide

Tune in `PID_Tuning_code.ino` first.

### Straight controller

- `KP_STRAIGHT`
  - Increase: stronger correction back to heading
  - Too high: oscillation/weaving
- `KD_STRAIGHT`
  - Increase: more damping, smoother corrections
  - Too high: sluggish response
- `BASE_SPEED_STRAIGHT`
  - Too low: stalls / poor response
  - Too high: harder to keep straight
- `STRAIGHT_ERR_DEADBAND_DEG`
  - Small dead zone to reduce jitter around zero error

### Turn controller

- `KP_TURN`
  - Increase: faster angle convergence
  - Too high: overshoot
- `KD_TURN`
  - Increase: damping and stability
  - Too high: slow turn completion
- `TURN_MIN_PWM`
  - Must overcome motor stiction
- `TURN_MAX_PWM`
  - Limits aggressive turn speed
- `TURN_TOL_DEG`, `TURN_RATE_TOL_DPS`
  - Final stop quality (angle + low angular rate)

## How control works (high level)

1. Startup:
   - initialize MPU6050
   - calibrate gyro Z bias while stationary
   - settle and zero heading before first movement
   - estimate residual gyro drift (main code)
2. Runtime loop:
   - update yaw from gyro Z integration
   - in straight mode: run heading-hold PD and sample ultrasonic
   - if obstacle is close: stop and start random in-place turn
   - in turning mode: run turn PD until settled, then return to straight
3. Safety:
   - stop motors on IMU read failure
   - turn timeout fail-safe resumes straight mode if a turn takes too long

## Serial monitor output

Main sketch prints telemetry (about 10 Hz), including:
- `yaw`
- gyro rate (`rate`, and raw/drift values)
- mode
- signed motor commands (`Lpwm`, `Rpwm`)
- ultrasonic distance (`distCm`, or `NA` if no echo)

## Power and reliability notes

- Use a stable power source for Arduino, motors, IMU, and ultrasonic.
- Make sure all modules share a **common ground**.
- Unstable power can cause turning issues, sensor dropouts, and inconsistent behavior.
- Avoid reading ultrasonic continuously at very high rate; the main code already samples with a fixed interval.

## Recommended setup checklist

- Robot lifted off ground for first flash test
- Motor directions verified (use invert flags if needed)
- IMU calibration done while robot is completely still
- PD tuned in `PID_Tuning_code.ino`
- Tuned values copied into `Main_Code.ino`
- Final test on floor with stable battery and shared ground

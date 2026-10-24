#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// =====================================================
// HARDWARE
// =====================================================

const int ENA = 5;
const int IN1 = 4;
const int IN2 = 7;

const int IN3 = 8;
const int IN4 = 9;
const int ENB = 6;

const int TRIG = A0;
const int ECHO = A1;

const int BRAKE_LED = 2;
const int BUZZER = 3;

LiquidCrystal_I2C lcd(0x27, 16, 2);

// =====================================================
// MAVP ACC DEMO  (Arduino only: no Raspberry Pi, no camera, no track needed)
//   cruise -> car ahead -> BRAKING (one beep) -> car ahead backs toward us -> we BACK OFF STRAIGHT
//   (gyro holds the heading, repeating beeps) -> car ahead drives away -> we FOLLOW forward.
// This is the original stage6_high_speed_follow logic. Added: gyro straight-hold (forward and reverse),
// gyro start-up with retries (the demo still runs without the gyro), cruise time limit, back-off time limit.
// =====================================================

// Gyro (MPU6050 on the I2C bus next to the LCD). Z axis = turning. Positive angle = nose turned RIGHT.
class DemoGyro {
public:
  bool begin() {
    ok = false;
    fault = 1;
    Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(4000, true);
#endif
    address = present(0x68) ? 0x68 : present(0x69) ? 0x69 : 0;
    if (!address) return false;
    byte identity;
    if (!readRegister(0x75, identity) || identity != 0x68) return false;
    if (!writeRegister(0x6B, 0x80)) return false;
    delay(100);
    if (!writeRegister(0x6B, 0x01) || !writeRegister(0x6C, 0x00) ||
        !writeRegister(0x1B, 0x08) || !writeRegister(0x1C, 0x00) ||
        !writeRegister(0x1A, 0x03) || !writeRegister(0x19, 0x04)) return false;
    delay(500);
    long sum = 0;
    long accelSums[3] = {0, 0, 0};
    int16_t minimum = 32767, maximum = -32767;
    for (unsigned int sample = 0; sample < 200; sample++) {
      int16_t accel[3], gz;
      if (!readMotion(accel, gz)) return false;
      for (byte i = 0; i < 3; i++) accelSums[i] += accel[i];
      sum += gz;
      if (gz < minimum) minimum = gz;
      if (gz > maximum) maximum = gz;
      delay(5);
    }
    float mean = sum / 200.0f;
    // The car moved while calibrating: try again.
    if ((long)maximum - minimum > 197L || mean < -655 || mean > 655) return false;
    float ax = accelSums[0] / (200.0f * 16384.0f);
    float ay = accelSums[1] / (200.0f * 16384.0f);
    float az = accelSums[2] / (200.0f * 16384.0f);
    if (ax < -0.35f || ax > 0.35f || ay < -0.35f || ay > 0.35f || az < 0.70f || az > 1.30f) { fault = 5; return false; }
    zBias = mean;
    degreesRight = 0.0f;
    lastUs = micros();
    ok = true;
    fault = 0;
    return true;
  }

  void update() {
    if (!ok) return;
    unsigned long now = micros();
    unsigned long elapsed = now - lastUs;
    if (elapsed < 10000UL) return;
    int16_t accel[3], gz;
    if (elapsed > 200000UL) fault = 3;                 // loop stalled: the angle can no longer be trusted
    else if (!readMotion(accel, gz)) fault = 2;        // I2C error
    else if (gz > 32000 || gz < -32000) fault = 4;     // sensor saturated
    if (fault != 0) { ok = false; return; }
    degreesRight += -(gz - zBias) / 65.5f * (elapsed / 1000000.0f);
    lastUs = now;
  }

  bool healthy() const { return ok; }
  byte faultCode() const { return fault; }
  float angle() const { return degreesRight; }

private:
  byte address = 0;
  bool ok = false;
  byte fault = 1;
  float zBias = 0.0f;
  float degreesRight = 0.0f;
  unsigned long lastUs = 0;

  bool present(byte candidate) {
    Wire.beginTransmission(candidate);
    return Wire.endTransmission() == 0;
  }
  bool writeRegister(byte reg, byte value) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
  }
  bool readRegister(byte reg, byte &value) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom(address, (byte)1) != 1) return false;
    value = Wire.read();
    return true;
  }
  bool readMotion(int16_t accel[3], int16_t &gyroZ) {
    Wire.beginTransmission(address);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom(address, (byte)14) != 14) return false;
    for (byte i = 0; i < 3; i++) { uint16_t h = (uint8_t)Wire.read(); uint16_t l = (uint8_t)Wire.read(); accel[i] = (int16_t)((h << 8) | l); }
    Wire.read(); Wire.read();                          // temperature
    Wire.read(); Wire.read();                          // gyro X
    Wire.read(); Wire.read();                          // gyro Y
    uint16_t h = (uint8_t)Wire.read(); uint16_t l = (uint8_t)Wire.read();
    gyroZ = (int16_t)((h << 8) | l);
    return true;
  }
};

DemoGyro gyro;

// =====================================================
// DISTANCE CONTROL
// =====================================================

const int DETECTION_LIMIT_CM = 45;
const int RECHECK_DISTANCE_CM = 20;
const int BRAKE_DISTANCE_CM = 10;
const int FORWARD_RESTART_CM = 18;
const int REVERSE_RELEASE_CM = 24;

const int MOTION_THRESHOLD_CM = 2;
const int URGENT_CLOSING_SPEED_CM_S = 20;

const unsigned long SENSOR_PERIOD_MS = 50;
const unsigned long MOTION_WINDOW_MS = 180;

// Reverse must run briefly before direction is checked again.
const unsigned long MIN_REVERSE_RUN_MS = 500;

// =====================================================
// PWM CONTROL
// =====================================================

const int FORWARD_START_PWM = 245;
const int REVERSE_START_PWM = 220;

const int CRUISE_PWM = 215;

const int FOLLOW_INITIAL_PWM = 215;
const int FORWARD_MIN_PWM = 210;
const int FORWARD_MAX_PWM = 225;

const int REVERSE_MIN_PWM = 170;
const int REVERSE_MAX_PWM = 190;

const unsigned long FORWARD_KICK_MS = 260;
const unsigned long REVERSE_KICK_MS = 220;

const unsigned long FORWARD_TO_REVERSE_WAIT_MS = 450;
const unsigned long REVERSE_TO_FORWARD_WAIT_MS = 300;

const unsigned long PWM_RAMP_PERIOD_MS = 20;

const int RUN_RAMP_STEP = 3;
const int STOP_RAMP_STEP = 15;

// =====================================================
// SENSOR STATE
// =====================================================

long sensorSamples[3] = {0, 0, 0};

byte sensorSampleIndex = 0;
byte sensorSampleCount = 0;
byte noEchoCount = 0;

long measuredDistance = 999;
long filteredDistance = -1;
long lastRawDistance = 999;

unsigned long lastSensorReadingAt = 0;

bool sensorReady = false;

// =====================================================
// TARGET TREND
// =====================================================

enum TargetTrend {
  TREND_UNKNOWN,
  TREND_STABLE,
  TREND_OPENING,
  TREND_CLOSING
};

TargetTrend targetTrend = TREND_UNKNOWN;

long trendReferenceDistance = 0;
long estimatedGapSpeedCmS = 0;

unsigned long trendReferenceTime = 0;
unsigned long trendVersion = 0;

bool trendTracking = false;

// =====================================================
// FOLLOW DIRECTION
// =====================================================

enum FollowIntent {
  INTENT_OBSERVE,
  INTENT_STOPPED,
  INTENT_FORWARD,
  INTENT_REVERSE
};

FollowIntent followIntent = INTENT_OBSERVE;

bool observationStarted = false;
bool targetWasDetected = false;

bool reverseRunTiming = false;
unsigned long reverseRunStartedAt = 0;

// =====================================================
// ADAPTIVE FORWARD SPEED
// =====================================================

int adaptiveForwardPwm = FOLLOW_INITIAL_PWM;
unsigned long lastAppliedTrendVersion = 0;

// =====================================================
// MOTOR STATE
// =====================================================

enum DrivePhase {
  DRIVE_STOPPED,
  DRIVE_RAMP_DOWN,
  DRIVE_DIRECTION_WAIT,
  DRIVE_START_KICK,
  DRIVE_RUNNING
};

DrivePhase drivePhase = DRIVE_STOPPED;

int requestedDirection = 0;
int requestedPwm = 0;

int activeDirection = 0;
int currentPwm = 0;
int lastStoppedDirection = 0;

unsigned long fullyStoppedAt = 0;
unsigned long drivePhaseStartedAt = 0;
unsigned long lastPwmRampAt = 0;

// =====================================================
// ALERT STATE
// =====================================================

enum AlertMode {
  ALERT_NONE,
  ALERT_BRAKE,
  ALERT_REVERSE
};

AlertMode activeAlert = ALERT_NONE;
unsigned long alertStartedAt = 0;

// =====================================================
// LCD STATE
// =====================================================

char previousTop[17] = "";
char previousBottom[17] = "";

unsigned long lastLcdUpdateAt = 0;

// =====================================================
// LCD FUNCTIONS
// =====================================================

void writeLcdLine(
  byte row,
  const char* text,
  char* previous
) {
  char padded[17];

  for (byte i = 0; i < 16; i++) {
    padded[i] = ' ';
  }

  padded[16] = '\0';

  byte i = 0;

  while (text[i] != '\0' && i < 16) {
    padded[i] = text[i];
    i++;
  }

  if (strcmp(padded, previous) == 0) {
    return;
  }

  lcd.setCursor(0, row);

  for (byte position = 0; position < 16; position++) {
    lcd.write((uint8_t)padded[position]);
  }

  strcpy(previous, padded);
}

void makeSpeedBar(
  char* output,
  const char* direction,
  int pwm
) {
  byte blocks = map(
    constrain(pwm, 0, 255),
    0,
    255,
    0,
    8
  );

  for (byte i = 0; i < 16; i++) {
    output[i] = ' ';
  }

  output[16] = '\0';

  output[0] = direction[0];
  output[1] = direction[1];
  output[2] = direction[2];

  output[3] = ' ';
  output[4] = '[';

  for (byte i = 0; i < 8; i++) {
    output[5 + i] =
      i < blocks ? char(255) : ' ';
  }

  output[13] = ']';
}

// =====================================================
// ULTRASONIC SENSOR
// =====================================================

long readRawDistanceCm() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG, LOW);

  unsigned long duration =
    pulseIn(ECHO, HIGH, 25000UL);

  if (duration == 0) {
    return 999;
  }

  return duration / 58;
}

long medianOfThree(long a, long b, long c) {
  return max(min(a, b), min(max(a, b), c));
}

bool updateDistanceSensor() {
  unsigned long now = millis();

  if (
    now - lastSensorReadingAt <
    SENSOR_PERIOD_MS
  ) {
    return false;
  }

  lastSensorReadingAt = now;

  long rawDistance = readRawDistanceCm();

  if (rawDistance >= 999) {
    if (noEchoCount < 3) {
      noEchoCount++;
    }

    if (noEchoCount >= 3) {
      measuredDistance = 999;
      filteredDistance = -1;
      lastRawDistance = 999;

      sensorSampleCount = 0;
      sensorSampleIndex = 0;

      sensorReady = true;
    }

    return true;
  }

  noEchoCount = 0;
  lastRawDistance = rawDistance;

  // Emergency reading bypasses the normal filtering.
  if (rawDistance <= BRAKE_DISTANCE_CM) {
    for (byte i = 0; i < 3; i++) {
      sensorSamples[i] = rawDistance;
    }

    sensorSampleCount = 3;
    sensorSampleIndex = 0;

    filteredDistance = rawDistance;
    measuredDistance = rawDistance;

    sensorReady = true;
    return true;
  }

  sensorSamples[sensorSampleIndex] =
    rawDistance;

  sensorSampleIndex =
    (sensorSampleIndex + 1) % 3;

  if (sensorSampleCount < 3) {
    sensorSampleCount++;
  }

  long medianDistance;

  if (sensorSampleCount == 1) {
    medianDistance = rawDistance;
  }
  else if (sensorSampleCount == 2) {
    medianDistance =
      (sensorSamples[0] + sensorSamples[1]) / 2;
  }
  else {
    medianDistance = medianOfThree(
      sensorSamples[0],
      sensorSamples[1],
      sensorSamples[2]
    );
  }

  if (filteredDistance < 0) {
    filteredDistance = medianDistance;
  }
  else if (medianDistance < filteredDistance) {
    // Faster response when an object approaches.
    filteredDistance =
      (filteredDistance + medianDistance * 3) / 4;
  }
  else {
    // Gentle smoothing when the gap opens.
    filteredDistance =
      (filteredDistance * 2 + medianDistance) / 3;
  }

  measuredDistance = filteredDistance;
  sensorReady = true;

  return true;
}

// =====================================================
// TARGET-TREND MEASUREMENT
// =====================================================

void beginTrendMeasurement(long distance) {
  trendTracking = true;

  trendReferenceDistance = distance;
  trendReferenceTime = millis();

  estimatedGapSpeedCmS = 0;
  targetTrend = TREND_UNKNOWN;
}

void resetTrendMeasurement() {
  trendTracking = false;

  estimatedGapSpeedCmS = 0;
  targetTrend = TREND_UNKNOWN;
}

void updateTargetTrend(long distance) {
  unsigned long now = millis();

  if (!trendTracking) {
    beginTrendMeasurement(distance);
    return;
  }

  unsigned long elapsed =
    now - trendReferenceTime;

  if (elapsed < MOTION_WINDOW_MS) {
    return;
  }

  long distanceChange =
    distance - trendReferenceDistance;

  estimatedGapSpeedCmS =
    (abs(distanceChange) * 1000L) / elapsed;

  if (distanceChange >= MOTION_THRESHOLD_CM) {
    // Target is moving away.
    targetTrend = TREND_OPENING;
  }
  else if (
    distanceChange <= -MOTION_THRESHOLD_CM
  ) {
    // Target is moving toward the car.
    targetTrend = TREND_CLOSING;
  }
  else {
    targetTrend = TREND_STABLE;
    estimatedGapSpeedCmS = 0;
  }

  trendReferenceDistance = distance;
  trendReferenceTime = now;

  trendVersion++;
}

// Remove old filter movement before determining
// the target's real direction.
void prepareStoppedObservation() {
  if (lastRawDistance < 999) {
    for (byte i = 0; i < 3; i++) {
      sensorSamples[i] = lastRawDistance;
    }

    sensorSampleCount = 3;
    sensorSampleIndex = 0;

    filteredDistance = lastRawDistance;
    measuredDistance = lastRawDistance;
  }

  beginTrendMeasurement(measuredDistance);
  observationStarted = true;
}

void enterObservationMode() {
  followIntent = INTENT_OBSERVE;

  observationStarted = false;
  reverseRunTiming = false;

  resetTrendMeasurement();
}

// =====================================================
// ADAPTIVE SPEED
// =====================================================

int calculateForwardFollowPwm(long distance) {
  if (lastAppliedTrendVersion != trendVersion) {
    lastAppliedTrendVersion = trendVersion;

    int adjustment = constrain(
      3 + (int)(estimatedGapSpeedCmS / 3),
      3,
      14
    );

    if (targetTrend == TREND_OPENING) {
      // Target pulling away: accelerate.
      adaptiveForwardPwm += adjustment;
    }
    else if (targetTrend == TREND_CLOSING) {
      // Car catching target: decelerate.
      adaptiveForwardPwm -= adjustment;
    }

    // A stable gap means approximately matched speed.
  }

  if (distance <= 20) {
    adaptiveForwardPwm =
      min(adaptiveForwardPwm, 215);
  }
  else if (distance >= 27) {
    adaptiveForwardPwm =
      max(adaptiveForwardPwm, 220);
  }

  adaptiveForwardPwm = constrain(
    adaptiveForwardPwm,
    FORWARD_MIN_PWM,
    FORWARD_MAX_PWM
  );

  return adaptiveForwardPwm;
}

int calculateReversePwm(long closingSpeed) {
  int pwm = map(
    constrain(closingSpeed, 0, 40),
    0,
    40,
    REVERSE_MIN_PWM,
    REVERSE_MAX_PWM
  );

  return constrain(
    pwm,
    REVERSE_MIN_PWM,
    REVERSE_MAX_PWM
  );
}

// =====================================================
// MOTOR OUTPUT
// =====================================================

void applyMotorDirection(int direction) {
  if (direction > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else if (direction < 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
}

// ---------------------------------------------------
// STRAIGHT-HOLD (gyro). Bank A (ENA) = LEFT wheels, bank B (ENB) = RIGHT wheels.
//   The heading at the start of a motion is remembered. If the nose swings RIGHT (positive error):
//   forward  -> right bank faster / left bank slower
//   reverse  -> left bank faster (in reverse that swings the nose back to the LEFT)
//   A slow bank never goes below the lowest PWM the motors need to turn; the fast bank is raised instead.
// ---------------------------------------------------
const float HOLD_KP = 8.0f;                  // PWM difference per degree of heading error
const float HOLD_KI = 25.0f;                 // PWM per degree-second: learns the weaker side
const float HOLD_MAX_FWD = 45.0f;            // largest left/right PWM difference going forward
const float HOLD_MAX_REV = 60.0f;            // ... and backing off
const float HOLD_ABORT_DEG = 12.0f;          // nose 12 degrees off the held heading: something is wrong, equal PWM from then on
const unsigned long CRUISE_MAX_MS = 20000;   // cruising this long with no car ahead: stop and wait
const unsigned long MAX_REVERSE_TOTAL_MS = 5000;   // most backing off in one go (nothing looks behind the car)

float holdYawRef = 0;
float holdTrimFwd = 0;
float holdTrimRev = 0;
unsigned long holdAt = 0;
bool reverseEpisode = false;                 // one back-off keeps ONE heading reference, also between its small steps
bool holdFault = false;
int lastLeftPwm = 0;
int lastRightPwm = 0;
unsigned long reverseUsedMs = 0;
bool cruiseTiming = false;
bool cruiseIdle = false;
unsigned long cruiseStartedAt = 0;

bool holdWanted() {
  return
    gyro.healthy() &&
    !holdFault &&
    drivePhase != DRIVE_RAMP_DOWN &&
    activeDirection != 0 &&
    requestedDirection == activeDirection;
}

// PWM difference (+ = the nose must swing LEFT).
int holdCommand() {
  unsigned long now = millis();
  float dt = min(now - holdAt, 100UL) / 1000.0f;
  holdAt = now;

  float err = gyro.angle() - holdYawRef;

  if (err > HOLD_ABORT_DEG || err < -HOLD_ABORT_DEG) {
    holdFault = true;
    Serial.println("HOLD FAULT: heading error > 12 deg, equal PWM from now on");
    return 0;
  }

  float limit = activeDirection > 0 ? HOLD_MAX_FWD : HOLD_MAX_REV;
  float &trim = activeDirection > 0 ? holdTrimFwd : holdTrimRev;

  trim = constrain(trim + HOLD_KI * err * dt, -limit, limit);

  return (int)constrain(HOLD_KP * err + trim, -limit, limit);
}

void applyMotorPwm(int pwm) {
  pwm = constrain(pwm, 0, 255);

  int leftPwm = pwm;
  int rightPwm = pwm;

  if (pwm > 0 && holdWanted()) {
    int t = holdCommand();
    int d = abs(t);
    int floorPwm = min(activeDirection > 0 ? FORWARD_MIN_PWM : REVERSE_MIN_PWM, pwm);
    int hi = pwm + (d + 1) / 2;
    int lo = pwm - d / 2;

    if (lo < floorPwm) {
      lo = floorPwm;
      hi = lo + d;
    }
    if (hi > 255) {
      hi = 255;
      lo = max(floorPwm, hi - d);
    }

    bool leftFaster = activeDirection > 0 ? (t < 0) : (t > 0);

    leftPwm = leftFaster ? hi : lo;
    rightPwm = leftFaster ? lo : hi;
  }

  analogWrite(ENA, leftPwm);
  analogWrite(ENB, rightPwm);

  lastLeftPwm = leftPwm;
  lastRightPwm = rightPwm;

  currentPwm = pwm;
}

// Re-send the (held) output while the car drives, so the heading correction is applied every cycle.
void refreshHold() {
  if (currentPwm > 0 && holdWanted()) {
    applyMotorPwm(currentPwm);
  }
}

void setDriveRequest(int direction, int pwm) {
  requestedDirection = direction;

  if (direction == 0) {
    requestedPwm = 0;
  } else {
    requestedPwm = pwm;
  }
}

bool oppositeDirectionRequested() {
  return
    lastStoppedDirection != 0 &&
    requestedDirection != 0 &&
    requestedDirection != lastStoppedDirection;
}

unsigned long requiredDirectionWaitMs() {
  if (
    lastStoppedDirection > 0 &&
    requestedDirection < 0
  ) {
    return FORWARD_TO_REVERSE_WAIT_MS;
  }

  if (
    lastStoppedDirection < 0 &&
    requestedDirection > 0
  ) {
    return REVERSE_TO_FORWARD_WAIT_MS;
  }

  return 0;
}

void beginMotorStart(int direction) {
  applyMotorDirection(direction);

  // Heading reference: taken while the car stands still. Forward: at every start. Backing off: once per back-off.
  if (direction > 0) {
    holdYawRef = gyro.angle();
    reverseEpisode = false;
  }
  else if (!reverseEpisode) {
    holdYawRef = gyro.angle();
    reverseEpisode = true;
  }
  holdAt = millis();

  activeDirection = direction;
  drivePhase = DRIVE_START_KICK;

  if (direction > 0) {
    applyMotorPwm(FORWARD_START_PWM);
  } else {
    applyMotorPwm(REVERSE_START_PWM);
  }

  drivePhaseStartedAt = millis();
  lastPwmRampAt = millis();
}

void beginRampDown() {
  drivePhase = DRIVE_RAMP_DOWN;
  lastPwmRampAt = millis();
}

void recordFullStop(unsigned long now) {
  if (activeDirection != 0) {
    lastStoppedDirection = activeDirection;
  }

  applyMotorPwm(0);

  activeDirection = 0;
  fullyStoppedAt = now;
}

void serviceMotors() {
  unsigned long now = millis();

  if (drivePhase == DRIVE_RAMP_DOWN) {
    if (
      now - lastPwmRampAt >=
      PWM_RAMP_PERIOD_MS
    ) {
      lastPwmRampAt = now;

      int nextPwm =
        max(0, currentPwm - STOP_RAMP_STEP);

      applyMotorPwm(nextPwm);

      if (nextPwm == 0) {
        recordFullStop(now);

        if (
          requestedDirection != 0 &&
          oppositeDirectionRequested()
        ) {
          drivePhase = DRIVE_DIRECTION_WAIT;
        } else {
          drivePhase = DRIVE_STOPPED;
        }
      }
    }

    return;
  }

  if (drivePhase == DRIVE_DIRECTION_WAIT) {
    applyMotorPwm(0);
    activeDirection = 0;

    if (requestedDirection == 0) {
      drivePhase = DRIVE_STOPPED;
      return;
    }

    if (!oppositeDirectionRequested()) {
      beginMotorStart(requestedDirection);
      return;
    }

    if (
      now - fullyStoppedAt >=
      requiredDirectionWaitMs()
    ) {
      beginMotorStart(requestedDirection);
    }

    return;
  }

  if (requestedDirection == 0) {
    if (currentPwm > 0) {
      beginRampDown();
    } else {
      drivePhase = DRIVE_STOPPED;
      activeDirection = 0;
    }

    return;
  }

  if (drivePhase == DRIVE_STOPPED) {
    if (
      oppositeDirectionRequested() &&
      now - fullyStoppedAt <
        requiredDirectionWaitMs()
    ) {
      drivePhase = DRIVE_DIRECTION_WAIT;
      return;
    }

    beginMotorStart(requestedDirection);
    return;
  }

  if (activeDirection != requestedDirection) {
    beginRampDown();
    return;
  }

  if (drivePhase == DRIVE_START_KICK) {
    unsigned long kickTime =
      activeDirection > 0
        ? FORWARD_KICK_MS
        : REVERSE_KICK_MS;

    if (
      now - drivePhaseStartedAt >= kickTime
    ) {
      drivePhase = DRIVE_RUNNING;
      lastPwmRampAt = now;
    }
    else {
      refreshHold();
    }

    return;
  }

  if (drivePhase == DRIVE_RUNNING) {
    if (
      now - lastPwmRampAt >=
      PWM_RAMP_PERIOD_MS
    ) {
      lastPwmRampAt = now;

      if (currentPwm < requestedPwm) {
        applyMotorPwm(
          min(
            currentPwm + RUN_RAMP_STEP,
            requestedPwm
          )
        );
      }
      else if (currentPwm > requestedPwm) {
        applyMotorPwm(
          max(
            currentPwm - RUN_RAMP_STEP,
            requestedPwm
          )
        );
      }
      else {
        refreshHold();
      }
    }
  }
}

// =====================================================
// ALERTS
// =====================================================

void updateAlerts(bool braking, bool reversing) {
  unsigned long now = millis();

  AlertMode requestedAlert = ALERT_NONE;

  if (braking) {
    requestedAlert = ALERT_BRAKE;
  }
  else if (reversing) {
    requestedAlert = ALERT_REVERSE;
  }

  bool alertChanged =
    requestedAlert != activeAlert;

  if (alertChanged) {
    activeAlert = requestedAlert;
    alertStartedAt = now;

    noTone(BUZZER);
  }

  unsigned long alertTime =
    now - alertStartedAt;

  if (activeAlert == ALERT_BRAKE) {
    digitalWrite(BRAKE_LED, HIGH);

    if (alertChanged) {
      tone(BUZZER, 800, 600);
    }
  }
  else if (activeAlert == ALERT_REVERSE) {
    bool reverseBeep =
      (alertTime % 700UL) < 200UL;

    if (reverseBeep) {
      digitalWrite(BRAKE_LED, HIGH);
      tone(BUZZER, 1150);
    } else {
      digitalWrite(BRAKE_LED, LOW);
      noTone(BUZZER);
    }
  }
  else {
    digitalWrite(BRAKE_LED, LOW);
    noTone(BUZZER);
  }
}

// =====================================================
// LCD
// =====================================================

void updateCluster(
  bool targetDetected,
  bool braking,
  bool transitionBraking,
  bool reversing
) {
  unsigned long now = millis();

  if (now - lastLcdUpdateAt < 100) {
    return;
  }

  lastLcdUpdateAt = now;

  char top[17];
  char bottom[17];

  if (!sensorReady) {
    strcpy(top, "SENSOR CHECK");
    strcpy(bottom, "PLEASE WAIT");
  }
  else if (transitionBraking) {
    strcpy(top, "!!  BRAKING  !!");

    snprintf(
      bottom,
      sizeof(bottom),
      "D:%2ldcm REV PREP",
      measuredDistance
    );
  }
  else if (braking) {
    strcpy(top, "!!  BRAKING  !!");

    snprintf(
      bottom,
      sizeof(bottom),
      "D:%2ldcm SPEED 0",
      measuredDistance
    );
  }
  else if (reversing) {
    strcpy(top, "<< REVERSING <<");

    snprintf(
      bottom,
      sizeof(bottom),
      "D:%2ldcm BACK-OFF",
      measuredDistance
    );
  }
  else if (!targetDetected && cruiseIdle) {
    strcpy(top, "NO CAR AHEAD");
    strcpy(bottom, "WAITING...");
  }
  else if (!targetDetected) {
    strcpy(top, "ACC     CRUISE");
    makeSpeedBar(bottom, "FWD", currentPwm);
  }
  else {
    snprintf(
      top,
      sizeof(top),
      "D:%2ldcm FOLLOW",
      measuredDistance
    );

    if (
      activeDirection > 0 &&
      currentPwm > 0
    ) {
      makeSpeedBar(bottom, "FWD", currentPwm);
    }
    else if (
      activeDirection < 0 &&
      currentPwm > 0
    ) {
      makeSpeedBar(bottom, "REV", currentPwm);
    }
    else {
      makeSpeedBar(bottom, "SPD", 0);
    }
  }

  writeLcdLine(0, top, previousTop);
  writeLcdLine(1, bottom, previousBottom);
}

// =====================================================
// SETUP
// =====================================================

// Calibrates the gyro (car must stand still), then counts down so you can step away.
// If the gyro cannot be started the demo still runs, only without the straight-hold.
void demoGyroStart() {
  bool ok = false;

  for (byte attempt = 0; attempt < 4 && !ok; attempt++) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("GYRO CALIBRATE");
    lcd.setCursor(0, 1);
    lcd.print("KEEP CAR STILL");

    ok = gyro.begin();

    if (!ok) {
      delay(400);
    }
  }

  lcd.clear();

  if (!ok) {
    lcd.setCursor(0, 0);
    lcd.print("GYRO FAILED");
    lcd.setCursor(0, 1);
    lcd.print("NO STRAIGHT HOLD");

    Serial.print("GYRO FAILED, code ");
    Serial.println(gyro.faultCode());

    delay(2500);
    lcd.clear();
    return;
  }

  lcd.setCursor(0, 0);
  lcd.print("GYRO OK");

  for (byte seconds = 3; seconds > 0; seconds--) {
    char text[17];
    snprintf(text, sizeof(text), "STARTS IN %d", seconds);

    lcd.setCursor(0, 1);
    lcd.print(text);

    unsigned long until = millis() + 1000UL;

    while ((long)(millis() - until) < 0) {
      gyro.update();
      delay(5);
    }
  }

  lcd.clear();
  Serial.println("GYRO OK");
}

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  pinMode(BRAKE_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  applyMotorPwm(0);

  digitalWrite(BRAKE_LED, LOW);
  noTone(BUZZER);

  Serial.begin(115200);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("     M A V P");

  lcd.setCursor(0, 1);
  lcd.print("ACC DEMO  GYRO");

  delay(1500);
  lcd.clear();

  demoGyroStart();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  unsigned long now = millis();

  gyro.update();

  bool newMeasurement =
    updateDistanceSensor();

  bool targetDetected =
    sensorReady &&
    measuredDistance < 999 &&
    measuredDistance <= DETECTION_LIMIT_CM;

  bool newTarget =
    targetDetected &&
    !targetWasDetected;

  // A car ahead ends the cruise timer and a "no car ahead" wait.
  if (targetDetected) {
    cruiseTiming = false;
    cruiseIdle = false;
  }
  else {
    reverseEpisode = false;
  }

  if (newMeasurement) {
    if (!targetDetected) {
      resetTrendMeasurement();

      observationStarted = false;
      adaptiveForwardPwm = CRUISE_PWM;
    }
    else if (!trendTracking) {
      beginTrendMeasurement(measuredDistance);
    }
    else {
      updateTargetTrend(measuredDistance);
    }
  }

  if (newTarget) {
    enterObservationMode();
  }

  bool motorsStopped =
    activeDirection == 0 &&
    currentPwm == 0;

  bool motorsMovingReverse =
    activeDirection < 0 &&
    currentPwm > 0;

  bool stationaryBraking = false;

  // Time spent backing off in one go (cleared when the car drives forward again or the car ahead is gone).
  {
    static unsigned long lastBudgetTickAt = 0;

    if (motorsMovingReverse) {
      reverseUsedMs += min(now - lastBudgetTickAt, 100UL);
    }
    else if (
      (activeDirection > 0 && currentPwm > 0) ||
      !targetDetected
    ) {
      reverseUsedMs = 0;
    }

    lastBudgetTickAt = now;
  }

  bool reverseAllowed =
    reverseUsedMs < MAX_REVERSE_TOTAL_MS;

  // ---------------------------------------------------
  // SENSOR STARTUP
  // ---------------------------------------------------

  if (!sensorReady) {
    setDriveRequest(0, 0);
  }

  // ---------------------------------------------------
  // NO TARGET: CRUISE
  // ---------------------------------------------------

  else if (!targetDetected) {
    followIntent = INTENT_FORWARD;

    observationStarted = false;
    reverseRunTiming = false;

    adaptiveForwardPwm = CRUISE_PWM;

    if (!cruiseTiming) {
      cruiseTiming = true;
      cruiseStartedAt = now;
    }

    if (now - cruiseStartedAt >= CRUISE_MAX_MS) {
      // Nothing ahead for a long time: do not run away. A car ahead (or a hand) restarts the logic.
      cruiseIdle = true;
      setDriveRequest(0, 0);
    }
    else {
      setDriveRequest(1, CRUISE_PWM);
    }
  }

  // ---------------------------------------------------
  // STOP AND OBSERVE TARGET DIRECTION
  // ---------------------------------------------------

  else if (followIntent == INTENT_OBSERVE) {
    setDriveRequest(0, 0);

    if (motorsStopped) {
      if (!observationStarted) {
        prepareStoppedObservation();
      }

      else if (
        targetTrend == TREND_OPENING &&
        measuredDistance >= FORWARD_RESTART_CM
      ) {
        // Distance increasing:
        // target is moving forward/away.
        followIntent = INTENT_FORWARD;
        observationStarted = false;

        adaptiveForwardPwm =
          max(adaptiveForwardPwm, FOLLOW_INITIAL_PWM);

        setDriveRequest(
          1,
          adaptiveForwardPwm
        );
      }

      else if (targetTrend == TREND_CLOSING && reverseAllowed) {
        // Distance decreasing:
        // target is reversing toward the car.
        followIntent = INTENT_REVERSE;
        observationStarted = false;
        reverseRunTiming = false;

        int reversePwm =
          calculateReversePwm(
            estimatedGapSpeedCmS
          );

        setDriveRequest(-1, reversePwm);
      }

      else if (targetTrend == TREND_STABLE) {
        // Target speed is zero.
        followIntent = INTENT_STOPPED;

        stationaryBraking =
          measuredDistance <=
          BRAKE_DISTANCE_CM;

        setDriveRequest(0, 0);
      }
    }
  }

  // ---------------------------------------------------
  // STATIONARY TARGET
  // ---------------------------------------------------

  else if (followIntent == INTENT_STOPPED) {
    setDriveRequest(0, 0);

    stationaryBraking =
      measuredDistance <= BRAKE_DISTANCE_CM;

    if (
      targetTrend == TREND_OPENING &&
      measuredDistance >= FORWARD_RESTART_CM
    ) {
      // Target begins moving forward.
      followIntent = INTENT_FORWARD;

      adaptiveForwardPwm =
        max(adaptiveForwardPwm, FOLLOW_INITIAL_PWM);

      setDriveRequest(
        1,
        adaptiveForwardPwm
      );
    }
    else if (targetTrend == TREND_CLOSING && reverseAllowed) {
      // Target begins reversing.
      followIntent = INTENT_REVERSE;
      reverseRunTiming = false;

      int reversePwm =
        calculateReversePwm(
          estimatedGapSpeedCmS
        );

      setDriveRequest(-1, reversePwm);
    }
  }

  // ---------------------------------------------------
  // FORWARD FOLLOW
  // ---------------------------------------------------

  else if (followIntent == INTENT_FORWARD) {
    bool dangerouslyClose =
      measuredDistance <=
      RECHECK_DISTANCE_CM;

    bool rapidlyClosing =
      targetTrend == TREND_CLOSING &&
      estimatedGapSpeedCmS >=
        URGENT_CLOSING_SPEED_CM_S &&
      measuredDistance <= 25;

    if (dangerouslyClose || rapidlyClosing) {
      // Stop and measure the target independently
      // before choosing reverse.
      enterObservationMode();
      setDriveRequest(0, 0);
    }
    else {
      int forwardPwm =
        calculateForwardFollowPwm(
          measuredDistance
        );

      setDriveRequest(1, forwardPwm);
    }
  }

  // ---------------------------------------------------
  // REVERSE FOLLOW
  // ---------------------------------------------------

  else if (followIntent == INTENT_REVERSE) {
    if (motorsMovingReverse) {
      if (!reverseRunTiming) {
        reverseRunTiming = true;
        reverseRunStartedAt = now;
      }
    }

    bool reverseRanLongEnough =
      reverseRunTiming &&
      now - reverseRunStartedAt >=
        MIN_REVERSE_RUN_MS;

    bool gapSafelyOpening =
      targetTrend == TREND_OPENING &&
      measuredDistance >=
        REVERSE_RELEASE_CM;

    if (
      !reverseAllowed ||
      (reverseRanLongEnough &&
      gapSafelyOpening)
    ) {
      // Stop and check whether the target changed
      // from reverse to forward.
      enterObservationMode();
      setDriveRequest(0, 0);
    }
    else {
      int reversePwm =
        calculateReversePwm(
          estimatedGapSpeedCmS
        );

      setDriveRequest(-1, reversePwm);
    }
  }

  serviceMotors();

  bool transitionBraking =
    drivePhase == DRIVE_DIRECTION_WAIT &&
    lastStoppedDirection > 0 &&
    requestedDirection < 0;

  // Brake light, BRAKING screen and ONE beep whenever the car brakes or stands behind a car ahead
  // (the old code only did this closer than 10 cm).
  if (
    targetDetected &&
    requestedDirection == 0 &&
    followIntent <= INTENT_STOPPED &&
    (
      (activeDirection == 0 && currentPwm == 0) ||
      (drivePhase == DRIVE_RAMP_DOWN && activeDirection > 0)
    )
  ) {
    stationaryBraking = true;
  }

  // The beeps and the REVERSING screen stay alive between the short back-off steps (0.8 s).
  static unsigned long lastReverseMovingAt = 0;

  if (activeDirection < 0 && currentPwm > 0) {
    lastReverseMovingAt = now;
  }

  bool reverseHold =
    lastReverseMovingAt != 0 &&
    now - lastReverseMovingAt < 800UL;

  bool displayBraking =
    (stationaryBraking && !reverseHold) ||
    transitionBraking;

  bool reversing =
    reverseHold &&
    !displayBraking;

  updateAlerts(
    displayBraking,
    reversing
  );

  updateCluster(
    targetDetected,
    displayBraking && !transitionBraking,
    transitionBraking,
    reversing
  );

  if (newMeasurement) {
    Serial.print("D:");
    Serial.print(measuredDistance);

    Serial.print(" Trend:");

    if (targetTrend == TREND_OPENING) {
      Serial.print("OPENING");
    }
    else if (targetTrend == TREND_CLOSING) {
      Serial.print("CLOSING");
    }
    else if (targetTrend == TREND_STABLE) {
      Serial.print("STABLE");
    }
    else {
      Serial.print("CHECK");
    }

    Serial.print(" Intent:");

    if (followIntent == INTENT_FORWARD) {
      Serial.print("FORWARD");
    }
    else if (followIntent == INTENT_REVERSE) {
      Serial.print("REVERSE");
    }
    else if (followIntent == INTENT_STOPPED) {
      Serial.print("STOPPED");
    }
    else {
      Serial.print("OBSERVE");
    }

    Serial.print(" PWM:");
    Serial.print(currentPwm);

    Serial.print(" L/R:");
    Serial.print(lastLeftPwm);
    Serial.print("/");
    Serial.print(lastRightPwm);

    Serial.print(" HDG:");
    if (gyro.healthy()) {
      Serial.println((int)(gyro.angle() - holdYawRef));
    } else {
      Serial.println("GYRO OFF");
    }
  }

  targetWasDetected = targetDetected;
}
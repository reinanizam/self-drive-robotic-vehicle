#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>
#include <Wire.h>

// Existing mounting: Z points upward with the car upright. Gyro Z is positive
// anticlockwise, so negate it to match the Pi convention (right = positive).
// This is a short-term yaw estimate, not position tracking or wheel odometry.
class MavpSensors {
public:
  static const byte BLACK_STATE = LOW;

  void beginLineArray() {
    for (byte i = 0; i < 5; i++) pinMode(linePin(i), INPUT_PULLUP);
  }

  bool beginImu() {
    healthy = false;
    faultCode = 1;
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

    long sums[3] = {0, 0, 0};
    long accelSums[3] = {0, 0, 0};
    int16_t minimum[3] = {32767, 32767, 32767};
    int16_t maximum[3] = {-32767, -32767, -32767};
    for (unsigned int sample = 0; sample < 200; sample++) {
      int16_t accel[3], gyro[3];
      if (!readMotion(accel, gyro)) return false;
      for (byte i = 0; i < 3; i++) {
        sums[i] += gyro[i];
        accelSums[i] += accel[i];
        minimum[i] = min(minimum[i], gyro[i]);
        maximum[i] = max(maximum[i], gyro[i]);
      }
      delay(5);
    }
    for (byte i = 0; i < 3; i++) {
      float mean = sums[i] / 200.0f;
      if ((long)maximum[i] - minimum[i] > 197L || mean < -655 || mean > 655)
        return false;
    }
    // Calibrate only in the tested upright orientation, not during a side tilt.
    float ax = accelSums[0] / (200.0f * 16384.0f);
    float ay = accelSums[1] / (200.0f * 16384.0f);
    float az = accelSums[2] / (200.0f * 16384.0f);
    if (ax < -0.35f || ax > 0.35f || ay < -0.35f || ay > 0.35f ||
        az < 0.70f || az > 1.30f) { faultCode = 5; return false; }
    zBias = sums[2] / 200.0f;
    yawRateRight = 0.0f;
    yawDegreesRight = 0.0f;
    lastImuUs = micros();
    healthy = true;
    faultCode = 0;
    return true;
  }

  void update() {
    updateLine();
    if (!healthy) return;
    unsigned long now = micros();
    unsigned long elapsed = now - lastImuUs;
    if (elapsed < 10000UL) return;
    int16_t accel[3], gyro[3];
    // A fault latches until reset/recalibration; it cannot silently resume drive.
    if (elapsed > 200000UL) faultCode = 3;
    else if (!readMotion(accel, gyro)) faultCode = 2;
    else if (gyro[2] > 32000 || gyro[2] < -32000) faultCode = 4;
    if (faultCode != 0) {
      healthy = false;
      return;
    }
    // +/-500 deg/s avoids the old +/-250 range clipping during a fast turn.
    float measuredRate = -(gyro[2] - zBias) / 65.5f;
    yawRateRight += 0.35f * (measuredRate - yawRateRight);
    yawDegreesRight += measuredRate * (elapsed / 1000000.0f);
    lastImuUs = now;
  }

  bool imuHealthy() const { return healthy; }
  byte imuFault() const { return faultCode; }
  float rightRate() const { return yawRateRight; }
  float rightAngle() const { return yawDegreesRight; }
  byte mask() const { return lineMask; }
  int lineError() const { return linePosition; }
  bool lineValid() const { return validLine; }
  byte lineConfidence() const { return confidence; }
  byte blackCount() const { return activeCount; }
  bool lineSaturated() const { return lineMask == 31; }
private:
  byte address = 0;
  bool healthy = false;
  byte faultCode = 1; // 0 OK, 1 startup, 2 I2C, 3 gap, 4 clip, 5 mount.
  float zBias = 0.0f;
  float yawRateRight = 0.0f;
  float yawDegreesRight = 0.0f;
  unsigned long lastImuUs = 0;
  unsigned long lastLineAt = 0;
  byte evidence[5] = {0, 0, 0, 0, 0};
  byte lineMask = 0;
  bool validLine = false;
  int linePosition = 0;
  byte activeCount = 0;
  byte confidence = 0;

  static byte linePin(byte index) {
    const byte pins[5] = {A2, 13, 12, 11, 10};
    return pins[index];
  }

  void updateLine() {
    unsigned long now = millis();
    if (now - lastLineAt < 10) return;
    lastLineAt = now;
    byte highCounts[5] = {0, 0, 0, 0, 0};
    for (byte sample = 0; sample < 5; sample++) {
      for (byte i = 0; i < 5; i++) {
        if (digitalRead(linePin(i)) == HIGH) highCounts[i]++;
      }
      delayMicroseconds(100);
    }
    for (byte i = 0; i < 5; i++) {
      byte value = highCounts[i] >= 3 ? HIGH : LOW;
      if (value == BLACK_STATE) {
        if (evidence[i] < 3) evidence[i]++;
      } else if (evidence[i] > 0) {
        evidence[i]--;
      }
      if (evidence[i] == 3) lineMask |= (1 << i);
      if (evidence[i] == 0) lineMask &= ~(1 << i);
    }
    byte count = 0;
    int sum = 0;
    bool ended = false;
    bool contiguous = true;
    for (byte i = 0; i < 5; i++) {
      if (lineMask & (1 << i)) {
        if (ended) contiguous = false;
        count++;
        sum += ((int)i - 2) * 50;
      } else if (count > 0) {
        ended = true;
      }
    }
    // A binary array measures a centroid, not an analogue percentage of tape.
    // Four adjacent black channels still indicate a coarse left/right offset,
    // but one clipped edge makes that estimate less certain. All five provide
    // no centre/edge information: let the camera decide, never invent centre.
    activeCount = count;
    validLine = count > 0 && count <= 4 && contiguous;
    linePosition = validLine ? sum / count : 0;
    confidence = !validLine ? 0 : count == 4 ? 30 : count == 3 ? 80 : 95;
  }

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
    if (Wire.endTransmission(false) != 0 ||
        Wire.requestFrom(address, (byte)1) != 1) return false;
    value = Wire.read();
    return true;
  }

  int16_t readWord() {
    uint16_t high = (uint8_t)Wire.read();
    uint16_t low = (uint8_t)Wire.read();
    return (int16_t)((high << 8) | low);
  }

  bool readMotion(int16_t accel[3], int16_t gyro[3]) {
    Wire.beginTransmission(address);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0 ||
        Wire.requestFrom(address, (byte)14) != 14) return false;
    for (byte i = 0; i < 3; i++) accel[i] = readWord();
    readWord();
    for (byte i = 0; i < 3; i++) gyro[i] = readWord();
    return true;
  }
};

/*
  MAVP Stage 8 — combined ACC + Raspberry Pi vision steering

  Stage 6 remains responsible for longitudinal safety, distance following,
  braking and reverse. The matching Raspberry Pi supplies structured GUIDE4,
  TRACK_LOST or STOP over USB serial. Missing vision commands stop both motors.
  This version adds the tested line array and an upright Z-axis MPU6050.
  Raise the wheels for the first powered check; gyro gains require road tuning.
*/

// =====================================================
// HARDWARE
// =====================================================

const int ENA = 5;
const int IN1 = 4;
const int IN2 = 7;

const int IN3 = 8;
const int IN4 = 9;
const int ENB = 6;

// Reina's powered test: the old +RIGHT command physically turned LEFT.
// Swap physical motor banks, NOT camera/IR coordinates. Both banks still
// retain their proven forward/reverse polarity. A = right on this chassis.
const bool MOTOR_A_IS_LEFT = false;
const int LEFT_PWM_PIN = MOTOR_A_IS_LEFT ? ENA : ENB;
const int RIGHT_PWM_PIN = MOTOR_A_IS_LEFT ? ENB : ENA;

const int TRIG = A0;
const int ECHO = A1;

const int BRAKE_LED = 2;
const int BUZZER = 3;

LiquidCrystal_I2C lcd(0x27, 16, 2);
MavpSensors localSensors;

// =====================================================
// DISTANCE CONTROL
// =====================================================

// Detect early enough to slow down before the close zone.
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

// A short strong kick starts all four wheels. Running PWM is lower.
const int FORWARD_START_PWM = 245;
const int REVERSE_START_PWM = 220;

// Keep the proven motor values: this chassis did not move reliably below
// these PWM settings.  Steering changes are applied separately below.
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
// RASPBERRY PI VISION CONTROL
// =====================================================

const unsigned long VISION_TIMEOUT_MS = 400;
// Legacy GUIDE2/3 use joint acquisition. V5 accepts stable live camera-led
// acquisition; only weak/remembered guidance needs a short recovery budget.
const unsigned long RECOVERY_CAMERA_MS = 1800;
const unsigned long RECOVERY_IR_MS = 1000;
const unsigned long RECOVERY_STRAIGHT_GAP_MS = 350;
// A live camera still measures a real path when the array briefly goes blank.
// Crawl to rejoin rather than expiring in 350 ms before correcting the offset.
const unsigned long CAMERA_REJOIN_MS = 1200;
const unsigned long RECOVERY_REJOIN_MS = 120;
const float RECOVERY_MAX_YAW_DEG = 75.0f;
const int RECOVERY_CLEARANCE_CM = 30;
// The previous 10-count reduction was too small to turn a four-wheel chassis.
// Use a stronger differential: keep the outside wheels powered and slow the
// inside wheels enough to create a real turn.
const int STEERING_INNER_MIN_PWM = FORWARD_MIN_PWM;
// Gentle turns keep BOTH sides above their proven running floor. For an
// observed sharp corner only, stopping the inside side deliberately provides
// a forward pivot instead of feeding a stalled motor a weak PWM.
const bool ENABLE_SHARP_CORNER_PIVOT = true;
const int MAX_STEERING_REDUCTION = 55;
const int MAX_STEERING_BOOST = 25;
const int STEERING_FULL_SCALE_ERROR = 65;
const int STEERING_DEADBAND = 3;
const int OCCLUDED_TRACK_MAX_PWM = 210;

char visionCommand[48];
byte visionCommandLength = 0;

int steeringError = 0;  // Camera command; right is positive.
int motorSteeringError = 0;
int rememberedPathError = 0;
byte cameraQuality = 0;
byte cameraBend = 0;
int cameraNearError = 0;
int cameraPreviewError = 0;
bool splitCameraGuide = false;
const int lineCenterTrim = 0; // Geometric middle; never learn an arbitrary off-centre startup.
bool sharpCornerMode = false;
unsigned long lastFusionAt = 0;
int previousPathError = 0;
float pathDerivative = 0;
float filteredSteering = 0;
float desiredYawRate = 0;
float yawEffort = 0;
float yawServoIntegral = 0;
float turnGainRight = 120.0f; // Initial guesses, refined only from measured yaw.
float turnGainLeft = 120.0f;
float previousYawEffort = 0;
unsigned long yawSteadySince = 0;
float innerPulseAccumulator = 0;
float travelPulseAccumulator = 0;
byte cameraPace = 100;
bool arrivalGuide = false;
bool handoffGuide = false;
byte cameraGuidePhase = 0;
bool cameraLeading = false;
bool motorDirectionFault = false; // Latches until reset; never auto-flip moving motors.
unsigned long wrongYawSince = 0;
bool cameraTurnPlanned = false;
bool extendedCorner = false;
unsigned long cornerSequence = 0;
float lastOpticalHeading = 0;
int lastLeftPwm = 0;
int lastRightPwm = 0;
bool discardingVisionFrame = false;
unsigned long lastTelemetryAt = 0;
unsigned long lastVisionCommandAt = 0;

bool visionEverConnected = false;
bool visionTapeVisible = false;
bool visionEmergencyStop = true;
bool trackArmed = false;
bool selfCentering = false;
bool alignmentTiming = false;
unsigned long alignmentStartedAt = 0;
unsigned long visionSequence = 0;
unsigned long alignedSequence = 0;
byte alignedFrames = 0;

enum RecoveryMode { RECOVERY_NONE, RECOVERY_CAMERA, RECOVERY_IR, RECOVERY_MEMORY };
RecoveryMode recoveryMode = RECOVERY_NONE;
bool hadConfirmedTrack = false;
bool recoveryLimitReached = false;
unsigned long lastJointSeenAt = 0;
unsigned long recoveryStartedAt = 0;
unsigned long recoveryBudgetMs = 0;
float recoveryStartYaw = 0;
int lastConfirmedLocalError = 0;
int lastConfirmedPreview = 0;
bool cornerRemembered = false;
unsigned long cornerObservedAt = 0;
float cornerObservedYaw = 0;
int cornerSign = 0;

// Sonar remains a longitudinal measurement, never proof of a lateral track.
bool targetCurrentlyDetected = false;

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
// VISION SERIAL
// =====================================================

bool visionCommandsOnline() {
  return
    visionEverConnected &&
    millis() - lastVisionCommandAt <=
      VISION_TIMEOUT_MS;
}

bool trackSourcesConfirmed() {
  return visionTapeVisible && cameraQuality >= 50 && localSensors.lineValid();
}

bool recoveryObstacleClear() {
  // Same no-echo convention as existing ACC; no claim of rear protection.
  // Recovery is forward-only and leaves a larger gap than normal braking.
  return sensorReady && millis() - lastSensorReadingAt <= 200 &&
    measuredDistance > RECOVERY_CLEARANCE_CM &&
    lastRawDistance > RECOVERY_CLEARANCE_CM;
}

bool strongCameraTrack() {
  return handoffGuide && visionTapeVisible && cameraQuality >= 85 &&
    abs(cameraNearError) <= 60;
}

unsigned long recoveryRemainingMs() {
  if (recoveryMode == RECOVERY_NONE) return 0;
  unsigned long elapsed = millis() - recoveryStartedAt;
  return elapsed < recoveryBudgetMs ? recoveryBudgetMs - elapsed : 0;
}

bool recoveryAngleExceeded() {
  float rotation = localSensors.rightAngle() - recoveryStartYaw;
  return rotation >= RECOVERY_MAX_YAW_DEG || rotation <= -RECOVERY_MAX_YAW_DEG;
}

bool rememberedCornerSafe() {
  float progress = (localSensors.rightAngle() - cornerObservedYaw) * cornerSign;
  return cornerRemembered && millis() - cornerObservedAt <= (extendedCorner ? 800UL : 450UL) &&
    progress >= -5 && progress < (extendedCorner ? 30 : 18);
}

bool recoveryEvidenceSafe() {
  if (!recoveryRemainingMs() || !recoveryObstacleClear() ||
      recoveryAngleExceeded())
    return false;
  if (trackSourcesConfirmed()) return true; // Both must settle before rejoin.
  bool cameraOnly = visionTapeVisible && cameraQuality >= 70 &&
    localSensors.mask() == 0;
  bool irOnly = localSensors.lineValid() && localSensors.lineConfidence() >= 70 &&
    (!visionTapeVisible || cameraQuality < 50);
  // A previously observed close corner can bridge a VERY short loss of both
  // guides. This is executing a known turn, not a blind/random sweep.
  bool rememberedTurn = localSensors.mask() == 0 && rememberedCornerSafe();
  return cameraOnly || irOnly || rememberedTurn;
}

bool trackGuidanceAvailable() {
  if (!trackArmed || !visionCommandsOnline() || visionEmergencyStop ||
      !localSensors.imuHealthy() || motorDirectionFault) return false;
  if (recoveryMode != RECOVERY_NONE && (activeDirection < 0 || requestedDirection < 0)) return false;
  if (cameraLeading && recoveryMode == RECOVERY_NONE)
    return strongCameraTrack() && recoveryObstacleClear();
  return recoveryMode != RECOVERY_NONE ? recoveryEvidenceSafe() : trackSourcesConfirmed();
}

bool tapeIsCentered() {
  int nearOffset = splitCameraGuide ? cameraNearError : steeringError;
  return abs(nearOffset) <= 18 &&
    abs(localSensors.lineError() - lineCenterTrim) <= 25 &&
    localSensors.lineConfidence() >= 70;
}

void resetTrackConfirmation() {
  alignmentTiming = false;
  alignedFrames = 0;
  alignedSequence = visionSequence;
}

void disarmTrack() {
  cameraLeading = false;
  lastConfirmedLocalError = 0; // Old-run IR position is not current evidence.
  trackArmed = false;
  selfCentering = false;
  hadConfirmedTrack = false;
  cornerRemembered = false;
  extendedCorner = false;
  cameraTurnPlanned = false;
  travelPulseAccumulator = innerPulseAccumulator = 0;
  recoveryMode = RECOVERY_NONE;
  resetTrackConfirmation();
}

bool confirmedFor(unsigned long duration) {
  if (!alignmentTiming) {
    alignmentTiming = true;
    alignmentStartedAt = millis();
  }
  if (alignedSequence != visionSequence) {
    alignedSequence = visionSequence;
    if (alignedFrames < 255) alignedFrames++;
  }
  return millis() - alignmentStartedAt >= duration && alignedFrames >= 3;
}

void rememberJointTrack() {
  hadConfirmedTrack = true;
  lastJointSeenAt = millis();
  if (localSensors.lineValid() && localSensors.lineConfidence() >= 70)
    lastConfirmedLocalError = constrain(localSensors.lineError() - lineCenterTrim, -100, 100);
  lastConfirmedPreview = splitCameraGuide ? cameraPreviewError : steeringError;
  lastOpticalHeading = localSensors.rightAngle();
}

float floatAbs(float value) { return value < 0 ? -value : value; }

byte travelPaceLimit() {
  if (recoveryMode == RECOVERY_NONE) return cameraLeading ? min(cameraPace, (byte)60) : cameraPace;
  byte cap = arrivalGuide && recoveryMode == RECOVERY_CAMERA ? 35 : 45;
  return min(cameraPace, cap);
}

void updateYawServo(float dt) {
  float previousDesired = desiredYawRate;
  bool entered = handoffGuide && rememberedCornerSafe();
  float rateLimit = handoffGuide ? (entered ? 50.0f : 32.0f) : 85.0f;
  desiredYawRate = constrain(motorSteeringError * (handoffGuide ? 0.65f : 0.85f), -rateLimit, rateLimit);
  if (abs(motorSteeringError) <= 4) desiredYawRate = 0;
  if (desiredYawRate * previousDesired < 0) {
    yawServoIntegral = 0;
    yawEffort = 0;
  } else if (desiredYawRate == 0 && floatAbs(previousDesired) > 5) {
    yawServoIntegral = 0;
    yawEffort *= 0.5f;
  }
  float measured = localSensors.rightRate();
  // A reversed bank or gyro convention must not produce positive feedback.
  // Require sustained actual opposite yaw, not one noisy sample or zero yaw
  // while the wheels are raised. Stop instead of guessing/flipping in motion.
  bool moving = activeDirection > 0 && requestedDirection > 0 &&
    drivePhase == DRIVE_RUNNING && currentPwm >= FORWARD_MIN_PWM;
  if (moving && floatAbs(yawEffort) >= 0.25f &&
      ((yawEffort > 0 && measured < -8) || (yawEffort < 0 && measured > 8))) {
    if (!wrongYawSince) wrongYawSince = millis();
    if (millis() - wrongYawSince >= 300) motorDirectionFault = true;
  } else wrongYawSince = 0;
  float error = desiredYawRate - measured;
  if (dt > 0 && dt <= 0.10f) {
    if (!moving || desiredYawRate * measured <= 0 || floatAbs(measured) < 3)
      yawServoIntegral *= 0.95f; // No wind-up against a blocked/raised chassis.
    else if (floatAbs(error) < 2) yawServoIntegral *= 0.995f;
    else yawServoIntegral = constrain(yawServoIntegral + error * dt * 0.0015f, -0.22f, 0.22f);
  }
  float pace = travelPaceLimit() / 100.0f;
  float gain = (desiredYawRate >= 0 ? turnGainRight : turnGainLeft) * pace;
  float limit = handoffGuide ? (entered ? 0.85f : 0.65f) : 1.0f;
  float target = constrain(desiredYawRate / gain + 0.003f * error + yawServoIntegral, -limit, limit);
  // Never keep turning the wrong way because of a stale integral.
  if ((target < 0 && desiredYawRate > 0) || (target > 0 && desiredYawRate < 0)) target = 0;
  float change = max(dt, 0.01f) * 5.0f;
  yawEffort += constrain(target - yawEffort, -change, change);

  // Bounded online actuator identification, not a trained ML model. Learn
  // only with joint optical evidence, steady command and real matching yaw.
  if (floatAbs(yawEffort - previousYawEffort) > 0.035f) yawSteadySince = millis();
  if (trackArmed && visionCommandsOnline() && activeDirection > 0 && drivePhase == DRIVE_RUNNING &&
      currentPwm >= FORWARD_MIN_PWM &&
      trackSourcesConfirmed() && recoveryMode == RECOVERY_NONE && cameraQuality >= 70 &&
      localSensors.lineConfidence() >= 70 && millis() - yawSteadySince >= 150 &&
      floatAbs(yawEffort) >= 0.15f && floatAbs(measured) >= 8 && floatAbs(measured) <= 200 &&
      measured * yawEffort > 0) {
    float observedGain = constrain(measured / (yawEffort * pace), 35.0f, 300.0f);
    if (yawEffort > 0) turnGainRight += 0.025f * (observedGain - turnGainRight);
    else turnGainLeft += 0.025f * (observedGain - turnGainLeft);
  }
  previousYawEffort = yawEffort;
}

void updateTrackAgreement() {
  if (!visionCommandsOnline() || visionEmergencyStop || !localSensors.imuHealthy() || motorDirectionFault) {
    disarmTrack();
    recoveryLimitReached = false;
    return;
  }

  // Strong, real camera observations can acquire/maintain drive without IR.
  // This is current visual tracking, not a renewable blind recovery timer.
  if (strongCameraTrack() && recoveryObstacleClear()) {
    if ((trackArmed && recoveryMode == RECOVERY_NONE) ||
        (confirmedFor(400) && alignedFrames >= 8)) {
      trackArmed = true;
      cameraLeading = !localSensors.lineValid() || localSensors.lineConfidence() < 70;
      recoveryMode = RECOVERY_NONE;
      recoveryLimitReached = false;
      selfCentering = !tapeIsCentered();
      rememberJointTrack();
      resetTrackConfirmation();
    } else if (trackArmed && !recoveryEvidenceSafe()) disarmTrack();
    return;
  }
  if (cameraLeading) cameraLeading = false;

  if (recoveryMode != RECOVERY_NONE) {
    if (!recoveryEvidenceSafe()) {
      recoveryLimitReached = !recoveryRemainingMs() ||
        recoveryAngleExceeded();
      disarmTrack();
      return;
    }
    if (trackSourcesConfirmed() && localSensors.lineConfidence() >= 70) {
      if (confirmedFor(RECOVERY_REJOIN_MS)) {
        recoveryMode = RECOVERY_NONE;
        selfCentering = !tapeIsCentered();
        recoveryLimitReached = false;
        rememberJointTrack();
      }
    } else {
      // One good frame cannot renew the recovery budget. Only a stable
      // camera+IR rejoin can do so; switching sources keeps the same deadline.
      resetTrackConfirmation();
      recoveryMode = localSensors.lineValid() ? RECOVERY_IR :
        visionTapeVisible && cameraQuality >= 70 ? RECOVERY_CAMERA : RECOVERY_MEMORY;
    }
    return;
  }

  if (trackArmed && trackSourcesConfirmed()) {
    if (selfCentering && tapeIsCentered()) selfCentering = false;
    if (localSensors.lineConfidence() >= 70) rememberJointTrack();
    return;
  }

  if (trackArmed && hadConfirmedTrack && millis() - lastJointSeenAt <= 200 &&
      activeDirection > 0 && requestedDirection > 0 && currentPwm > 0) {
    bool cameraOnly = visionTapeVisible && cameraQuality >= 70 && localSensors.mask() == 0;
    bool irOnly = localSensors.lineValid() && localSensors.lineConfidence() >= 70 &&
      (!visionTapeVisible || cameraQuality < 50);
    bool rememberedTurn = localSensors.mask() == 0 && rememberedCornerSafe() &&
      millis() - cornerObservedAt <= 200;
    if ((cameraOnly || irOnly || rememberedTurn) && recoveryObstacleClear()) {
      recoveryMode = cameraOnly ? RECOVERY_CAMERA : irOnly ? RECOVERY_IR : RECOVERY_MEMORY;
      recoveryBudgetMs = cameraOnly ? RECOVERY_CAMERA_MS : RECOVERY_IR_MS;
      if (recoveryMode == RECOVERY_MEMORY) recoveryBudgetMs = extendedCorner ? 800 : 450;
      int nearOffset = splitCameraGuide ? cameraNearError : steeringError;
      bool turnEvidence = abs(nearOffset) >= 12 || abs(lastConfirmedLocalError) >= 50 ||
        (abs(cameraPreviewError) >= 25 && cameraBend >= 25);
      if (cameraOnly && !turnEvidence)
        recoveryBudgetMs = arrivalGuide ? CAMERA_REJOIN_MS : RECOVERY_STRAIGHT_GAP_MS;
      recoveryStartedAt = millis();
      recoveryStartYaw = localSensors.rightAngle();
      selfCentering = true;
      recoveryLimitReached = false;
      resetTrackConfirmation();
      lastFusionAt = 0;
      return;
    }
  }

  if (!trackSourcesConfirmed()) {
    disarmTrack();
    return;
  }
  if (localSensors.lineConfidence() < 70) {
    resetTrackConfirmation();
    return;
  }
  // Startup/restart needs reliable agreement, but never manual centring.
  if (confirmedFor(250)) {
    trackArmed = true;
    selfCentering = !tapeIsCentered();
    recoveryLimitReached = false;
    rememberJointTrack();
  }
}

void updateFusedSteering() {
  if (!trackSourcesConfirmed() && !trackGuidanceAvailable()) {
    motorSteeringError = 0;
    sharpCornerMode = false;
    lastFusionAt = 0;
    filteredSteering = 0;
    pathDerivative = 0;
    previousPathError = 0;
    desiredYawRate = yawEffort = yawServoIntegral = 0;
    return;
  }
  // Reverse follows the previously measured forward arc. Do not apply forward
  // yaw feedback in reverse: its correction sign would be wrong.
  if (activeDirection < 0 || requestedDirection < 0) {
    motorSteeringError = rememberedPathError;
    sharpCornerMode = false;
    return;
  }
  unsigned long now = millis();
  unsigned long elapsed = now - lastFusionAt;
  if (elapsed < 10 && lastFusionAt != 0) return;
  float dt = min(elapsed, 100UL) / 1000.0f;
  bool restartFilter = lastFusionAt == 0 || elapsed > 150;
  lastFusionAt = now;
  int localError = constrain(localSensors.lineError() - lineCenterTrim, -100, 100);
  bool opposedLocal = localSensors.lineConfidence() >= 70 && abs(localError) >= (handoffGuide ? 75 : 50) &&
    (long)localError * cameraPreviewError < 0;
  bool localEntry = localSensors.lineConfidence() >= 70 && abs(localError) >= 25 &&
    (long)localError * cameraPreviewError > 0;
  bool turnNow = handoffGuide && cameraGuidePhase > 0 && cameraBend >= 50 &&
    (localEntry || (cornerRemembered && rememberedCornerSafe() &&
      (long)cornerSign * cameraPreviewError > 0) ||
     (cameraLeading && cameraGuidePhase == 2 && cameraBend >= 90));
  // One image may be serviced by many 10 ms gyro cycles. Only a NEW packet
  // containing an actually observed entry can renew the corner evidence.
  if (cornerSequence != visionSequence && visionTapeVisible && cameraQuality >= 70 &&
      cameraBend >= 50 && abs(cameraPreviewError) >= 35 &&
      !opposedLocal && (handoffGuide ? turnNow && cameraGuidePhase != 3 :
      (!arrivalGuide || cameraTurnPlanned)) && (trackArmed || hadConfirmedTrack)) {
    cornerRemembered = true;
    extendedCorner = handoffGuide || (arrivalGuide && cameraTurnPlanned);
    cornerObservedAt = now;
    cornerObservedYaw = localSensors.rightAngle();
    cornerSign = cameraPreviewError > 0 ? 1 : -1;
  } else if (opposedLocal || ((!handoffGuide || cameraGuidePhase == 0) &&
             visionTapeVisible && cameraQuality >= 70 && cameraBend < 20 &&
             abs(cameraPreviewError) < 12)) cornerRemembered = false;
  cornerSequence = visionSequence;
  int pathError = rememberedPathError;
  if (recoveryMode != RECOVERY_NONE && !trackSourcesConfirmed()) {
    float remaining = recoveryRemainingMs() / (float)max(recoveryBudgetMs, 1UL);
    if (localSensors.lineValid()) {
      // Near IR is live; the last observed preview fades rather than holding
      // a stale corner indefinitely. Subtract heading already executed.
      float yawProgress = localSensors.rightAngle() - lastOpticalHeading;
      bool planned = extendedCorner && rememberedCornerSafe();
      float hint = constrain(lastConfirmedPreview * (planned ? 0.55f : 0.25f) - yawProgress * 1.5f,
                             planned ? -45.0f : -20.0f, planned ? 45.0f : 20.0f);
      if (handoffGuide && !planned) hint = 0; // An unseen FUTURE bend is not a current yaw request.
      if ((long)localError * lastConfirmedPreview < 0 && abs(localError) >= 50) hint *= 0.20f;
      pathError = constrain((int)(localError + hint * remaining), -100, 100);
    } else if (visionTapeVisible && cameraQuality >= 70) {
      // The array is blank, not centred. Use current camera near position,
      // preview and fading last-known IR offset; never average in a fake zero.
      int nearOffset = splitCameraGuide ? cameraNearError : steeringError;
      float urgency = constrain((cameraBend - 25) / 65.0f, 0.0f, 1.0f);
      float preview = splitCameraGuide ? cameraPreviewError * (0.25f + 0.35f * urgency) : 0;
      if (handoffGuide && !turnNow) preview = 0;
      float memory = lastConfirmedLocalError * 0.25f * remaining;
      if ((long)nearOffset * lastConfirmedLocalError < 0 && abs(nearOffset) >= 15) memory *= 0.20f;
      pathError = constrain((int)(nearOffset * 1.35f + preview + memory), -100, 100);
    } else {
      float executed = (localSensors.rightAngle() - cornerObservedYaw) * cornerSign;
      // Ease off as the observed heading budget is consumed. The old fixed
      // minimum kept asking for a large turn right up to the recovery cutoff.
      pathError = handoffGuide ? cornerSign * constrain((int)((30 - executed) * 1.2f), 0, 36) :
        cornerSign * constrain((int)((extendedCorner ? 60 : 48) - executed), 25, 60);
    }
  } else if (localSensors.lineValid() && visionTapeVisible) {
    if (splitCameraGuide) {
      // IR measures the tape under the axle; camera near is only a weaker
      // position fallback. Preview changes desired heading, not that centre.
      float localQ = localSensors.lineConfidence() / 100.0f;
      float cameraQ = cameraQuality / 100.0f;
      float urgency = constrain((cameraBend - 25) / 65.0f, 0.0f, 1.0f);
      float localWeight = (selfCentering ? 0.60f : 0.85f) * localQ;
      float cameraWeight = (selfCentering ? 0.40f : 0.15f) * cameraQ;
      float lateral = (localError * localWeight + cameraNearError * cameraWeight) /
        max(localWeight + cameraWeight, 0.01f);
      float preview = cameraPreviewError * (0.12f + 0.60f * urgency) * cameraQ;
      if (handoffGuide && localSensors.lineConfidence() >= 70) lateral = localError;
      if (handoffGuide) preview = turnNow ? cameraPreviewError * 0.70f * cameraQ : 0;
      // A still-centred array limits early turn-in. Edge evidence permits a
      // decisive turn. Opposing IR always has enough authority to recenter.
      float limit = (abs(localError) <= 25 ? 15.0f + 35.0f * urgency : 70.0f);
      preview = constrain(preview, -limit, limit);
      if ((long)localError * cameraPreviewError < 0 && abs(localError) >= 50)
        preview *= handoffGuide && abs(localError) < 75 ? 0.60f : 0.20f;
      if (selfCentering && abs(localError) >= 50) preview *= 0.50f;
      pathError = constrain((int)(lateral * 0.95f + preview), -100, 100);
    } else {
    // Camera influence increases as an OBSERVED bend comes near. A wide IR
    // pattern gets less authority; neither source is averaged blindly 65/35.
    long cameraWeight = (35L + cameraBend * 45L / 100) * cameraQuality / 100;
    long localWeight = 60L * localSensors.lineConfidence() / 100;
    if (abs(localError) >= 50) localWeight = localWeight * 135 / 100;
    long total = cameraWeight + localWeight;
    pathError = total > 0
      ? (int)((localError * localWeight + steeringError * cameraWeight) / total)
      : 0;
    }
  } else if (localSensors.lineValid()) {
    pathError = localError;
  } else if (visionTapeVisible) {
    pathError = handoffGuide ? constrain((int)(cameraNearError +
      (turnNow ? cameraPreviewError * 0.35f : 0)), -100, 100) : steeringError;
  }
  if (recoveryMode == RECOVERY_NONE) rememberedPathError = pathError;
  if (restartFilter || dt <= 0) {
    pathDerivative = 0;
    filteredSteering = pathError;
  } else {
    float derivative = constrain((pathError - previousPathError) / dt, -300.0f, 300.0f);
    pathDerivative += (dt / (0.10f + dt)) * (derivative - pathDerivative);
    float target = constrain(pathError * 1.12f + pathDerivative * 0.035f, -100.0f, 100.0f);
    float alpha = dt / (0.035f + dt);
    float maxChange = (abs(pathError) >= 55 ? 1200.0f : 650.0f) * dt;
    filteredSteering += constrain(alpha * (target - filteredSteering), -maxChange, maxChange);
  }
  previousPathError = pathError;
  int filteredError = (int)filteredSteering;
  motorSteeringError = constrain(filteredError, -100, 100);
  updateYawServo(dt);
  bool localCorner = localSensors.lineConfidence() >= 70 && abs(localError) >= 75;
  bool cameraCorner = visionTapeVisible && cameraQuality >= 65 && cameraBend >= 70 && abs(steeringError) >= 55;
  if (splitCameraGuide && localSensors.lineValid())
    cameraCorner = cameraCorner && abs(localError) >= 50 && (long)localError * cameraPreviewError > 0;
  sharpCornerMode = !handoffGuide && ENABLE_SHARP_CORNER_PIVOT &&
    ((abs(motorSteeringError) >= 55 && (localCorner || cameraCorner)) ||
     (selfCentering && abs(motorSteeringError) >= 28) ||
     (recoveryMode != RECOVERY_NONE && abs(motorSteeringError) >= 18));
}

void handleVisionCommand() {
  visionCommand[visionCommandLength] = '\0';
  bool valid = true;
  if (strncmp(visionCommand, "GUIDE2,", 7) == 0 || strncmp(visionCommand, "GUIDE3,", 7) == 0 ||
             strncmp(visionCommand, "GUIDE4,", 7) == 0) {
    bool phaseGuide = visionCommand[5] == '4';
    bool timedGuide = visionCommand[5] != '2';
    byte count = timedGuide ? 6 : 4;
    long values[6];
    const char *cursor = visionCommand + 7;
    for (byte i = 0; i < count; i++) {
      char *tail;
      values[i] = strtol(cursor, &tail, 10);
      bool more = i + 1 < count;
      if (tail == cursor || (more ? *tail != ',' : *tail != '\0')) { valid = false; break; }
      cursor = tail + (more ? 1 : 0);
      if (values[i] < (i < 2 ? -100 : i == 4 ? 35 : 0) ||
          values[i] > (i == 5 ? (phaseGuide ? 3 : 1) : 100)) { valid = false; break; }
    }
    if (valid) {
      cameraNearError = (int)values[0];
      cameraPreviewError = (int)values[1];
      cameraQuality = (byte)values[2];
      cameraBend = (byte)values[3];
      arrivalGuide = timedGuide;
      handoffGuide = phaseGuide;
      cameraGuidePhase = phaseGuide ? (byte)values[5] : 0;
      cameraPace = timedGuide ? (byte)values[4] : 100;
      cameraTurnPlanned = phaseGuide ? values[5] == 2 : timedGuide && values[5] == 1;
      float urgency = constrain((cameraBend - 25) / 65.0f, 0.0f, 1.0f);
      steeringError = constrain((int)(cameraNearError + cameraPreviewError * (0.20f + 0.65f * urgency)), -100, 100);
      splitCameraGuide = true;
      visionTapeVisible = cameraQuality >= 25;
      visionEmergencyStop = false;
    }
  } else if (strcmp(visionCommand, "TRACK_LOST") == 0) {
    // Camera observation unavailable; a bounded, already-acquired IR-led
    // recovery may continue. STOP remains unconditional.
    visionTapeVisible = false;
    cameraQuality = 0;
    cameraBend = 0;
    cameraTurnPlanned = false;
    visionEmergencyStop = false;
  } else if (strcmp(visionCommand, "STOP") == 0) {
    steeringError = 0;
    rememberedPathError = 0;
    cameraQuality = 0;
    cameraBend = 0;
    visionTapeVisible = false;
    visionEmergencyStop = true;
  } else {
    valid = false;
  }
  if (valid) {
    lastVisionCommandAt = millis();
    visionEverConnected = true;
    visionSequence++;
  } else {
    visionTapeVisible = false;
    visionEmergencyStop = true;
    steeringError = 0;
    rememberedPathError = 0;
    cameraQuality = 0;
    cameraBend = 0;
  }
  visionCommandLength = 0;
}

void serviceVisionSerial() {
  // Bound one pass so a continuous stream cannot starve motor/sensor checks.
  byte budget = 64;
  while (Serial.available() > 0 && budget-- > 0) {
    char incoming = Serial.read();
    if (incoming == '\n' || incoming == '\r') {
      if (!discardingVisionFrame && visionCommandLength > 0) handleVisionCommand();
      discardingVisionFrame = false;
      visionCommandLength = 0;
    } else if (!discardingVisionFrame) {
      if (visionCommandLength < sizeof(visionCommand) - 1) {
        visionCommand[visionCommandLength++] = incoming;
      } else {
        discardingVisionFrame = true;
        visionCommandLength = 0;
        visionTapeVisible = false;
        visionEmergencyStop = true;
        cameraQuality = 0;
        cameraBend = 0;
      }
    }
  }
}

void publishSensorTelemetry() {
  if (millis() - lastTelemetryAt < 200) return;
  lastTelemetryAt = millis();
  Serial.print(F("SENS,"));
  for (byte i = 0; i < 5; i++)
    Serial.print((localSensors.mask() & (1 << i)) ? '1' : '0');
  Serial.print(F(",VALID,")); Serial.print(localSensors.lineValid() ? 1 : 0);
  Serial.print(F(",IR,")); Serial.print(localSensors.lineError());
  Serial.print(F(",CAM,")); Serial.print(steeringError);
  Serial.print(F(",CMD,")); Serial.print(motorSteeringError);
  Serial.print(F(",YAW,")); Serial.print((int)localSensors.rightAngle());
  Serial.print(F(",RATE,")); Serial.print((int)localSensors.rightRate());
  Serial.print(F(",IMU,")); Serial.print(localSensors.imuHealthy() ? 1 : 0);
  Serial.print(F(",L,")); Serial.print(lastLeftPwm);
  Serial.print(F(",R,")); Serial.print(lastRightPwm);
  Serial.print(F(",Q,")); Serial.print(localSensors.lineConfidence());
  Serial.print(F(",N,")); Serial.print(localSensors.blackCount());
  Serial.print(F(",SAT,")); Serial.print(localSensors.lineSaturated() ? 1 : 0);
  Serial.print(F(",CQ,")); Serial.print(cameraQuality);
  Serial.print(F(",BEND,")); Serial.print(cameraBend);
  Serial.print(F(",PIVOT,")); Serial.print(sharpCornerMode ? 1 : 0);
  Serial.print(F(",ERR,")); Serial.print(motorDirectionFault ? 6 : localSensors.imuFault());
  Serial.print(F(",ARM,")); Serial.print(trackArmed ? 1 : 0);
  Serial.print(F(",REC,")); Serial.print(recoveryRemainingMs());
  Serial.print(F(",YR,")); Serial.print((int)desiredYawRate);
  Serial.print(F(",EFF,")); Serial.print((int)(yawEffort * 100));
  Serial.print(F(",SRC,"));
  if (!localSensors.imuHealthy() || motorDirectionFault) Serial.println(F("FAULT"));
  else if (!visionCommandsOnline()) Serial.println(F("OFF"));
  else if (visionEmergencyStop) Serial.println(F("STOP"));
  else if (recoveryLimitReached) Serial.println(F("REC_STOP"));
  else if (recoveryMode != RECOVERY_NONE && trackGuidanceAvailable()) {
    if (trackSourcesConfirmed()) Serial.println(F("REJOIN"));
    else if (recoveryMode == RECOVERY_MEMORY) Serial.println(F("TURN_MEM"));
    else if (localSensors.lineValid()) Serial.println(F("REC_IR"));
    else Serial.println(F("REC_CAM"));
  }
  else if (cameraLeading && trackGuidanceAvailable()) Serial.println(F("CAM_TRACK"));
  else if (!trackSourcesConfirmed() && !strongCameraTrack()) Serial.println(F("LOST"));
  else if (!trackArmed) Serial.println(F("CHECK"));
  else if (selfCentering) Serial.println(F("CENTER"));
  else Serial.println(F("BOTH")); // Other guidance modes were handled above.
}

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
    pulseIn(ECHO, HIGH, 8000UL);

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

  if (distance <= 24) {
    adaptiveForwardPwm =
      min(adaptiveForwardPwm, 198);
  }
  else if (distance >= 36) {
    adaptiveForwardPwm =
      max(adaptiveForwardPwm, 205);
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

void applyMotorPwm(int pwm) {
  pwm = constrain(pwm, 0, 255);
  // Final output guard also covers pending starts, reverse and ramp-down.
  if (pwm > 0 && (!visionCommandsOnline() || visionEmergencyStop ||
      !localSensors.imuHealthy() || !trackGuidanceAvailable())) pwm = 0;

  int leftPwm = pwm;
  int rightPwm = pwm;

  if (pwm > 0 && activeDirection > 0 && requestedDirection > 0 &&
      drivePhase != DRIVE_RAMP_DOWN && trackGuidanceAvailable()) {
    // Continuous turn authority: a moderate command must not collapse into
    // 215 vs 210 on both banks. Keep energized pulses above the proven motor
    // floor, but vary INSIDE-bank on-time instead of feeding a stalled low PWM.
    float magnitude = constrain(floatAbs(yawEffort), 0.0f, 1.0f);
    float insideDuty = 1.0f - magnitude;
    byte pace = travelPaceLimit();
    travelPulseAccumulator += pace / 100.0f;
    bool travelOn = travelPulseAccumulator >= 1.0f;
    if (travelOn) travelPulseAccumulator -= 1.0f;
    if (travelOn) innerPulseAccumulator += insideDuty;
    bool insideOn = travelOn && innerPulseAccumulator >= 1.0f;
    if (insideOn) innerPulseAccumulator -= 1.0f;
    int outsidePwm = constrain(pwm + (int)(magnitude * 25), FORWARD_MIN_PWM, FORWARD_START_PWM);
    int insidePwm = insideOn ? max(FORWARD_MIN_PWM, pwm) : 0;
    if (!travelOn) leftPwm = rightPwm = 0;
    else if (magnitude < 0.015f) { leftPwm = rightPwm = pwm; }
    else if (yawEffort > 0) { leftPwm = outsidePwm; rightPwm = insidePwm; }
    else { leftPwm = insidePwm; rightPwm = outsidePwm; }
    analogWrite(LEFT_PWM_PIN, leftPwm); analogWrite(RIGHT_PWM_PIN, rightPwm);
    lastLeftPwm = leftPwm; lastRightPwm = rightPwm; currentPwm = pwm;
    return;
  }

  // Reuse the remembered steering differential if ACC has already decided
  // to reverse.  With both wheel directions reversed, keeping the same PWM
  // differential retraces the recent forward arc.  This does not initiate
  // reverse; the distance-control safety state machine remains responsible
  // for that decision.
  bool motionCanSteer =
    activeDirection != 0 ||
    requestedDirection != 0;

  if (
    pwm > 0 &&
    motionCanSteer &&
    requestedDirection != 0 && drivePhase != DRIVE_RAMP_DOWN &&
    visionCommandsOnline() &&
    !visionEmergencyStop &&
    trackGuidanceAvailable()
  ) {
    int magnitude = min(
      abs(motorSteeringError),
      STEERING_FULL_SCALE_ERROR
    );

    if (magnitude > STEERING_DEADBAND) {
      int reduction = (magnitude - STEERING_DEADBAND) * MAX_STEERING_REDUCTION /
        (STEERING_FULL_SCALE_ERROR - STEERING_DEADBAND);

      reduction = constrain(
        reduction,
        0,
        min(MAX_STEERING_REDUCTION, pwm)
      );

      int boost = (magnitude - STEERING_DEADBAND) * MAX_STEERING_BOOST /
        (STEERING_FULL_SCALE_ERROR - STEERING_DEADBAND);

      boost = constrain(
        boost,
        0,
        MAX_STEERING_BOOST
      );

      if (motorSteeringError > 0) {
        // Tape is right: boost left and gently slow right.
        leftPwm = min(255, leftPwm + boost);
        rightPwm = max(
          STEERING_INNER_MIN_PWM,
          rightPwm - reduction
        );
      }
      else if (motorSteeringError < 0) {
        // Tape is left: boost right and gently slow left.
        rightPwm = min(255, rightPwm + boost);
        leftPwm = max(
          STEERING_INNER_MIN_PWM,
          leftPwm - reduction
        );
      }
    }
    // Forward motion already returned through the progressive yaw mixer above.
  }

  analogWrite(LEFT_PWM_PIN, constrain(leftPwm, 0, 255));
  analogWrite(RIGHT_PWM_PIN, constrain(rightPwm, 0, 255));

  lastLeftPwm = constrain(leftPwm, 0, 255);
  lastRightPwm = constrain(rightPwm, 0, 255);
  currentPwm = pwm;
}

void applyEqualMotorPwm(int pwm) {
  pwm = constrain(pwm, 0, 255);
  if (pwm > 0 && (!visionCommandsOnline() || visionEmergencyStop ||
      !localSensors.imuHealthy() || !trackGuidanceAvailable())) pwm = 0;

  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);

  lastLeftPwm = pwm;
  lastRightPwm = pwm;
  currentPwm = pwm;
}

void forceVisionSafetyStop() {
  unsigned long now = millis();

  if (activeDirection != 0) {
    lastStoppedDirection = activeDirection;
  }

  applyEqualMotorPwm(0);

  requestedDirection = 0;
  requestedPwm = 0;
  activeDirection = 0;

  drivePhase = DRIVE_STOPPED;
  fullyStoppedAt = now;

  reverseRunTiming = false;
  sharpCornerMode = false;
  filteredSteering = 0;
  pathDerivative = 0;
  lastFusionAt = 0;
  desiredYawRate = yawEffort = yawServoIntegral = 0;
  innerPulseAccumulator = 0;
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
  activeDirection = direction;
  drivePhase = DRIVE_START_KICK;

  if (direction > 0) {
    // Apply steering during the kick too; the old equal 260 ms kick drove
    // straight into a bend whenever the car restarted near that bend.
    applyMotorPwm(FORWARD_START_PWM);
  } else {
    applyEqualMotorPwm(REVERSE_START_PWM);
  }

  activeDirection = direction;

  drivePhase = DRIVE_START_KICK;
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
    else if (activeDirection > 0 && now - lastPwmRampAt >= PWM_RAMP_PERIOD_MS) {
      lastPwmRampAt = now;
      applyMotorPwm(FORWARD_START_PWM);
    }

    return;
  }

  if (drivePhase == DRIVE_RUNNING) {
    if (
      now - lastPwmRampAt >=
      PWM_RAMP_PERIOD_MS
    ) {
      lastPwmRampAt = now;

      int nextPwm = currentPwm;

      if (currentPwm < requestedPwm) {
        nextPwm = min(
          currentPwm + RUN_RAMP_STEP,
          requestedPwm
        );
      }
      else if (currentPwm > requestedPwm) {
        nextPwm = max(
          currentPwm - RUN_RAMP_STEP,
          requestedPwm
        );
      }

      // Always rewrite both motor PWM outputs. The Raspberry Pi can change
      // steeringError while the requested forward speed stays constant; the
      // old code only updated steering while speed itself was ramping.
      applyMotorPwm(nextPwm);
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
      (alertTime % 700UL) < 100UL;

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
  bool transitionBraking
) {
  unsigned long now = millis();

  if (now - lastLcdUpdateAt < 100) {
    return;
  }

  lastLcdUpdateAt = now;

  char top[17];
  char bottom[17];

  if (motorDirectionFault) {
    strcpy(top, "TURN SIGN FAULT");
    strcpy(bottom, "CHECK BANKS/GYRO");
  }
  else if (!localSensors.imuHealthy()) {
    strcpy(top, "IMU CHECK FAIL");
    strcpy(bottom, "RESET WHEN STILL");
  }
  else if (!visionCommandsOnline()) {
    strcpy(top, "VISION OFFLINE");
    strcpy(bottom, "MOTORS STOPPED");
  }
  else if (visionEmergencyStop) {
    strcpy(top, "VISION STOP");
    strcpy(bottom, "MOTORS STOPPED");
  }
  else if (recoveryMode != RECOVERY_NONE && trackGuidanceAvailable()) {
    strcpy(top, "TRACK RECOVERY");
    strcpy(bottom, localSensors.lineValid() ? "IR + GYRO" : "CAMERA + GYRO");
  }
  else if (recoveryLimitReached) {
    strcpy(top, "RECOVERY LIMIT");
    strcpy(bottom, "MOTORS STOPPED");
  }
  else if (cameraLeading && trackGuidanceAvailable()) {
    strcpy(top, "CAMERA TRACK");
    strcpy(bottom, "IR OPTIONAL");
  }
  else if (!trackSourcesConfirmed() && !strongCameraTrack()) {
    strcpy(top, "TAPE LOST");
    strcpy(bottom, "MOTORS STOPPED");
  }
  else if (!trackArmed) {
    strcpy(top, "CHECKING TRACK");
    strcpy(bottom, "MOTORS STOPPED");
  }
  else if (selfCentering) {
    strcpy(top, "SELF CENTERING");
    strcpy(bottom, "CAMERA + IR");
  }
  else if (!sensorReady) {
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
  else if (!targetDetected) {
    if (measuredDistance >= 999) {
      strcpy(top, "D:--cm  CRUISE");
    }
    else {
      snprintf(
        top,
        sizeof(top),
        "D:%2ldcm CRUISE",
        measuredDistance
      );
    }

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
  localSensors.beginLineArray();
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(4000, true);
#endif
  Serial.println(F("SENSOR_BOOT - KEEP CAR UPRIGHT AND STILL"));

  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("     M A V P");

  lcd.setCursor(0, 1);
  lcd.print("DIRECTION ACC");

  delay(1500);
  lcd.clear();

  bool imuReady = localSensors.beginImu();
  // Discard serial bytes received before calibration/readiness. No motion
  // command is accepted until this setup has completed.
  while (Serial.available() > 0) Serial.read();
  if (imuReady) {
    Serial.println(F("SENSOR_FUSION_READY V6 A_RIGHT ARC_GUARD"));
    Serial.println(F("ARDUINO_READY"));
  } else {
    Serial.println(F("SENSOR_FAULT: MPU check failed"));
  }
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  unsigned long now = millis();

  serviceVisionSerial();
  localSensors.update();
  updateTrackAgreement();

  bool newMeasurement =
    updateDistanceSensor();

  bool targetDetected =
    sensorReady &&
    measuredDistance < 999 &&
    measuredDistance <= DETECTION_LIMIT_CM;

  targetCurrentlyDetected = targetDetected;
  updateFusedSteering();
  publishSensorTelemetry();

  bool newTarget =
    targetDetected &&
    !targetWasDetected;

  if (sensorReady && lastRawDistance <= BRAKE_DISTANCE_CM &&
      activeDirection > 0 && currentPwm > 0) {
    forceVisionSafetyStop();
    enterObservationMode();
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

  // Stable live camera can acquire drive without IR. Weak/remembered guidance
  // stays bounded; STOP, wrong-way yaw and sensor faults remain unconditional.
  if (
    !visionCommandsOnline() ||
    visionEmergencyStop ||
    !localSensors.imuHealthy() ||
    !trackGuidanceAvailable()
  ) {
    forceVisionSafetyStop();
    updateAlerts(false, false);
    updateCluster(
      targetDetected,
      false,
      false
    );

    targetWasDetected = targetDetected;
    return;
  }

  if (recoveryMode != RECOVERY_NONE) {
    // Do not let normal cruise or an ACC reverse request overrule recovery.
    // A nearer/raw obstacle cuts both outputs at the final guard as well.
    setDriveRequest(1, FORWARD_MIN_PWM);
    serviceMotors();
    updateAlerts(false, false);
    updateCluster(targetDetected, false, false);
    targetWasDetected = targetDetected;
    return;
  }

  bool motorsStopped =
    activeDirection == 0 &&
    currentPwm == 0;

  bool motorsMovingReverse =
    activeDirection < 0 &&
    currentPwm > 0;

  bool stationaryBraking = false;

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

    setDriveRequest(1, CRUISE_PWM);
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

      else if (targetTrend == TREND_CLOSING) {
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
    else if (targetTrend == TREND_CLOSING) {
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
      reverseRanLongEnough &&
      gapSafelyOpening
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

  // Reduce translation near observed bends without lowering the proven
  // running floor. Sharp forward pivots further reduce forward travel.
  if (requestedDirection > 0 &&
      (selfCentering || cameraBend >= 55 || sharpCornerMode || localSensors.blackCount() >= 4)) {
    requestedPwm = min(requestedPwm, FORWARD_MIN_PWM);
  }

  // Move more cautiously while the followed object blocks the track.  The
  // ultrasonic controller still brakes, stops, and reverses as required.
  if (
    targetDetected &&
    !visionTapeVisible &&
    requestedDirection > 0
  ) {
    requestedPwm = min(
      requestedPwm,
      OCCLUDED_TRACK_MAX_PWM
    );
  }

  serviceMotors();

  bool transitionBraking =
    drivePhase == DRIVE_DIRECTION_WAIT &&
    lastStoppedDirection > 0 &&
    requestedDirection < 0;

  bool displayBraking =
    stationaryBraking ||
    transitionBraking;

  bool reversing =
    activeDirection < 0 &&
    currentPwm > 0 &&
    !displayBraking;

  updateAlerts(
    displayBraking,
    reversing
  );

  updateCluster(
    targetDetected,
    stationaryBraking,
    transitionBraking
  );

  targetWasDetected = targetDetected;
}

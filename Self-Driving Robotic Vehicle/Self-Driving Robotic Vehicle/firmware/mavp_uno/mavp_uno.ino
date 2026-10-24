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
  // A corner leg shows as 3-4 contiguous black channels with exactly one edge
  // channel lit: -1 = leg on the left, +1 = on the right, 0 = not a corner.
  int8_t cornerSide() const {
    if (!validLine || activeCount < 3) return 0;
    bool left = lineMask & 1, right = lineMask & 16;
    return left == right ? 0 : (left ? -1 : 1);
  }
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
  MAVP Stage 8 — combined ACC + Raspberry Pi vision steering  (FUSION V7)

  Stage 6 remains responsible for longitudinal safety, distance following,
  braking and reverse. The matching Raspberry Pi supplies structured GUIDE5,
  TRACK_LOST or STOP over USB serial. Missing vision commands stop both motors.
  Camera = what is AHEAD (corner direction/angle, remembered 4 s). IR array =
  what is UNDER the car now (centring, when a corner has arrived). Gyro =
  actual rotation (yaw-rate control, turn angle, wrong-direction latch).
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

// Motor sides: bank A (ENA/IN1/IN2, L298N OUT1/OUT2) = LEFT, bank B (ENB/IN3/IN4,
// OUT3/OUT4) = RIGHT. Source: the saved wiring record (README_FIRST, 2026-09-12) and a
// look at the wiring on 2026-10-21 (Motor B terminals feed the right-side motors).
// Both banks keep their proven forward/reverse polarity. Camera/IR coordinates are not
// inverted. Bench check: tape on the car's RIGHT => LEFT wheels faster. If that is ever
// reversed, flip this one line (true <-> false) and re-flash.
const bool MOTOR_A_IS_LEFT = true;
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
// One-source travel budgets are measured from the last time camera+IR (or a
// strong camera) confirmed the track. Only a fresh strong confirmation renews
// them. IR alone may carry the car further because it is a live reading of the
// tape under the bar; a weak camera alone may not.
const unsigned long CAM_ONLY_MS = 1200;
// V7.6: the IR bar is support only (see patch_fw76.py). With 4.7 cm tape 3-4 channels are lit whenever the car is a
// little off-centre, which the bar cannot tell from a corner leg.
const bool IR_STARTS_TURNS = false;     // the bar alone never starts a corner
const bool IR_LEADS_STEERING = false;   // the camera steers when it sees the tape; the bar steers when it does not
const unsigned long GRACE_MS = 400;   // V7.5: a dropout of BOTH sources shorter than this does not stop the car
const unsigned long IR_ONLY_MS = 3500;
const float ONE_SOURCE_MAX_YAW_DEG = 75.0f;
const int RECOVERY_CLEARANCE_CM = 30;
const int TURN_CLEARANCE_CM = 20;   // a pivot barely translates

// Corner turn: the camera sees a bend early and usually loses it before the
// wheels reach it. The direction is remembered here; the turn starts when the
// IR bar (underneath, "now") meets the corner, runs on the gyro, and ends when
// the camera/IR find the exit leg or a gyro budget is used up.
const float TURN_RATE_DPS = 60.0f;
const byte TURN_PACE_PCT = 60;   // arc turn only (CORNER_SPIN false)
// Corner spin: during a corner turn the INSIDE bank runs in REVERSE while the
// outside bank runs forward, so the car rotates about its own centre instead of
// sweeping 10+ cm past the corner. Each bank still only uses its already proven
// forward/reverse drive; PWM is zero while any bank changes direction.
// Set false to fall back to a forward-only arc.
const bool CORNER_SPIN = true;
const unsigned long SPIN_BRAKE_MS = 80;
const unsigned long SPIN_GAP_MS = 60;
const int SPIN_PWM = 235;
// The Pi's measured angle bounds the turn: stop at about that angle, not at a fixed 100-140 deg.
const float TURN_BUDGET_MIN_DEG = 40.0f;
const float TURN_BUDGET_MAX_DEG = 120.0f;
const float TURN_NO_ESTIMATE_DEG = 90.0f;
// TRACK LAYOUT PRIOR: a fixed constant, NOT measured. The demo track's corner leg is about 45 deg.
// If the Pi's measured angle is roughly that (PRIOR_MIN..PRIOR_MAX) the turn aims for TRACK_LEG_DEG,
// counted from the heading of the last clear straight view, and still ends earlier when the camera
// or IR bar finds the leg. Any other angle (e.g. 90) uses the Pi's measurement exactly as before.
const unsigned long POST_TURN_LOCK_MS = 1600;  // no new corner this soon after a turn (a 30 cm side lasts ~2 s)
const int TRACK_LEG_DEG = 45;
const byte PRIOR_MIN_DEG = 20;   // V7.5: the Pi reads a 45 deg corner as anything from ~25 to ~65 deg depending on camera tilt
const byte PRIOR_MAX_DEG = 80;
// After a turn that ended on the gyro (leg not seen), drive straight this long and let the
// camera/IR take over if they find the tape.
const unsigned long HOLD_AFTER_TURN_MS = 3000;
const unsigned long TURN_MAX_MS = 5000;
const unsigned long PENDING_MS = 4000;
// IR-bar-to-wheel-midpoint distance divided by crawl speed. NOT MEASURED YET:
// 0 starts the turn as soon as the bar meets the corner (can cut the corner
// if the bar is well ahead of the wheels). See the README calibration list.
const unsigned long IR_TO_PIVOT_DELAY_MS = 0;
// The Pi sends phase 3 when the corner reaches the bottom of the camera picture (about to
// vanish into the blind zone). The wheels are still ~30 cm short of it. Pivot after this long
// (= that distance / crawl speed). TUNING KNOB: pivots too early -> raise it; too late -> lower it.
const unsigned long PHASE3_TO_PIVOT_MS = 1200;
// An IR dropout may start the pivot earlier, but never sooner than this after phase 3.
const unsigned long PHASE3_IR_MIN_MS = 600;
const unsigned long PHASE3_STALE_MS = 7000;
unsigned long phase3At = 0;
float turnRefStart = 0;   // refYaw when the current pivot began

// Reverse still retraces the last forward arc with this PWM differential.
const int STEERING_INNER_MIN_PWM = FORWARD_MIN_PWM;
const int MAX_STEERING_REDUCTION = 55;
const int MAX_STEERING_BOOST = 25;
const int STEERING_FULL_SCALE_ERROR = 65;
const int STEERING_DEADBAND = 3;
const int OCCLUDED_TRACK_MAX_PWM = 210;

char visionCommand[48];
byte visionCommandLength = 0;

int cameraNear = 0;      // GUIDE5 near bearing; + = tape right of the car
int cameraTurnDeg = 0;   // signed estimate of a FUTURE corner, 0 = none/unknown
byte cameraQuality = 0;
byte cameraBend = 0;
byte cameraPace = 100;
byte cameraPhase = 0;
int motorSteeringError = 0;
int rememberedPathError = 0;
// V7.6c: backing off (ACC reverse) holds the heading the car had when the reverse began (gyro) and is time-limited.
float reverseYawRef = 0;
unsigned long reverseUsedMs = 0;
unsigned long reverseTickAt = 0;
float reverseTrim = 0;                             // PWM: learns the weaker bank while backing off; kept between back-offs
unsigned long reverseHoldAt = 0;
const float REVERSE_HOLD_KP = 8.0f;                // PWM per degree the nose has swung away
const float REVERSE_HOLD_KI = 25.0f;               // PWM per degree-second
const float REVERSE_HOLD_MAX = 45.0f;              // largest left/right difference
const unsigned long MAX_REVERSE_TOTAL_MS = 2500;   // total back-off until the car has driven forward again
const int lineCenterTrim = 0; // Geometric middle; never learn an arbitrary off-centre startup.
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
bool motorDirectionFault = false; // Latches until reset; never auto-flip moving motors.
unsigned long wrongYawSince = 0;
unsigned long effortSignSince = 0;
int lastLeftPwm = 0;
int lastRightPwm = 0;
bool discardingVisionFrame = false;
unsigned long lastTelemetryAt = 0;
unsigned long lastVisionCommandAt = 0;

bool visionEverConnected = false;
bool visionTapeVisible = false;
bool visionEmergencyStop = true;
unsigned long visionSequence = 0;
unsigned long alignedSequence = 0;
unsigned long alignmentStartedAt = 0;
bool alignmentTiming = false;
byte alignedFrames = 0;

enum AuthMode : byte { AUTH_NONE, AUTH_BOTH, AUTH_CAM, AUTH_CAMWEAK, AUTH_IR, AUTH_TURN };
AuthMode authMode = AUTH_NONE;
bool trackArmed = false;
bool cameraLeading = false;
bool limitReached = false;
unsigned long lastGoodAt = 0;
float goodYaw = 0;

// Remembered approaching corner (from a LIVE camera packet) and the turn itself.
int8_t pendSign = 0;
byte pendDeg = 0;
unsigned long pendAt = 0;
float pendYaw = 0;
float refYaw = 0;
unsigned long refAt = 0;
unsigned long turnEndAt = 0;
unsigned long irCenteredAt = 0;
unsigned long irSeenAt = 0;
unsigned long irBlankSince = 0;
bool turning = false;
bool spinPinsReversed = false;
int8_t spinSign = 0;
unsigned long spinGapUntil = 0;
unsigned long spinLeaveAt = 0;
float spinPulse = 0;
int8_t turnSign = 0;
byte turnEst = 0;
bool legPrior = false;
float camExitDeg = 20.0f;
float irExitDeg = 70.0f;
float turnStartYaw = 0;
float turnBudget = 0;
unsigned long turnStartAt = 0;
unsigned long turnHoldUntil = 0;
unsigned long handoverUntil = 0;
unsigned long holdUntil = 0;
unsigned long irCenterAt = 0;     // set when the CENTRE IR channel has just LEFT the tape
unsigned long irCenterOn = 0;     // last time the centre channel was on tape
byte stopWhy = 0;                 // 1 obstacle, 2 turned too far, 3 sources gone, 4 turn: obstacle, 5 turn: angle, 6 turn: time

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

float floatAbs(float value) { return value < 0 ? -value : value; }

// IR under the car: "now". Camera: ahead. Both are usable on their own.
bool irGood() { return localSensors.lineValid() && localSensors.lineConfidence() >= 70; }
bool camGood() { return visionTapeVisible && cameraQuality >= 50; }
// Just after a corner a plain >=70 camera is enough to carry on briefly.
bool camStrong() {
  return visionTapeVisible && abs(cameraNear) <= 60 &&
    (cameraQuality >= 85 || (millis() < handoverUntil && cameraQuality >= 70));
}

bool obstacleClear(int centimetres) {
  // Same no-echo convention as the existing ACC; no claim of rear protection.
  return sensorReady && millis() - lastSensorReadingAt <= 200 &&
    measuredDistance > centimetres && lastRawDistance > centimetres;
}

bool limitedGuidance() {
  return authMode == AUTH_IR || authMode == AUTH_CAMWEAK || authMode == AUTH_TURN;
}

bool trackGuidanceAvailable() {
  if (!trackArmed || authMode == AUTH_NONE || !visionCommandsOnline() || visionEmergencyStop ||
      !localSensors.imuHealthy() || motorDirectionFault) return false;
  if (limitedGuidance() && (activeDirection < 0 || requestedDirection < 0)) return false;
  return true;
}

bool pendingValid() {
  return pendSign != 0 && millis() - pendAt <= PENDING_MS &&
    floatAbs(localSensors.rightAngle() - pendYaw) <= 45.0f;
}

// The Pi said the corner is in the camera's blind zone: keep driving straight until the wheels
// reach it (checkCornerTrigger pivots), instead of stopping because both sources went quiet.
bool blindApproach() {
  return phase3At != 0 && pendingValid() && millis() - phase3At <= PHASE3_TO_PIVOT_MS + 400;
}

void resetTrackConfirmation() {
  alignmentTiming = false;
  alignedFrames = 0;
  alignedSequence = visionSequence;
}

void disarmTrack() {
  trackArmed = false;
  cameraLeading = false;
  turning = false;
  authMode = AUTH_NONE;
  pendSign = 0;
  phase3At = 0;
  travelPulseAccumulator = innerPulseAccumulator = 0;
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

byte travelPaceLimit() {
  byte pace = cameraPace;
  if (turning) return CORNER_SPIN ? 100 : TURN_PACE_PCT; // fixed: the Pi may be blind here
  if (limitedGuidance()) return min(pace, (byte)45);
  return cameraLeading ? min(pace, (byte)60) : pace;
}

unsigned long budgetRemainingMs() {
  unsigned long now = millis();
  if (turning) return now - turnStartAt < TURN_MAX_MS ? TURN_MAX_MS - (now - turnStartAt) : 0;
  unsigned long limit = authMode == AUTH_IR ? IR_ONLY_MS : authMode == AUTH_CAMWEAK ? CAM_ONLY_MS : 0;
  unsigned long age = now - lastGoodAt;
  return limit > age ? limit - age : 0;
}

// The yaw servo turns a requested chassis rate into a differential effort.
// Sign convention everywhere: + = nose RIGHT (clockwise), image right, IR right.
void updateYawServo(float dt, float wanted) {
  float previousDesired = desiredYawRate;
  float rateLimit = turning ? TURN_RATE_DPS : millis() < handoverUntil ? 45.0f : 28.0f;
  desiredYawRate = constrain(wanted, -rateLimit, rateLimit);
  if (desiredYawRate * previousDesired < 0) {
    yawServoIntegral = 0;
    yawEffort = 0;
  } else if (desiredYawRate == 0 && floatAbs(previousDesired) > 5) {
    yawServoIntegral = 0;
    yawEffort *= 0.5f;
  }
  float measured = localSensors.rightRate();
  bool moving = activeDirection > 0 && requestedDirection > 0 &&
    drivePhase == DRIVE_RUNNING && currentPwm >= FORWARD_MIN_PWM;
  // A reversed motor bank or gyro must not produce positive feedback. Latch a
  // stop only after the SAME significant effort has been held for 250 ms and
  // the chassis has turned the OTHER way for 400 ms; a swing that is still
  // decaying after a command reversal is not a wiring fault.
  static int8_t lastEffortSign = 0;
  int8_t effortSign = yawEffort > 0.3f ? 1 : yawEffort < -0.3f ? -1 : 0;
  if (effortSign != lastEffortSign) {
    lastEffortSign = effortSign;
    effortSignSince = millis();
    wrongYawSince = 0;
  }
  if (moving && effortSign != 0 && millis() - effortSignSince >= 250 &&
      ((effortSign > 0 && measured < -8) || (effortSign < 0 && measured > 8))) {
    if (!wrongYawSince) wrongYawSince = millis();
    if (millis() - wrongYawSince >= 400) motorDirectionFault = true;
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
  float limit = turning ? 1.0f : millis() < handoverUntil ? 0.85f : 0.65f;
  float target = constrain(desiredYawRate / gain + 0.003f * error + yawServoIntegral, -limit, limit);
  // Never keep turning the wrong way because of a stale integral.
  if ((target < 0 && desiredYawRate > 0) || (target > 0 && desiredYawRate < 0)) target = 0;
  float change = max(dt, 0.01f) * 5.0f;
  yawEffort += constrain(target - yawEffort, -change, change);

  // Bounded online actuator identification, not a trained ML model: only steady
  // command with real matching yaw while moving forward.
  if (floatAbs(yawEffort - previousYawEffort) > 0.035f) yawSteadySince = millis();
  if (moving && authMode != AUTH_NONE && millis() - yawSteadySince >= 150 &&
      floatAbs(yawEffort) >= 0.15f && floatAbs(measured) >= 8 && floatAbs(measured) <= 200 &&
      measured * yawEffort > 0) {
    float observedGain = constrain(measured / (yawEffort * pace), 35.0f, 300.0f);
    if (yawEffort > 0) turnGainRight += 0.025f * (observedGain - turnGainRight);
    else turnGainLeft += 0.025f * (observedGain - turnGainLeft);
  }
  previousYawEffort = yawEffort;
}

void beginTurn(int8_t sign) {
  turnRefStart = refYaw;
  turnEst = (pendingValid() && pendSign == sign) ? pendDeg : 0;
  // The estimate only bounds the turn; the turn ENDS when the exit leg is found.
  // The Pi reads a ~45 deg leg as 55-65 deg (about 1.3x), so a small estimate is scaled down; the 90 deg
  // 'leg runs off one side' cue is exact and keeps +5. Under-turning is the cheaper error: the line
  // follower closes a few degrees, but an over-turn throws the leg out of the camera view.
  legPrior = turnEst >= PRIOR_MIN_DEG && turnEst <= PRIOR_MAX_DEG;
  // The car often leans into the corner before the pivot starts, so the 45 deg is counted from the heading
  // of the last clear straight view (refYaw), not from where the pivot starts.
  int lean = (int)(localSensors.rightAngle() - refYaw) * sign;
  lean = constrain(lean, -10, 33);
  if (legPrior) turnEst = TRACK_LEG_DEG - lean;
  float eighty = turnEst * (legPrior ? 0.9f : 0.8f);   // V7.5: with the 45 deg prior the target heading is known, so end the pivot closer to it
  camExitDeg = legPrior ? eighty : turnEst >= 45 ? max(40.0f, eighty) : 20.0f;
  irExitDeg = legPrior ? eighty : max(70.0f, turnEst * 0.85f);
  turnBudget = legPrior ? (float)turnEst : turnEst ? constrain(turnEst <= 75 ? eighty + 2.0f : turnEst + 5.0f, TURN_BUDGET_MIN_DEG, TURN_BUDGET_MAX_DEG) : TURN_NO_ESTIMATE_DEG;
  stopWhy = 0;
  holdUntil = 0;
  phase3At = 0;
  turning = true;
  turnSign = sign;
  authMode = AUTH_TURN;
  pendSign = 0;
  turnHoldUntil = millis() + IR_TO_PIVOT_DELAY_MS;
  turnStartYaw = localSensors.rightAngle();
  turnStartAt = millis();
  wrongYawSince = 0;
  handoverUntil = 0;
  lastFusionAt = 0;
  resetTrackConfirmation();
}

// Corner arrival: the Pi says the corner reached the blind zone (phase 3); the IR bar or a
// timer then say when the wheels are there. An IR dropout alone is NOT a corner.
void checkCornerTrigger() {
  if (turning || millis() < handoverUntil || (turnEndAt && refAt < turnEndAt && millis() - turnEndAt < POST_TURN_LOCK_MS) || !(activeDirection > 0 && requestedDirection > 0 &&
      currentPwm > 0 && drivePhase == DRIVE_RUNNING)) return;
  unsigned long now = millis();
  bool arrived = phase3At != 0 && now - phase3At <= PHASE3_STALE_MS;
  unsigned long age = arrived ? now - phase3At : 0;
  int8_t side = IR_STARTS_TURNS ? localSensors.cornerSide() : 0;
  if (side != 0) {
    // A leg runs along the bar on one side. The camera's remembered direction
    // must agree; with no memory the tape must have been centred just before.
    if (pendingValid() ? pendSign == side : now - irCenteredAt <= 600) beginTurn(side);
  } else if (arrived && age >= PHASE3_IR_MIN_MS && localSensors.mask() == 0 && irBlankSince &&
             now - irBlankSince >= 60 && pendingValid() && now - irSeenAt <= 1500) {
    beginTurn(pendSign); // The approach tape ended under the bar.
  } else if (arrived && age >= PHASE3_IR_MIN_MS && pendingValid() && cameraQuality < 25 && irCenterAt &&
             now - irCenterAt >= 60 && now - irCenterAt <= 1500) {
    beginTurn(pendSign); // Corner in the blind spot and the bar's centre channel just left the tape.
  } else if (arrived && age >= PHASE3_TO_PIVOT_MS && pendingValid()) {
    beginTurn(pendSign); // Dead-reckoned: the wheels have had time to reach the corner.
  }
}

// After a pivot the next corner's lean is counted from THIS leg. After a 45 deg corner the leg heading is the
// heading before the corner + 45; the pivot usually ends a little earlier (camera/IR already see the leg), so take
// the expected heading but never more than 15 deg away from the measured one.
void takeLegReference() {
  float yaw = localSensors.rightAngle();
  float expected = turnRefStart + turnSign * (float)TRACK_LEG_DEG;
  refYaw = !legPrior ? yaw : expected < yaw - 15.0f ? yaw - 15.0f : expected > yaw + 15.0f ? yaw + 15.0f : expected;
}

void serviceTurn(unsigned long now, bool ir) {
  if (now < turnHoldUntil) {
    turnStartYaw = localSensors.rightAngle();
    turnStartAt = now;
    return;
  }
  float executed = (localSensors.rightAngle() - turnStartYaw) * turnSign;
  unsigned long elapsed = now - turnStartAt;
  if (executed < -12.0f && elapsed > 200) {
    motorDirectionFault = true; // Commanded one way, rotated the other.
    disarmTrack();
    return;
  }
  bool clear = obstacleClear(TURN_CLEARANCE_CM);
  // Tape ahead on the turn side (or centred) is the exit leg; tape on the other
  // side is the approach leg leaving the picture.
  bool cameraExit = executed >= camExitDeg && visionTapeVisible && cameraQuality >= 70 &&
    abs(cameraNear) <= 70 &&
    (cameraNear * turnSign >= -15 || executed >= max(60.0f, turnEst * 0.6f));
  bool irExit = ir && localSensors.lineConfidence() >= 80 && abs(localSensors.lineError()) <= 25 &&
    executed >= irExitDeg && (!visionTapeVisible || cameraQuality < 35);   // V7.6: the bar ends a turn only while the camera is blind
  // A thin diagonal leg seldom reaches quality 70 while the car is still rotating. Accept a
  // weaker, roughly centred view once most of the angle is done.
  bool cameraSoft = executed >= (legPrior ? turnEst * 0.85f : max(35.0f, turnEst * 0.7f)) && visionTapeVisible && cameraQuality >= 35 &&
    abs(cameraNear) <= 45 && cameraNear * turnSign >= -15;
  if (clear && (cameraExit || irExit || cameraSoft)) {
    turning = false;
    pendSign = 0;
    authMode = AUTH_CAMWEAK;
    turnEndAt = now;
    lastGoodAt = now;
    goodYaw = localSensors.rightAngle();
    takeLegReference();
    handoverUntil = now + 1500;
    lastFusionAt = 0;
    resetTrackConfirmation();
    return;
  }
  if (clear && executed >= turnBudget && elapsed < TURN_MAX_MS) {
    // Turned by the angle the Pi measured without seeing the leg: do not keep spinning and do
    // not stop. Go straight on the new heading; camera/IR take over as soon as they find tape.
    turning = false;
    pendSign = 0;
    authMode = AUTH_CAMWEAK;
    turnEndAt = now;
    lastGoodAt = now;
    goodYaw = localSensors.rightAngle();
    takeLegReference();
    handoverUntil = now + 1500;
    holdUntil = now + HOLD_AFTER_TURN_MS;
    rememberedPathError = 0;
    lastFusionAt = 0;
    resetTrackConfirmation();
    return;
  }
  if (!clear || executed >= turnBudget || elapsed >= TURN_MAX_MS) {
    stopWhy = !clear ? 4 : elapsed >= TURN_MAX_MS ? 6 : 5;
    limitReached = true;
    disarmTrack();
    return;
  }
  authMode = AUTH_TURN;
}

void updateTrackAgreement() {
  unsigned long now = millis();
  if (!visionCommandsOnline() || visionEmergencyStop || !localSensors.imuHealthy() || motorDirectionFault) {
    disarmTrack();
    limitReached = false;
    return;
  }
  bool ir = irGood();
  if (ir) {
    irSeenAt = now;
    if (abs(localSensors.lineError() - lineCenterTrim) <= 25) irCenteredAt = now;
  }
  if (localSensors.mask() != 0) irBlankSince = 0;
  else if (!irBlankSince) irBlankSince = now;
  if (localSensors.mask() & 4) { irCenterOn = now; irCenterAt = 0; }              // centre channel ON tape
  else if (!irCenterAt && irCenterOn && now - irCenterOn <= 150) irCenterAt = now; // it just left
  if (turning) {
    serviceTurn(now, ir);
    return;
  }
  // Corner announced, then the tape left the picture before phase 3: it is in the blind zone now.
  if (phase3At == 0 && pendingValid() && pendDeg >= 25 && cameraQuality < 25 && now - pendAt <= 2000) phase3At = now;
  bool cam = camGood();
  bool both = cam && (trackArmed ? localSensors.lineValid() : ir);
  bool strong = camStrong() && obstacleClear(RECOVERY_CLEARANCE_CM);
  if (both || strong) {
    if (!trackArmed) {
      // Startup/restart needs steady agreement, never manual centring.
      bool ready = both ? confirmedFor(250) : (confirmedFor(400) && alignedFrames >= 8);
      if (!ready) {
        authMode = AUTH_NONE;
        return;
      }
      trackArmed = true;
    }
    authMode = both ? AUTH_BOTH : AUTH_CAM;
    cameraLeading = !both;
    lastGoodAt = now;
    goodYaw = localSensors.rightAngle();
    limitReached = false;
    stopWhy = 0;
    holdUntil = 0;
    if (trackArmed) checkCornerTrigger();
    return;
  }
  if (trackArmed) {
    // One live source: bounded. IR is the tape under the bar; a weak camera
    // is only a brief bridge. Only a strong confirmation renews the budget.
    unsigned long age = now - lastGoodAt;
    float rotated = localSensors.rightAngle() - goodYaw;
    // With a corner announced by the Pi the car is about to pivot, not drive on: the pivot
    // itself needs only TURN_CLEARANCE_CM, so do not cancel the approach at 30 cm.
    bool clearAhead = obstacleClear(pendingValid() ? TURN_CLEARANCE_CM : RECOVERY_CLEARANCE_CM);
    bool turnedOk = rotated < ONE_SOURCE_MAX_YAW_DEG && rotated > -ONE_SOURCE_MAX_YAW_DEG;
    bool safe = clearAhead && turnedOk;
    bool irOnly = ir && age <= IR_ONLY_MS;
    bool camWeak = visionTapeVisible && cameraQuality >= 70 && localSensors.mask() == 0 && age <= CAM_ONLY_MS;
    bool holding = now < holdUntil;
    bool grace = age <= GRACE_MS;   // V7.5: both sources dropped for a moment: keep going straight, do not stop
    if (safe && (irOnly || camWeak || holding || blindApproach() || grace)) {
      authMode = irOnly ? AUTH_IR : AUTH_CAMWEAK;
      cameraLeading = false;
      checkCornerTrigger();
      return;
    }
    stopWhy = !clearAhead ? 1 : !turnedOk ? 2 : 3;
    limitReached = authMode == AUTH_IR || authMode == AUTH_CAMWEAK;
  }
  disarmTrack();
}

void updatePending() {
  if (cameraPhase >= 1 && cameraPhase <= 3 && cameraQuality >= 70 && abs(cameraTurnDeg) >= 15 &&
      abs(cameraNear) <= 45 && floatAbs(localSensors.rightRate()) <= 25) {
    pendSign = cameraTurnDeg > 0 ? 1 : -1;
    pendDeg = min(abs(cameraTurnDeg), 120);
    pendAt = millis();
    pendYaw = localSensors.rightAngle();
  } else if (cameraPhase == 0 && cameraQuality >= 85 && cameraBend < 20) {
    pendSign = 0; // A clear straight view cancels a remembered corner.
    phase3At = 0;
    refYaw = localSensors.rightAngle();
    refAt = millis();
  }
}

void updateFusedSteering() {
  if (!trackGuidanceAvailable()) {
    motorSteeringError = 0;
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
    // V7.6c: back off STRAIGHT: hold the heading the reverse began with (was: retrace the last forward arc, open loop).
    motorSteeringError = 0;
    return;
  }
  unsigned long now = millis();
  unsigned long elapsed = now - lastFusionAt;
  if (elapsed < 10 && lastFusionAt != 0) return;
  float dt = min(elapsed, 100UL) / 1000.0f;
  bool restartFilter = lastFusionAt == 0 || elapsed > 150;
  lastFusionAt = now;
  if (turning) {
    float executed = (localSensors.rightAngle() - turnStartYaw) * turnSign;
    float rate = now < turnHoldUntil ? 0.0f :
      TURN_RATE_DPS * ((turnEst && executed > 0.75f * turnEst) ? 0.6f : 1.0f);
    pathDerivative = 0;
    filteredSteering = 0;
    previousPathError = 0;
    motorSteeringError = (int)(turnSign * constrain(rate * 2.0f, 0.0f, 100.0f));
    updateYawServo(dt, turnSign * rate);
    return;
  }
  int localError = constrain(localSensors.lineError() - lineCenterTrim, -100, 100);
  bool ir = irGood();
  bool cam = camGood();
  int near = cam ? cameraNear : 0;
  float pathError = rememberedPathError;
  if (ir && cam && IR_LEADS_STEERING) {
    // IR is the current centre. Camera near adds the leading heading term so a
    // coarse five-bit array does not make the car swing; strong opposing IR
    // evidence keeps priority.
    // Right after a corner the bar can still sit over the corner's tape, so the
    // camera (which sees the new leg) leads for the hand-over window.
    pathError = millis() < handoverUntil ? near * 1.0f :
      ((long)localError * near < 0 && abs(localError) >= 50) ? localError :
      localError * 0.65f + near * 0.35f;
  } else if (ir && !cam) {
    pathError = localError;
  } else if (cam) {
    pathError = near * 1.2f;
  } else if (localSensors.lineValid()) {
    pathError = localError * 0.5f;
  }
  if (!ir && !cam && (blindApproach() || (authMode == AUTH_CAMWEAK && now - lastGoodAt <= GRACE_MS))) pathError = 0;
  int pathInt = constrain((int)pathError, -100, 100);
  if (authMode == AUTH_BOTH || authMode == AUTH_CAM) rememberedPathError = pathInt;
  if (restartFilter || dt <= 0) {
    pathDerivative = 0;
    filteredSteering = pathInt;
  } else {
    float derivative = constrain((pathInt - previousPathError) / dt, -300.0f, 300.0f);
    pathDerivative += (dt / (0.10f + dt)) * (derivative - pathDerivative);
    float target = constrain(pathInt * 1.12f + pathDerivative * 0.035f, -100.0f, 100.0f);
    float alpha = dt / (0.035f + dt);
    float maxChange = (abs(pathInt) >= 55 ? 1200.0f : 650.0f) * dt;
    filteredSteering += constrain(alpha * (target - filteredSteering), -maxChange, maxChange);
  }
  previousPathError = pathInt;
  motorSteeringError = constrain((int)filteredSteering, -100, 100);
  updateYawServo(dt, abs(motorSteeringError) <= 4 ? 0.0f : motorSteeringError * 0.55f);
}

// Signed integer parser for GUIDE5 fields (replaces libc strtol, which costs ~560 B of flash).
long parseLong(const char *s, char **end) {
  const char *p = s;
  bool neg = *p == '-';
  if (neg) p++;
  const char *d = p;
  long v = 0;
  while (*p >= '0' && *p <= '9' && p - d < 6) v = v * 10 + (*p++ - '0');
  *end = (char *)(p == d ? s : p);
  return neg ? -v : v;
}

void handleVisionCommand() {
  visionCommand[visionCommandLength] = '\0';
  bool valid = true;
  if (strncmp(visionCommand, "GUIDE5,", 7) == 0) {
    long values[6];
    const char *cursor = visionCommand + 7;
    for (byte i = 0; i < 6; i++) {
      char *tail;
      values[i] = parseLong(cursor, &tail);
      bool more = i < 5;
      if (tail == cursor || (more ? *tail != ',' : *tail != '\0')) { valid = false; break; }
      cursor = tail + (more ? 1 : 0);
      if (values[i] < (i == 1 ? -120 : i == 0 ? -100 : i == 4 ? 35 : 0) ||
          values[i] > (i == 1 ? 120 : i == 5 ? 3 : 100)) { valid = false; break; }
    }
    if (valid) {
      cameraNear = (int)values[0];
      cameraTurnDeg = (int)values[1];
      cameraQuality = (byte)values[2];
      cameraBend = (byte)values[3];
      cameraPace = (byte)values[4];
      cameraPhase = (byte)values[5];
      visionTapeVisible = cameraQuality >= 25;
      visionEmergencyStop = false;
      updatePending();
      if (cameraPhase == 3 && pendingValid() && phase3At == 0) phase3At = millis();
    }
  } else if (strcmp(visionCommand, "TRACK_LOST") == 0) {
    // Camera observation unavailable; bounded IR travel or a corner turn may
    // continue. STOP remains unconditional.
    visionTapeVisible = false;
    cameraQuality = 0;
    cameraBend = 0;
    visionEmergencyStop = false;
  } else if (strcmp(visionCommand, "STOP") == 0) {
    cameraNear = 0;
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
    cameraNear = 0;
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
  Serial.print(F(",CAM,")); Serial.print(cameraNear);
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
  Serial.print(F(",PIVOT,")); Serial.print(turning ? 1 : 0);
  Serial.print(F(",ERR,")); Serial.print(motorDirectionFault ? 6 : localSensors.imuFault());
  Serial.print(F(",ARM,")); Serial.print(trackArmed ? 1 : 0);
  Serial.print(F(",REC,")); Serial.print(budgetRemainingMs());
  Serial.print(F(",YR,")); Serial.print((int)desiredYawRate);
  Serial.print(F(",EFF,")); Serial.print((int)(yawEffort * 100));
  Serial.print(F(",D,")); Serial.print(measuredDistance > 999 ? 999 : measuredDistance < 0 ? 0 : measuredDistance);
  Serial.print(F(",W,")); Serial.print(stopWhy);
  Serial.print(F(",TS,"));
  Serial.print(turning ? (turnSign > 0 ? 4 : 3) : pendingValid() ? (pendSign > 0 ? 2 : 1) : 0);
  Serial.print(F(",SRC,"));
  if (!localSensors.imuHealthy() || motorDirectionFault) Serial.println(F("FAULT"));
  else if (!visionCommandsOnline()) Serial.println(F("OFF"));
  else if (visionEmergencyStop) Serial.println(F("STOP"));
  else if (limitReached) Serial.println(F("REC_STOP"));
  else if (authMode == AUTH_TURN) Serial.println(F("TURN"));
  else if (authMode == AUTH_IR) Serial.println(F("REC_IR"));
  else if (authMode == AUTH_CAMWEAK) Serial.println(F("REC_CAM"));
  else if (authMode == AUTH_CAM) Serial.println(F("CAM_TRACK"));
  else if (authMode == AUTH_BOTH) Serial.println(F("BOTH"));
  else if (!trackArmed && ((camGood() && irGood()) || camStrong())) Serial.println(F("CHECK"));
  else Serial.println(F("LOST"));
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

void setSpinPins(int8_t sign) {
  // sign > 0 = nose right: left bank forward, right bank reverse.
  bool leftForward = sign > 0, rightForward = sign < 0;
  bool aForward = MOTOR_A_IS_LEFT ? leftForward : rightForward;
  bool bForward = MOTOR_A_IS_LEFT ? rightForward : leftForward;
  digitalWrite(IN1, aForward ? LOW : HIGH);
  digitalWrite(IN2, aForward ? HIGH : LOW);
  digitalWrite(IN3, bForward ? LOW : HIGH);
  digitalWrite(IN4, bForward ? HIGH : LOW);
}

void applyMotorDirection(int direction) {
  spinPinsReversed = false;
  spinGapUntil = 0;
  spinLeaveAt = 0;
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

// V7.6c: PWM difference that keeps the car's heading while backing off. + = left bank faster, which swings the nose LEFT
// in reverse, so it answers a nose that has swung right (positive error).
int reverseHoldCorrection() {
  unsigned long now = millis();
  float dt = min(now - reverseHoldAt, 100UL) / 1000.0f;
  reverseHoldAt = now;
  float err = localSensors.rightAngle() - reverseYawRef;
  reverseTrim = constrain(reverseTrim + REVERSE_HOLD_KI * err * dt, -REVERSE_HOLD_MAX, REVERSE_HOLD_MAX);
  return (int)constrain(REVERSE_HOLD_KP * err + reverseTrim, -REVERSE_HOLD_MAX, REVERSE_HOLD_MAX);
}

void applyMotorPwm(int pwm) {
  pwm = constrain(pwm, 0, 255);
  // Final output guard also covers pending starts, reverse and ramp-down.
  if (pwm > 0 && (!visionCommandsOnline() || visionEmergencyStop ||
      !localSensors.imuHealthy() || !trackGuidanceAvailable())) pwm = 0;

  bool spinWanted = CORNER_SPIN && turning && pwm > 0 && activeDirection > 0 && requestedDirection > 0 &&
    drivePhase == DRIVE_RUNNING && trackGuidanceAvailable();
  if (spinWanted) {
    unsigned long now = millis();
    if (now - turnStartAt < SPIN_BRAKE_MS || now < spinGapUntil ||
        !spinPinsReversed || spinSign != turnSign) {
      // PWM is zero while the direction pins change.
      analogWrite(LEFT_PWM_PIN, 0); analogWrite(RIGHT_PWM_PIN, 0);
      lastLeftPwm = lastRightPwm = 0; currentPwm = pwm;
      if (now - turnStartAt >= SPIN_BRAKE_MS && now >= spinGapUntil &&
          (!spinPinsReversed || spinSign != turnSign)) {
        setSpinPins(turnSign);
        spinPinsReversed = true;
        spinSign = turnSign;
        spinGapUntil = now + 10;
      }
      return;
    }
    // Pulse density sets the spin rate; both banks share the same ON slots.
    spinLeaveAt = 0;
    spinPulse += constrain(floatAbs(yawEffort), 0.3f, 1.0f);
    bool on = spinPulse >= 1.0f;
    if (on) spinPulse -= 1.0f;
    int spinOut = on ? SPIN_PWM : 0;
    analogWrite(LEFT_PWM_PIN, spinOut); analogWrite(RIGHT_PWM_PIN, spinOut);
    lastLeftPwm = lastRightPwm = spinOut; currentPwm = pwm;
    return;
  }
  if (spinPinsReversed) {
    // Leaving the spin: stop, wait, put both banks forward, then use the normal
    // start kick (steering-aware) so the car restarts from rest reliably.
    analogWrite(LEFT_PWM_PIN, 0); analogWrite(RIGHT_PWM_PIN, 0);
    lastLeftPwm = lastRightPwm = 0; currentPwm = pwm;
    if (!spinLeaveAt) spinLeaveAt = millis();
    if (millis() - spinLeaveAt < SPIN_GAP_MS) return;
    applyMotorDirection(1); // also clears spinPinsReversed
    if (pwm > 0 && activeDirection > 0 && requestedDirection > 0 && drivePhase == DRIVE_RUNNING) {
      drivePhase = DRIVE_START_KICK;
      drivePhaseStartedAt = millis();
      lastPwmRampAt = millis();
      pwm = max(pwm, FORWARD_START_PWM);
    }
  }

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

  // V7.6c: reversing = equal base PWM plus a gyro-hold correction. In reverse the LEFT bank running faster swings the nose
  // LEFT, so a positive motorSteeringError (nose swung right) = left faster.
  if (pwm > 0 && activeDirection < 0 && requestedDirection < 0 && drivePhase != DRIVE_RAMP_DOWN && trackGuidanceAvailable()) {
    int d = reverseHoldCorrection();
    leftPwm = constrain(pwm + 10 + d, REVERSE_MIN_PWM, 255);
    rightPwm = constrain(pwm + 10 - d, REVERSE_MIN_PWM, 255);
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
  if (spinPinsReversed) applyMotorDirection(1);

  requestedDirection = 0;
  requestedPwm = 0;
  activeDirection = 0;

  drivePhase = DRIVE_STOPPED;
  fullyStoppedAt = now;

  reverseRunTiming = false;
  turning = false;
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
  if (direction < 0) reverseYawRef = localSensors.rightAngle();   // V7.6c
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
  if (spinPinsReversed) applyMotorDirection(1); // PWM is zero: put both banks back to forward

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

// "D:%2ldcm TAIL" without the printf family: it cost about 1 KB of the Uno's 32 KB flash, and V7.5 needs the room.
void distText(char *out, size_t size, long cm, const char *tail) {
  char digits[8];
  byte n = 0;
  bool negative = cm < 0;
  unsigned long v = negative ? (unsigned long)(-cm) : (unsigned long)cm;
  do { digits[n++] = '0' + v % 10; v /= 10; } while (v && n < 6);
  if (negative) digits[n++] = '-';
  byte len = 0;
  out[len++] = 'D';
  out[len++] = ':';
  if (n < 2) out[len++] = ' ';          // %2ld pads to width 2
  while (n && len < size - 1) out[len++] = digits[--n];
  out[len++] = 'c';
  out[len++] = 'm';
  out[len++] = ' ';
  while (*tail && len < size - 1) out[len++] = *tail++;
  out[len] = 0;
}

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
  else if (authMode == AUTH_TURN && trackGuidanceAvailable()) {
    strcpy(top, "CORNER TURN");
    strcpy(bottom, "GYRO + CAMERA");
  }
  else if (limitedGuidance() && trackGuidanceAvailable()) {
    strcpy(top, "TRACK RECOVERY");
    strcpy(bottom, authMode == AUTH_IR ? "IR + GYRO" : "CAMERA + GYRO");
  }
  else if (limitReached) {
    strcpy(top, "RECOVERY LIMIT");
    strcpy(bottom, "MOTORS STOPPED");
  }
  else if (cameraLeading && trackGuidanceAvailable()) {
    strcpy(top, "CAMERA TRACK");
    strcpy(bottom, "IR OPTIONAL");
  }
  else if (!trackArmed) {
    strcpy(top, "CHECKING TRACK");
    strcpy(bottom, "MOTORS STOPPED");
  }
  else if (!sensorReady) {
    strcpy(top, "SENSOR CHECK");
    strcpy(bottom, "PLEASE WAIT");
  }
  else if (transitionBraking) {
    strcpy(top, "!!  BRAKING  !!");

    distText(bottom, sizeof(bottom), measuredDistance, "REV PREP");
  }
  else if (braking) {
    strcpy(top, "!!  BRAKING  !!");

    distText(bottom, sizeof(bottom), measuredDistance, "SPEED 0");
  }
  else if (!targetDetected) {
    if (measuredDistance >= 999) {
      strcpy(top, "D:--cm  CRUISE");
    }
    else {
      distText(top, sizeof(top), measuredDistance, "CRUISE");
    }

    makeSpeedBar(bottom, "FWD", currentPwm);
  }
  else {
    distText(top, sizeof(top), measuredDistance, "FOLLOW");

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
    Serial.print(F("SENSOR_FUSION_READY V7 A_"));
    Serial.print(MOTOR_A_IS_LEFT ? F("LEFT") : F("RIGHT"));
    Serial.println(F(" CORNER_TURN FW73 P45 C22 V76c"));
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

  if (limitedGuidance() && trackGuidanceAvailable()) {
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
  if (activeDirection > 0 && currentPwm > 0) reverseUsedMs = 0;   // V7.6c: the back-off allowance is renewed by driving forward

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

      else if (targetTrend == TREND_CLOSING && reverseUsedMs < MAX_REVERSE_TOTAL_MS) {
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
    else if (targetTrend == TREND_CLOSING && reverseUsedMs < MAX_REVERSE_TOTAL_MS) {
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
      reverseUsedMs += min(now - reverseTickAt, 100UL);
      reverseTickAt = now;
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
    else if (reverseUsedMs >= MAX_REVERSE_TOTAL_MS) {
      // V7.6c: nothing sees the tape behind the car: stop backing off and wait for the gap to open.
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
      (turning || cameraBend >= 55 || localSensors.blackCount() >= 4)) {
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

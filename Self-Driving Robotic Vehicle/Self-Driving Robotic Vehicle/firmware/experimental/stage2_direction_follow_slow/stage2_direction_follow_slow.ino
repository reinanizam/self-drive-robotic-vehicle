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
// DISTANCE CONTROL
// =====================================================

const int DETECTION_LIMIT_CM = 30;
const int RECHECK_DISTANCE_CM = 16;
const int BRAKE_DISTANCE_CM = 10;
const int FORWARD_RESTART_CM = 14;
const int REVERSE_RELEASE_CM = 18;

const int MOTION_THRESHOLD_CM = 2;
const int URGENT_CLOSING_SPEED_CM_S = 25;

// A single ultrasonic change is not enough to choose a direction.
// The target must produce two consecutive motion windows while
// the car is fully stopped. This rejects hand shake and echo jitter.
const byte DIRECTION_CONFIRM_WINDOWS = 2;

const unsigned long SENSOR_PERIOD_MS = 50;
const unsigned long MOTION_WINDOW_MS = 180;

// Reverse must run briefly before direction is checked again.
const unsigned long MIN_REVERSE_RUN_MS = 500;

// =====================================================
// PWM CONTROL
// =====================================================

// A short strong kick starts all four wheels together. The lower
// running PWM values below make the car calmer with the new batteries.
const int FORWARD_START_PWM = 255;
const int REVERSE_START_PWM = 220;

const int CRUISE_PWM = 210;

const int FOLLOW_INITIAL_PWM = 205;
const int FORWARD_MIN_PWM = 195;
const int FORWARD_MAX_PWM = 220;

const int REVERSE_MIN_PWM = 170;
const int REVERSE_MAX_PWM = 190;

const unsigned long FORWARD_KICK_MS = 240;
const unsigned long REVERSE_KICK_MS = 180;

const unsigned long FORWARD_TO_REVERSE_WAIT_MS = 450;
const unsigned long REVERSE_TO_FORWARD_WAIT_MS = 300;

const unsigned long PWM_RAMP_PERIOD_MS = 20;

const int RUN_RAMP_STEP = 3;
const int STOP_RAMP_STEP = 20;

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

// Direction evidence is collected only while the car is stopped.
unsigned long evaluatedTrendVersion = 0;
byte openingEvidence = 0;
byte closingEvidence = 0;

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

void resetDirectionEvidence() {
  openingEvidence = 0;
  closingEvidence = 0;
  evaluatedTrendVersion = trendVersion;
}

void updateDirectionEvidence() {
  // Count each completed motion window only once.
  if (evaluatedTrendVersion == trendVersion) {
    return;
  }

  evaluatedTrendVersion = trendVersion;

  if (targetTrend == TREND_OPENING) {
    if (openingEvidence < DIRECTION_CONFIRM_WINDOWS) {
      openingEvidence++;
    }

    closingEvidence = 0;
  }
  else if (targetTrend == TREND_CLOSING) {
    if (closingEvidence < DIRECTION_CONFIRM_WINDOWS) {
      closingEvidence++;
    }

    openingEvidence = 0;
  }
  else {
    // A stable window cancels unconfirmed movement.
    openingEvidence = 0;
    closingEvidence = 0;
  }
}

bool openingMotionConfirmed() {
  return openingEvidence >= DIRECTION_CONFIRM_WINDOWS;
}

bool closingMotionConfirmed() {
  return closingEvidence >= DIRECTION_CONFIRM_WINDOWS;
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
  resetDirectionEvidence();
  observationStarted = true;
}

void enterObservationMode() {
  followIntent = INTENT_OBSERVE;

  observationStarted = false;
  reverseRunTiming = false;

  resetTrendMeasurement();
  resetDirectionEvidence();
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

void applyMotorPwm(int pwm) {
  pwm = constrain(pwm, 0, 255);

  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);

  currentPwm = pwm;
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

  if (direction > 0) {
    applyMotorPwm(FORWARD_START_PWM);
  } else {
    applyMotorPwm(REVERSE_START_PWM);
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

  Serial.begin(9600);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("     M A V P");

  lcd.setCursor(0, 1);
  lcd.print("DIRECTION ACC");

  delay(1500);
  lcd.clear();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  unsigned long now = millis();

  bool newMeasurement =
    updateDistanceSensor();

  bool targetDetected =
    sensorReady &&
    measuredDistance < 999 &&
    measuredDistance <= DETECTION_LIMIT_CM;

  bool newTarget =
    targetDetected &&
    !targetWasDetected;

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

  // Only the stopped car is allowed to decide that the target
  // itself is moving. While driving, gap changes also contain
  // the car's own motion and cannot safely request reverse.
  if (
    motorsStopped &&
    targetDetected &&
    observationStarted &&
    (
      followIntent == INTENT_OBSERVE ||
      followIntent == INTENT_STOPPED
    )
  ) {
    updateDirectionEvidence();
  }

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
        openingMotionConfirmed() &&
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

      else if (closingMotionConfirmed()) {
        // Two stopped-car measurements confirm that
        // the target is reversing toward the car.
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
      openingMotionConfirmed() &&
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
    else if (closingMotionConfirmed()) {
      // Target begins reversing, confirmed twice while stopped.
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
    Serial.println(currentPwm);
  }

  targetWasDetected = targetDetected;
}


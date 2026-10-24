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
// DISTANCE ZONES
// =====================================================

// Look much farther ahead.
const int DETECTION_LIMIT_CM = 60;

// Begin slowing and preparing to stop here.
const int CLOSE_OBSERVE_CM = 18;

// A stopped target inside this distance means BRAKING.
const int BRAKE_DISTANCE_CM = 12;

// Target must move away past this distance
// before forward motion restarts from the close zone.
const int FORWARD_RESTART_CM = 15;

// Ignore movement smaller than 3 cm.
const int MOTION_THRESHOLD_CM = 3;

// =====================================================
// SENSOR TIMING
// =====================================================

const unsigned long SENSOR_PERIOD_MS = 60;
const unsigned long MOTION_WINDOW_MS = 240;

// =====================================================
// LOWER, SMOOTHER PWM
// =====================================================

// Required only to overcome starting friction.
const int START_PWM = 230;

// Normal and FOLLOW speeds.
const int CRUISE_PWM = 160;
const int FORWARD_MIN_PWM = 140;
const int FORWARD_MAX_PWM = 170;

// Reverse speeds.
const int REVERSE_MIN_PWM = 145;
const int REVERSE_MAX_PWM = 175;

// Short starting push.
const unsigned long START_KICK_MS = 220;

// Full stop before changing direction.
const unsigned long DIRECTION_PAUSE_MS = 250;

// Gradual motor transitions.
const unsigned long PWM_RAMP_PERIOD_MS = 20;
const int RUN_RAMP_STEP = 3;
const int STOP_RAMP_STEP = 12;

// =====================================================
// SENSOR STATE
// =====================================================

long sensorSamples[3] = {0, 0, 0};

byte sensorSampleIndex = 0;
byte sensorSampleCount = 0;
byte noEchoCount = 0;

long measuredDistance = 999;
long filteredDistance = -1;

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

bool trendTracking = false;
bool closeObservationStarted = false;

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
    constrain(pwm, 0, START_PWM),
    0,
    START_PWM,
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
// HC-SR04 SENSOR
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

  // Reject two temporary missing echoes.
  if (rawDistance >= 999) {
    if (noEchoCount < 3) {
      noEchoCount++;
    }

    if (noEchoCount >= 3) {
      measuredDistance = 999;
      filteredDistance = -1;

      sensorSampleCount = 0;
      sensorReady = true;
    }

    return true;
  }

  noEchoCount = 0;

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
  } else {
    filteredDistance =
      (filteredDistance + medianDistance * 2) / 3;
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
    targetTrend = TREND_OPENING;
  }
  else if (
    distanceChange <= -MOTION_THRESHOLD_CM
  ) {
    targetTrend = TREND_CLOSING;
  }
  else {
    targetTrend = TREND_STABLE;
    estimatedGapSpeedCmS = 0;
  }

  trendReferenceDistance = distance;
  trendReferenceTime = now;
}

// =====================================================
// PWM CALCULATIONS
// =====================================================

int calculateForwardPwm(
  long distance,
  TargetTrend trend,
  long gapSpeed
) {
  int pwm;

  if (distance >= 45) {
    pwm = CRUISE_PWM;
  } else {
    pwm = map(
      constrain(distance, CLOSE_OBSERVE_CM, 45),
      CLOSE_OBSERVE_CM,
      45,
      FORWARD_MIN_PWM,
      FORWARD_MAX_PWM
    );
  }

  // If the gap is already closing, begin reducing
  // speed earlier.
  if (trend == TREND_CLOSING) {
    int reduction =
      constrain((int)(gapSpeed / 2), 0, 15);

    pwm -= reduction;
  }

  return constrain(
    pwm,
    FORWARD_MIN_PWM,
    FORWARD_MAX_PWM
  );
}

int calculateReversePwm(
  long distance,
  long closingSpeed
) {
  int speedPwm = map(
    constrain(closingSpeed, 0, 40),
    0,
    40,
    REVERSE_MIN_PWM,
    REVERSE_MAX_PWM
  );

  int distanceBoost = map(
    constrain(
      CLOSE_OBSERVE_CM - distance,
      0,
      CLOSE_OBSERVE_CM
    ),
    0,
    CLOSE_OBSERVE_CM,
    0,
    10
  );

  return constrain(
    speedPwm + distanceBoost,
    REVERSE_MIN_PWM,
    REVERSE_MAX_PWM
  );
}

// =====================================================
// MOTOR OUTPUT
// =====================================================

void applyMotorDirection(int direction) {
  if (direction > 0) {
    // Confirmed forward polarity.
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else if (direction < 0) {
    // Confirmed reverse polarity.
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

void beginMotorStart(int direction) {
  applyMotorDirection(direction);

  // Short starting push.
  applyMotorPwm(START_PWM);

  activeDirection = direction;

  drivePhase = DRIVE_START_KICK;
  drivePhaseStartedAt = millis();
  lastPwmRampAt = millis();
}

void beginRampDown() {
  drivePhase = DRIVE_RAMP_DOWN;
  lastPwmRampAt = millis();
}

void serviceMotors() {
  unsigned long now = millis();

  // Gradual stopping before rest or reverse.
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
        activeDirection = 0;

        if (requestedDirection == 0) {
          drivePhase = DRIVE_STOPPED;
        } else {
          drivePhase = DRIVE_DIRECTION_WAIT;
          drivePhaseStartedAt = now;
        }
      }
    }

    return;
  }

  // Safe pause before reversing direction.
  if (drivePhase == DRIVE_DIRECTION_WAIT) {
    if (requestedDirection == 0) {
      drivePhase = DRIVE_STOPPED;
      return;
    }

    if (
      now - drivePhaseStartedAt >=
      DIRECTION_PAUSE_MS
    ) {
      beginMotorStart(requestedDirection);
    }

    return;
  }

  // Stop requested.
  if (requestedDirection == 0) {
    if (currentPwm > 0) {
      beginRampDown();
    } else {
      drivePhase = DRIVE_STOPPED;
      activeDirection = 0;
    }

    return;
  }

  // Start from rest.
  if (drivePhase == DRIVE_STOPPED) {
    beginMotorStart(requestedDirection);
    return;
  }

  // Change direction.
  if (activeDirection != requestedDirection) {
    beginRampDown();
    return;
  }

  // Complete starting push.
  if (drivePhase == DRIVE_START_KICK) {
    if (
      now - drivePhaseStartedAt >=
      START_KICK_MS
    ) {
      drivePhase = DRIVE_RUNNING;
      lastPwmRampAt = now;
    }

    return;
  }

  // Smoothly approach requested running PWM.
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

  // One brake alert with steady LED.
  if (activeAlert == ALERT_BRAKE) {
    digitalWrite(BRAKE_LED, HIGH);

    if (alertChanged) {
      tone(BUZZER, 800, 600);
    }
  }

  // Reverse warning and synchronized LED.
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
  bool braking
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
    makeSpeedBar(bottom, "SPD", currentPwm);
  }
  else {
    snprintf(
      top,
      sizeof(top),
      "D:%2ldcm FOLLOW",
      measuredDistance
    );

    if (requestedDirection > 0) {
      makeSpeedBar(bottom, "FWD", currentPwm);
    }
    else if (requestedDirection < 0) {
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
  lcd.print("SMOOTH FOLLOW");

  delay(1500);
  lcd.clear();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  bool newMeasurement =
    updateDistanceSensor();

  bool targetDetected =
    sensorReady &&
    measuredDistance < 999 &&
    measuredDistance <= DETECTION_LIMIT_CM;

  bool closeZone =
    targetDetected &&
    measuredDistance <= CLOSE_OBSERVE_CM;

  bool motorsMovingForward =
    activeDirection > 0 &&
    currentPwm > 0;

  bool motorsMovingReverse =
    activeDirection < 0 &&
    currentPwm > 0;

  if (newMeasurement) {
    if (!targetDetected) {
      resetTrendMeasurement();
      closeObservationStarted = false;
    }
    else if (!trendTracking) {
      beginTrendMeasurement(measuredDistance);
    }
    else {
      updateTargetTrend(measuredDistance);
    }
  }

  bool braking = false;

  // ===================================================
  // SENSOR NOT READY
  // ===================================================

  if (!sensorReady) {
    setDriveRequest(0, 0);
  }

  // ===================================================
  // CLEAR ROAD
  // ===================================================

  else if (!targetDetected) {
    closeObservationStarted = false;
    setDriveRequest(1, CRUISE_PWM);
  }

  // ===================================================
  // TARGET FARTHER THAN 18 CM
  // ===================================================

  else if (!closeZone) {
    closeObservationStarted = false;

    // If currently reversing, stop before forward drive.
    if (motorsMovingReverse) {
      setDriveRequest(0, 0);
    } else {
      int forwardPwm = calculateForwardPwm(
        measuredDistance,
        targetTrend,
        estimatedGapSpeedCmS
      );

      setDriveRequest(1, forwardPwm);
    }
  }

  // ===================================================
  // CLOSE TARGET: 18 CM OR LESS
  // ===================================================

  else {
    // If the car is moving forward, first perform
    // a gradual stop before deciding BRAKE or reverse.
    if (motorsMovingForward) {
      closeObservationStarted = false;
      setDriveRequest(0, 0);
    }

    // While reverse-following:
    // stable or closing gap means keep reversing.
    else if (motorsMovingReverse) {
      if (
        targetTrend == TREND_OPENING ||
        measuredDistance >= CLOSE_OBSERVE_CM
      ) {
        // Target slowed, stopped, or moved away:
        // stop and measure it again.
        closeObservationStarted = false;
        setDriveRequest(0, 0);
      } else {
        int reversePwm = calculateReversePwm(
          measuredDistance,
          estimatedGapSpeedCmS
        );

        setDriveRequest(-1, reversePwm);
      }
    }

    // Motors have stopped: observe true target movement.
    else {
      if (!closeObservationStarted) {
        beginTrendMeasurement(measuredDistance);
        closeObservationStarted = true;
        setDriveRequest(0, 0);
      }

      else if (targetTrend == TREND_CLOSING) {
        // Target continues approaching:
        // follow it smoothly in reverse.
        int reversePwm = calculateReversePwm(
          measuredDistance,
          estimatedGapSpeedCmS
        );

        setDriveRequest(-1, reversePwm);
      }

      else if (
        targetTrend == TREND_OPENING &&
        measuredDistance >= FORWARD_RESTART_CM
      ) {
        // Target moved away:
        // resume forward FOLLOW.
        setDriveRequest(1, FORWARD_MIN_PWM);
      }

      else {
        // Stable close target.
        setDriveRequest(0, 0);

        braking =
          measuredDistance <= BRAKE_DISTANCE_CM &&
          targetTrend == TREND_STABLE;
      }
    }
  }

  serviceMotors();

  bool reversing =
    requestedDirection < 0 &&
    !braking;

  updateAlerts(braking, reversing);

  updateCluster(
    targetDetected,
    braking
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

    Serial.print(" Dir:");
    Serial.print(requestedDirection);

    Serial.print(" PWM:");
    Serial.print(currentPwm);

    Serial.print(" Brake:");
    Serial.println(braking ? "YES" : "NO");
  }
}
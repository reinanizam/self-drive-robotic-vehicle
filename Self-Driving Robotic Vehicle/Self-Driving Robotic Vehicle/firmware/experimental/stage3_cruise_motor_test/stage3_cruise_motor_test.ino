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
// DISTANCE AND MOTION
// =====================================================

const int DETECTION_LIMIT_CM = 30;
const int BRAKE_DISTANCE_CM = 10;

const int MOTION_THRESHOLD_CM = 3;

const unsigned long SENSOR_PERIOD_MS = 60;
const unsigned long MOTION_WINDOW_MS = 240;

// =====================================================
// LOWER-POWER MOTOR SETTINGS
// =====================================================

const int START_PWM = 230;

const int MIN_RUN_PWM = 175;
const int MAX_RUN_PWM = 200;
const int CRUISE_PWM = 180;

const unsigned long START_KICK_MS = 220;

const unsigned long DIRECTION_PAUSE_MS = 220;

const unsigned long PWM_RAMP_PERIOD_MS = 20;
const int RUN_RAMP_STEP = 4;

// FIX 1: softer stop ramp -- was 30, halved to reduce the
// current transient that was browning out the Arduino
// right at the moment braking engaged.
const int STOP_RAMP_STEP = 15;

// FIX 3: the start kick used to jump straight from 0 to
// START_PWM in a single analogWrite call. That instant
// full-power step is the single biggest current spike in
// the whole sketch (stall current on top of a step input),
// and it lines up exactly with the "screen dies, board
// resets" symptom. We now ramp INTO the kick over the same
// START_KICK_MS window instead of slamming it, using the
// same PWM_RAMP_PERIOD_MS cadence as the run-phase ramp.
// This does not fix a weak power rail by itself -- see the
// hardware notes below -- but it removes the sharpest edge
// the software was directly responsible for.
const int START_RAMP_STEP = 21;

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
// TARGET MOVEMENT
// =====================================================

enum TargetMotion {
  MOTION_UNKNOWN,
  SPEED_ZERO,
  MOVING_AWAY,
  MOVING_TOWARD
};

TargetMotion targetMotion = MOTION_UNKNOWN;

long motionReferenceDistance = 0;
long estimatedSpeedCmS = 0;

unsigned long motionReferenceTime = 0;

bool trackingTarget = false;
bool wasInBrakeZone = false;

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

// FIX 2: stagger the brake alert so the buzzer/LED don't
// fire in the exact same instant as the motor ramp-down.
// That simultaneous spike (motor current collapsing +
// buzzer + LED all switching at once) was the other half
// of the brownout. Waiting ~150ms lets motor current settle.
unsigned long brakingDetectedAt = 0;
bool brakeAlertArmed = false;
const unsigned long ALERT_DELAY_MS = 150;

// =====================================================
// LCD STATE
// =====================================================

char previousTop[17] = "";
char previousBottom[17] = "";

unsigned long lastLcdUpdateAt = 0;

// =====================================================
// LCD FUNCTIONS
// =====================================================

void writeLcdLine(byte row, const char* text, char* previous) {
  char padded[17];
  for (byte i = 0; i < 16; i++) padded[i] = ' ';
  padded[16] = '\0';

  byte i = 0;
  while (text[i] != '\0' && i < 16) { padded[i] = text[i]; i++; }

  if (strcmp(padded, previous) == 0) return;

  lcd.setCursor(0, row);
  for (byte position = 0; position < 16; position++) {
    lcd.write((uint8_t)padded[position]);
  }
  strcpy(previous, padded);
}

void makeSpeedBar(char* output, const char* direction, int pwm) {
  byte blocks = map(constrain(pwm, 0, START_PWM), 0, START_PWM, 0, 8);

  for (byte i = 0; i < 16; i++) output[i] = ' ';
  output[16] = '\0';

  output[0] = direction[0];
  output[1] = direction[1];
  output[2] = direction[2];
  output[3] = ' ';
  output[4] = '[';

  for (byte i = 0; i < 8; i++) output[5 + i] = i < blocks ? char(255) : ' ';
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

  unsigned long duration = pulseIn(ECHO, HIGH, 25000UL);
  if (duration == 0) return 999;
  return duration / 58;
}

long medianOfThree(long a, long b, long c) {
  return max(min(a, b), min(max(a, b), c));
}

bool updateDistanceSensor() {
  unsigned long now = millis();
  if (now - lastSensorReadingAt < SENSOR_PERIOD_MS) return false;
  lastSensorReadingAt = now;

  long rawDistance = readRawDistanceCm();

  if (rawDistance >= 999) {
    if (noEchoCount < 3) noEchoCount++;
    if (noEchoCount >= 3) {
      measuredDistance = 999;
      filteredDistance = -1;
      sensorSampleCount = 0;
      sensorReady = true;
    }
    return true;
  }

  noEchoCount = 0;
  sensorSamples[sensorSampleIndex] = rawDistance;
  sensorSampleIndex = (sensorSampleIndex + 1) % 3;
  if (sensorSampleCount < 3) sensorSampleCount++;

  long stableReading;
  if (sensorSampleCount == 1) {
    stableReading = rawDistance;
  } else if (sensorSampleCount == 2) {
    stableReading = (sensorSamples[0] + sensorSamples[1]) / 2;
  } else {
    stableReading = medianOfThree(sensorSamples[0], sensorSamples[1], sensorSamples[2]);
  }

  if (filteredDistance < 0) filteredDistance = stableReading;
  else filteredDistance = (filteredDistance + stableReading * 2) / 3;

  measuredDistance = filteredDistance;
  sensorReady = true;
  return true;
}

// =====================================================
// TARGET MOVEMENT
// =====================================================

void beginMotionTracking(long distance) {
  trackingTarget = true;
  motionReferenceDistance = distance;
  motionReferenceTime = millis();
  estimatedSpeedCmS = 0;
  targetMotion = MOTION_UNKNOWN;
}

void resetMotionTracking() {
  trackingTarget = false;
  estimatedSpeedCmS = 0;
  targetMotion = MOTION_UNKNOWN;
}

void updateTargetMotion(long distance) {
  unsigned long now = millis();
  if (!trackingTarget) { beginMotionTracking(distance); return; }

  unsigned long elapsed = now - motionReferenceTime;
  if (elapsed < MOTION_WINDOW_MS) return;

  long distanceChange = distance - motionReferenceDistance;
  estimatedSpeedCmS = (abs(distanceChange) * 1000L) / elapsed;

  if (distanceChange >= MOTION_THRESHOLD_CM) {
    targetMotion = MOVING_AWAY;
  } else if (distanceChange <= -MOTION_THRESHOLD_CM) {
    targetMotion = MOVING_TOWARD;
  } else {
    targetMotion = SPEED_ZERO;
    estimatedSpeedCmS = 0;
  }

  motionReferenceDistance = distance;
  motionReferenceTime = now;
}

// =====================================================
// MOTOR OUTPUT
// =====================================================

void applyMotorDirection(int direction) {
  if (direction > 0) {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  } else if (direction < 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
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
  if (direction == 0) requestedPwm = 0;
  else requestedPwm = constrain(pwm, MIN_RUN_PWM, MAX_RUN_PWM);
}

void beginMotorStart(int direction) {
  applyMotorDirection(direction);
  // FIX 3: start from 0 instead of slamming to START_PWM.
  // DRIVE_START_KICK now ramps this up gradually below.
  applyMotorPwm(0);
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

  if (drivePhase == DRIVE_RAMP_DOWN) {
    if (now - lastPwmRampAt >= PWM_RAMP_PERIOD_MS) {
      lastPwmRampAt = now;
      int nextPwm = max(0, currentPwm - STOP_RAMP_STEP);
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

  if (drivePhase == DRIVE_DIRECTION_WAIT) {
    if (requestedDirection == 0) { drivePhase = DRIVE_STOPPED; return; }
    if (now - drivePhaseStartedAt >= DIRECTION_PAUSE_MS) {
      beginMotorStart(requestedDirection);
    }
    return;
  }

  if (requestedDirection == 0) {
    if (currentPwm > 0) beginRampDown();
    else { drivePhase = DRIVE_STOPPED; activeDirection = 0; }
    return;
  }

  if (drivePhase == DRIVE_STOPPED) {
    beginMotorStart(requestedDirection);
    return;
  }

  if (activeDirection != requestedDirection) {
    beginRampDown();
    return;
  }

  if (drivePhase == DRIVE_START_KICK) {
    // FIX 3: ramp PWM up toward START_PWM over the kick
    // window instead of holding it pinned at full value
    // from the first millisecond. Same cadence as the run
    // ramp (PWM_RAMP_PERIOD_MS) so it stays smooth.
    if (now - lastPwmRampAt >= PWM_RAMP_PERIOD_MS) {
      lastPwmRampAt = now;
      if (currentPwm < START_PWM) {
        applyMotorPwm(min(currentPwm + START_RAMP_STEP, START_PWM));
      }
    }
    if (now - drivePhaseStartedAt >= START_KICK_MS) {
      drivePhase = DRIVE_RUNNING;
      lastPwmRampAt = now;
    }
    return;
  }

  if (drivePhase == DRIVE_RUNNING) {
    if (now - lastPwmRampAt >= PWM_RAMP_PERIOD_MS) {
      lastPwmRampAt = now;
      if (currentPwm < requestedPwm) {
        applyMotorPwm(min(currentPwm + RUN_RAMP_STEP, requestedPwm));
      } else if (currentPwm > requestedPwm) {
        applyMotorPwm(max(currentPwm - RUN_RAMP_STEP, requestedPwm));
      }
    }
  }
}

int calculateFollowPwm(long speedCmS) {
  long limitedSpeed = constrain(speedCmS, 0, 40);
  return map(limitedSpeed, 0, 40, MIN_RUN_PWM, MAX_RUN_PWM);
}

// =====================================================
// BRAKE AND REVERSE ALERTS  (fixed: staggered engagement)
// =====================================================

void updateAlerts(bool braking, bool reversing) {
  unsigned long now = millis();

  if (braking) {
    if (!brakeAlertArmed) {
      brakingDetectedAt = now;
      brakeAlertArmed = true;
    }
  } else {
    brakeAlertArmed = false;
  }

  bool brakeAlertDue = brakeAlertArmed && (now - brakingDetectedAt >= ALERT_DELAY_MS);

  AlertMode requestedAlert = ALERT_NONE;
  if (brakeAlertDue) {
    requestedAlert = ALERT_BRAKE;
  } else if (!braking && reversing) {
    requestedAlert = ALERT_REVERSE;
  }

  bool alertChanged = requestedAlert != activeAlert;
  if (alertChanged) {
    activeAlert = requestedAlert;
    alertStartedAt = now;
    noTone(BUZZER);
  }

  unsigned long alertTime = now - alertStartedAt;

  if (activeAlert == ALERT_BRAKE) {
    digitalWrite(BRAKE_LED, HIGH);
    if (alertChanged) tone(BUZZER, 800, 600);
  }
  else if (activeAlert == ALERT_REVERSE) {
    bool reverseBeep = (alertTime % 700UL) < 100UL;
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

void updateCluster(bool targetDetected, bool cruiseMode, bool braking) {
  unsigned long now = millis();
  if (now - lastLcdUpdateAt < 100) return;
  lastLcdUpdateAt = now;

  char top[17];
  char bottom[17];

  if (!sensorReady) {
    strcpy(top, "SENSOR CHECK");
    strcpy(bottom, "PLEASE WAIT");
  }
  else if (braking) {
    strcpy(top, "!!  BRAKING  !!");
    snprintf(bottom, sizeof(bottom), "D:%2ldcm SPEED 0", measuredDistance);
  }
  else if (cruiseMode) {
    if (targetDetected) {
      snprintf(top, sizeof(top), "D:%2ldcm CRUISE", measuredDistance);
    } else {
      strcpy(top, "ACC     CRUISE");
    }
    makeSpeedBar(bottom, "SPD", currentPwm);
  }
  else {
    snprintf(top, sizeof(top), "D:%2ldcm FOLLOW", measuredDistance);
    if (requestedDirection > 0) makeSpeedBar(bottom, "FWD", currentPwm);
    else if (requestedDirection < 0) makeSpeedBar(bottom, "REV", currentPwm);
    else makeSpeedBar(bottom, "SPD", 0);
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

  // FIX 4: motor switching is a strong EMI source and can
  // glitch the I2C bus (SDA/SCL) the LCD sits on, which can
  // hang Wire.h in a way that looks identical to a brownout
  // reset from the outside. This bounds any I2C stall to
  // 3ms and resets the bus instead of hanging forever.
  Wire.setWireTimeout(3000, true);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("     M A V P");
  lcd.setCursor(0, 1);
  lcd.print("LOW POWER ACC");
  delay(1500);
  lcd.clear();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  bool newMeasurement = updateDistanceSensor();

  bool targetDetected = sensorReady && measuredDistance < 999 &&
                         measuredDistance <= DETECTION_LIMIT_CM;

  bool inBrakeZone = targetDetected && measuredDistance <= BRAKE_DISTANCE_CM;

  if (newMeasurement) {
    if (!targetDetected) {
      resetMotionTracking();
    } else if (!trackingTarget || (inBrakeZone && !wasInBrakeZone)) {
      beginMotionTracking(measuredDistance);
    } else {
      updateTargetMotion(measuredDistance);
    }
    wasInBrakeZone = inBrakeZone;
  }

  bool braking = inBrakeZone && targetMotion == SPEED_ZERO;

  bool cruiseMode = sensorReady &&
    (!targetDetected ||
     (measuredDistance > BRAKE_DISTANCE_CM &&
      (targetMotion == SPEED_ZERO || targetMotion == MOTION_UNKNOWN)));

  if (!sensorReady) {
    setDriveRequest(0, 0);
  } else if (cruiseMode) {
    setDriveRequest(1, CRUISE_PWM);
  } else if (!braking && targetMotion == MOVING_AWAY) {
    setDriveRequest(1, calculateFollowPwm(estimatedSpeedCmS));
  } else if (!braking && targetMotion == MOVING_TOWARD) {
    setDriveRequest(-1, calculateFollowPwm(estimatedSpeedCmS));
  } else {
    setDriveRequest(0, 0);
  }

  serviceMotors();

  bool reversing = requestedDirection < 0 && !braking;

  updateAlerts(braking, reversing);
  updateCluster(targetDetected, cruiseMode, braking);

  if (newMeasurement) {
    Serial.print("D:"); Serial.print(measuredDistance);
    Serial.print(" Motion:");
    if (targetMotion == MOVING_AWAY) Serial.print("AWAY");
    else if (targetMotion == MOVING_TOWARD) Serial.print("TOWARD");
    else if (targetMotion == SPEED_ZERO) Serial.print("SPEED0");
    else Serial.print("CHECK");
    Serial.print(" Dir:"); Serial.print(requestedDirection);
    Serial.print(" PWM:"); Serial.print(currentPwm);
    Serial.print(" Brake:"); Serial.println(braking ? "YES" : "NO");
  }
}
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int ENA = 5, IN1 = 4, IN2 = 7;
const int IN3 = 8, IN4 = 9, ENB = 6;
const int TRIG = A0, ECHO = A1;
const int BUZZER = 3;

const int   TARGET_DIST = 12;
const int   PANIC_DIST  = 7;
const int   DEADBAND    = 3;
const int   MAX_DIST    = 80;
const float Kp          = 8.0;
const int   MIN_PWM     = 110;
const int   MAX_PWM     = 200;
const int   CRUISE_PWM  = 160;
const int   RAMP_STEP   = 10;

LiquidCrystal_I2C lcd(0x27, 16, 2);
int   currentPwm = 0, currentDir = 0;
long  smoothDist = 50;

void setup() {
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  pinMode(BUZZER, OUTPUT);
  lcd.init(); lcd.backlight();
  lcd.setCursor(0,0); lcd.print("   M A V P");
  lcd.setCursor(0,1); lcd.print(" ADAPTIVE ACC");
  delay(1500); lcd.clear();
  Serial.begin(9600);
}

long readRaw() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long d = pulseIn(ECHO, HIGH, 30000);
  if (d == 0) return 999;
  return d / 58;
}
long readDistance() {
  long a = readRaw(), b = readRaw(), c = readRaw();
  long med = max(min(a,b), min(max(a,b),c));
  if (med >= 999) return smoothDist;
  smoothDist = (smoothDist * 2 + med) / 3;
  return smoothDist;
}

void applyMotors(int dir, int pwm) {
  if (dir > 0) { digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH); digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH); }
  else if (dir < 0) { digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW); digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW); }
  analogWrite(ENA, pwm); analogWrite(ENB, pwm);
}
void driveSmooth(int targetDir, int targetPwm) {
  if (targetDir != currentDir && currentPwm > 0) {
    currentPwm -= RAMP_STEP;
    if (currentPwm <= 0) { currentPwm = 0; currentDir = targetDir; }
  } else {
    currentDir = targetDir;
    if (currentPwm < targetPwm) currentPwm = min(currentPwm + RAMP_STEP, targetPwm);
    else if (currentPwm > targetPwm) currentPwm = max(currentPwm - RAMP_STEP, targetPwm);
  }
  applyMotors(currentDir, currentPwm);
}

void showCluster(long d, const char* mode) {
  lcd.setCursor(0,0);
  lcd.print("D:");
  if (d >= 999) lcd.print("-- "); else { if(d<10)lcd.print(" "); lcd.print(d); lcd.print("cm"); }
  lcd.print(" "); lcd.print(mode); lcd.print("   ");
  lcd.setCursor(0,1);
  int bars = map(constrain(d,0,MAX_DIST), 0, MAX_DIST, 0, 16);
  for (int i=0;i<16;i++) lcd.print(i<bars ? (char)255 : ' ');
}

void loop() {
  long d = readDistance();
  int targetDir, targetPwm; const char* mode;

  if (d >= MAX_DIST) {
    targetDir = 1; targetPwm = CRUISE_PWM; mode = "CRUISE"; noTone(BUZZER);
  } else {
    int error = d - TARGET_DIST;
    if (abs(error) <= DEADBAND) {
      targetDir = 0; targetPwm = 0; mode = "HOLD  "; noTone(BUZZER);
    } else if (error > 0) {
      int spd = constrain((int)(Kp*error), MIN_PWM, MAX_PWM);
      targetDir = 1; targetPwm = spd; mode = "FOLLOW"; noTone(BUZZER);
    } else {
      int spd = constrain((int)(Kp*(-error)), MIN_PWM, MAX_PWM);
      targetDir = -1; targetPwm = spd; mode = "BACKUP"; tone(BUZZER, 1200);
    }
  }
  driveSmooth(targetDir, targetPwm);
  showCluster(d, mode);
  delay(40);
}
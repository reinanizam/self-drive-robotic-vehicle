/*
  MAVP drivetrain bench test
  Lift the wheels off the table before uploading.

  Sequence (repeats):
    1. left L298N channel forward
    2. right L298N channel forward
    3. both channels forward

  No LCD, ultrasonic sensor, ACC logic, or buzzer is used here.
*/

const byte ENA = 5, IN1 = 4, IN2 = 7;
const byte IN3 = 8, IN4 = 9, ENB = 6;
const byte STATUS_LED = LED_BUILTIN;
const int TEST_PWM = 220;  // Higher than the confirmed 200-PWM baseline.

void stopAll() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(STATUS_LED, LOW);
}

void leftForward() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  analogWrite(ENA, TEST_PWM);
  analogWrite(ENB, 0);
  digitalWrite(STATUS_LED, HIGH);
}

void rightForward() {
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 0);
  analogWrite(ENB, TEST_PWM);
  digitalWrite(STATUS_LED, HIGH);
}

void bothForward() {
  // This is the confirmed forward direction for both motor sides.
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  analogWrite(ENA, TEST_PWM);
  analogWrite(ENB, TEST_PWM);
  digitalWrite(STATUS_LED, HIGH);
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  stopAll();
}

void loop() {
  stopAll();     delay(1000);
  leftForward();  delay(2000);
  stopAll();     delay(800);
  rightForward(); delay(2000);
  stopAll();     delay(800);
  bothForward();  delay(2500);
  stopAll();     delay(1500);
}

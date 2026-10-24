/*
  L298N Motor A/B Channel Test

  Motor A runs alone, stops, then Motor B runs alone.
  This test only drives forward. It never reverses.
*/

const byte ENA = 5;
const byte IN1 = 4;
const byte IN2 = 7;

const byte ENB = 6;
const byte IN3 = 8;
const byte IN4 = 9;

const int TEST_PWM = 230;

void stopBothChannels() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Forward direction
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  stopBothChannels();

  Serial.begin(9600);
  Serial.println("L298N CHANNEL TEST READY");
}

void loop() {
  // Test Motor A: OUT1 and OUT2
  Serial.println("MOTOR A ONLY: ENA / OUT1-OUT2");

  analogWrite(ENA, TEST_PWM);
  analogWrite(ENB, 0);

  delay(2000);

  stopBothChannels();
  Serial.println("STOP");

  delay(2000);

  // Test Motor B: OUT3 and OUT4
  Serial.println("MOTOR B ONLY: ENB / OUT3-OUT4");

  analogWrite(ENA, 0);
  analogWrite(ENB, TEST_PWM);

  delay(2000);

  stopBothChannels();
  Serial.println("STOP");

  delay(3000);
}
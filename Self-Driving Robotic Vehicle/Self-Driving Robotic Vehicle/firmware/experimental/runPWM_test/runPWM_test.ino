const int ENA = 5, IN1 = 4, IN2 = 7;
const int IN3 = 8, IN4 = 9, ENB = 6;

const int START_PWM = 230;
const int RUN_PWM = 180; // Test 220, 210, 200, 190...

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  // Initial push
  analogWrite(ENA, START_PWM);
  analogWrite(ENB, START_PWM);
  delay(300);

  // Lower running power
  analogWrite(ENA, RUN_PWM);
  analogWrite(ENB, RUN_PWM);
  delay(1500);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(3000);
}
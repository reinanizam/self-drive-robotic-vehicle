const int ENA = 5, IN1 = 4, IN2 = 7;
const int IN3 = 8, IN4 = 9, ENB = 6;

const int TEST_PWM = 230; // Change this value
/* 128 =  FAIL — car does not move
140 =  FAIL — car does not move
150 = ...
...
230- SUCCESS - car moves
*/

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  // Confirmed forward polarity
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, TEST_PWM);
  analogWrite(ENB, TEST_PWM);
  delay(1000);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(3000);
}
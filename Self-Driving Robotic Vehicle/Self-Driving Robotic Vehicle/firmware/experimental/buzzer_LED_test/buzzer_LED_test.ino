const int LED_PIN = 2;
const int BUZZER = 3;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  // Two short brake-alert pulses
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER, 800);
    delay(180);

    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER);
    delay(250);
  }
}

void loop() {
  // Test runs once after upload/reset
}
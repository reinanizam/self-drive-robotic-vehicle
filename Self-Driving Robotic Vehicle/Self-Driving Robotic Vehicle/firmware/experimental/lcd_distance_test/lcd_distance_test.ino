#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int TRIG = A0;
const int ECHO = A1;

LiquidCrystal_I2C lcd(0x27, 16, 2);

long readDistanceCm() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG, LOW);

  unsigned long duration = pulseIn(ECHO, HIGH, 30000UL);

  if (duration == 0) {
    return 999;
  }

  return duration / 58;
}

void setup() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("   M A V P");

  lcd.setCursor(0, 1);
  lcd.print("DISTANCE SENSOR");

  delay(1500);
  lcd.clear();
}

void loop() {
  long distance = readDistanceCm();

  lcd.setCursor(0, 0);
  lcd.print("DISTANCE:       ");

  lcd.setCursor(0, 1);

  if (distance >= 999) {
    lcd.print("NO TARGET       ");
  } else {
    lcd.print(distance);
    lcd.print(" cm            ");
  }

  delay(200);
}
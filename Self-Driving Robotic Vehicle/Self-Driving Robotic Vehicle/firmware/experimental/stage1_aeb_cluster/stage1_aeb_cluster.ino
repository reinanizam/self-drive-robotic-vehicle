/*
 * MINI AUTONOMOUS VEHICLE PLATFORM — STAGE 1
 * Instrument cluster + AEB logic
 *
 * Wiring:
 *   LCD1602 I2C:  GND->GND  VCC->5V  SDA->A4  SCL->A5
 *   HC-SR04:      VCC->5V   TRIG->A0  ECHO->A1  GND->GND
 *   Buzzer:       +->D3     -->GND
 *   LED + 220ohm: anode->D8  cathode->GND
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ---------- CONFIG ----------
const uint8_t LCD_ADDR = 0x27;   // confirmed by I2C scanner
const int TRIG_PIN     = A0;
const int ECHO_PIN     = A1;
const int BUZZER_PIN   = 3;
const int LED_PIN      = 2;

const int AEB_DISTANCE_CM  = 15;
const int WARN_DISTANCE_CM = 30;
// ----------------------------

LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

long readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999;
  return duration / 58;
}

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("  MAVP  v0.1");
  lcd.setCursor(0, 1);
  lcd.print(" SYSTEM  CHECK");
  delay(1500);
  lcd.clear();

  Serial.begin(9600);
}

void loop() {
  long d = readDistanceCm();

  Serial.print("distance_cm=");
  Serial.println(d);

  if (d < AEB_DISTANCE_CM) {
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER_PIN, 1000);

    lcd.setCursor(0, 0);
    lcd.print("!! BRAKING !!   ");
    lcd.setCursor(0, 1);
    lcd.print("OBSTACLE: ");
    lcd.print(d);
    lcd.print("cm   ");
  }
  else {
    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER_PIN);

    lcd.setCursor(0, 0);
    lcd.print("SPD:--  DIST:");
    if (d >= 999) { lcd.print("-- "); }
    else          { lcd.print(d); lcd.print("cm "); }

    lcd.setCursor(0, 1);
    if (d < WARN_DISTANCE_CM) lcd.print("MODE: CAUTION   ");
    else                      lcd.print("MODE: STANDBY   ");
  }

  delay(100);
}
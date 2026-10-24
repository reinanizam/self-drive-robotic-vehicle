#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);

  while (!Serial) {
  }

  Serial.println("I2C device test");
}

void loop() {
  int deviceCount = 0;

  Serial.println("Scanning...");

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Device found at 0x");

      if (address < 16) {
        Serial.print("0");
      }

      Serial.println(address, HEX);
      deviceCount++;
    }
  }

  if (deviceCount == 0) {
    Serial.println("No devices found");
  }

  Serial.println("Done");
  Serial.println();

  delay(3000);
}
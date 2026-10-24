// Co-simulation harness: runs the real .ino (host stubs) as a child process.
// stdin protocol (one command per line):
//   I                         -> run setup() (gyro bias from current raw), print READY
//   S <gyroRaw> <irMask> <echoUs> <dtUs> [guide...]
//        set sensors, optionally queue a guide line, advance dtUs, run loop(),
//        print "O <ENA> <ENB> <activeDirection> <currentPwm>" then any new
//        serial text from the firmware as "T <line>" lines, then "E".
#include "stubs/Arduino.h"
#include <cassert>
#include <iostream>
#include <string>
#include <sstream>
#include FW_PATH

static void line(byte mask) {
  const int pins[5] = {A2, 13, 12, 11, 10};
  for (byte i = 0; i < 5; i++) inputPins[pins[i]] = mask & (1 << i) ? LOW : HIGH;
}
// World clock. The physical world advances by dt per "S" command. A pulseIn()/delay() inside loop()
// moves the firmware clock ahead of the world (on the real Uno the car keeps moving while the
// Uno waits for the echo, so the firmware clock and the world clock are the same). The firmware
// is then "busy" until the world catches up, and loop() is not called meanwhile. Define
// LEGACY_CLOCK to get the old behaviour (firmware clock runs ahead of the world).
static unsigned long worldUs = 0;
int main() {
  std::string cmd;
  size_t seen = 0;
  while (std::getline(std::cin, cmd)) {
    std::istringstream in(cmd);
    std::string op; in >> op;
    if (op == "I") {
      int raw, mask; in >> raw >> mask;
      Wire.gyro[2] = (int16_t)raw; line((byte)mask);
      setup();
      worldUs = simulatedUs;
      std::cout << "READY\n";
    } else if (op == "S") {
      int raw, mask; unsigned long echo, dt; in >> raw >> mask >> echo >> dt;
      Wire.gyro[2] = (int16_t)raw; line((byte)mask); echoUs = echo;
      std::string rest; std::getline(in, rest);
      size_t p = rest.find_first_not_of(' ');
      if (p != std::string::npos) {
        for (char c : rest.substr(p)) Serial.incoming.push_back(c);
        Serial.incoming.push_back('\n');
      }
#ifdef LEGACY_CLOCK
      simulatedUs += dt;
      loop();
#else
      worldUs += dt;
      if (simulatedUs <= worldUs) {      // not blocked in pulseIn(): run one loop() at world time
        simulatedUs = worldUs;
        loop();
      } else {                            // still waiting for the echo; keep the guide line queued
      }
#endif
      std::cout << "O " << pwmPins[ENA] << " " << pwmPins[ENB] << " " << (int)activeDirection << " " << outputPins[4] << " " << outputPins[7] << " " << outputPins[8] << " " << outputPins[9] << "\n";
    }
    std::string all = Serial.output.str();
    if (all.size() > seen) {
      std::istringstream lines(all.substr(seen));
      std::string l;
      while (std::getline(lines, l)) std::cout << "T " << l << "\n";
      seen = all.size();
    }
    std::cout << "E\n" << std::flush;
  }
}

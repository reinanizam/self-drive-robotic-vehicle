# Mini Autonomous Vehicle Platform (MAVP)

A small self-driving car built from an Arduino Uno and a Raspberry Pi 5. It follows a black-tape track, takes the corners on a gyro, and uses adaptive cruise control (ACC).

The repository holds everything: the car firmware, the Raspberry Pi vision program, a closed-loop simulator that runs the real code on a PC, and the documentation. Version V7.6c is the firmware that ran on the real car.

## What it does

| Part | Job |
|---|---|
| **Raspberry Pi 5 + camera** | Finds the tape in the camera picture, measures the offset, the corner distance and the corner angle, and sends them to the Uno ten times a second. Serves a live web dashboard (port 5001). It only suggests; it never drives the motors. |
| **Arduino Uno** | The only part that drives the motors. Steers on the Pi's suggestion, turns corners on the gyro, runs the cruise control from the sonar, and stops by itself when the Pi goes quiet for 0.4 s. |
| **5-channel IR bar** | Support only: it steers when the camera does not see the tape, and never starts a corner by itself. |
| **MPU6050 gyro** | Measures the corner turns. |
| **HC-SR04 sonar** | Distance to the car ahead: stop, watch the gap, back off, follow. |
| **16x2 LCD, LED, buzzer** | Show and sound what the car is doing: `!!  BRAKING  !!` with one beep, `<< REVERSING <<` with repeated beeps. |


## Status 

| Version | Where | Status |
|---|---|---|
| **V7.6c Uno + V7.6c Pi** (main) | `firmware/mavp_uno`, `raspberry_pi` | Ran on the real car. Cruise-control back-off works straight once the motor wiring is correct (see [docs/WIRING.md](docs/WIRING.md): swapped motor wires make the car curve in reverse). Octagon laps in the simulator: 9 of 13 cases complete. |
| V7.6e Uno | `firmware/experimental/mavp_uno_v76e` | Simulator-tested only. Cruise control keeps working when the lead car blocks the camera's view of the tape; stronger straight back-off. Hardware test pending. |
| ACC demo | `firmware/experimental/mavp_acc_demo` | Arduino-only demo (no Pi, no camera, no tape): cruise, brake, straight gyro-held back-off, follow. Simulator-tested only. Hardware test pending. |
| Original cruise control | `firmware/acc_standalone` | The first cruise-control program, kept as a fallback. No gyro. |

The simulator invents the car's camera geometry, motor lag and the lead car's sonar echo. A pass there is a software result, not a road test; a build that passed the simulator once behaved worse on the car. Details and numbers: [docs/TESTING.md](docs/TESTING.md).

## Repository layout

```
firmware/
  mavp_uno/                  main Uno program (V7.6c, camera tilted ~22 degrees)
  experimental/
    mavp_uno_v76e/           V7.6e (simulator-tested)
    mavp_acc_demo/           Arduino-only ACC demo (simulator-tested)
  acc_standalone/            first cruise-control program (fallback)
raspberry_pi/
  mavp_pi.py                 camera, vision, web dashboard
  requirements.txt
simulator/                   closed-loop simulator (real .ino + real Pi program on a PC)
docs/
  ARCHITECTURE.md  WIRING.md  GETTING_STARTED.md  TESTING.md  VERSION_HISTORY.md
```

## Hardware

Arduino Uno, Raspberry Pi 5 with a camera (OV5647 sensor through picamera2, 320x240 at 24 pictures per second, about 53° field of view, tilted about 22° down), 5-channel IR line bar, MPU6050, HC-SR04 sonar, L298N motor driver with DC motors on two banks (left and right), 16x2 I2C LCD (address 0x27), brake LED and buzzer. The Uno is connected to the Pi by USB (115200 baud). Pin table and wiring notes: [docs/WIRING.md](docs/WIRING.md).

## Quick start

1. **Uno:** open `firmware/mavp_uno/mavp_uno.ino` in the Arduino IDE, board Arduino Uno, install the library `LiquidCrystal I2C`, upload. At reset the Serial Monitor (115200) shows a line ending `P45 C22 V76c`, then `ARDUINO_READY`.
2. **Pi:** copy `raspberry_pi/mavp_pi.py` to the Pi, install the packages in `raspberry_pi/requirements.txt`, run `python3 -u mavp_pi.py`. It prints `PI FILE V7.6c`. Open `http://<pi-address>:5001` in a browser.
3. **Drive:** put the car on the tape, with the wheels off the floor for the first run. Ctrl+C on the Pi stops the motors.

Step by step, with the checks: [docs/GETTING_STARTED.md](docs/GETTING_STARTED.md).

## Simulator

The simulator compiles the real `.ino` for the PC, feeds the real Pi program with drawn camera pictures, and closes the loop with a simple car model. It runs the octagon laps, the lead-car back-off cases S1-S13, the demo cases D1-D5 and the safety checks. See [simulator/README.md](simulator/README.md).

## Known limits

- This is a small student-built car for a taped indoor track.
- The vision is tuned for one high-contrast marked track in indoor lighting. A different floor, lighting or camera tilt needs re-tuning, and strong glare can make the camera lose the track.
- The vision is classical (OpenCV, no trained model). It follows a marked track and does not classify objects.
- The Arduino Uno's flash memory is almost full (V7.6c uses 96.7%), so a new feature needs something removed first.
- This is an educational project. It is not safety-certified and not meant for real vehicles.




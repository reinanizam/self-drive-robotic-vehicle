# Getting started

## 1. Arduino Uno

1. Install the Arduino IDE and, in the Library Manager, the library **LiquidCrystal I2C**. `Wire` comes with the IDE.
2. Open `firmware/mavp_uno/mavp_uno.ino` (the folder name must match the file name, so open the file from inside its folder).
3. Tools > Board > Arduino Uno, choose the port, close the Serial Monitor, click Upload.
4. Open the Serial Monitor at 115200 baud and press RESET on the Uno. Keep the car standing still on a flat surface: the gyro calibrates for about one second. You should see

   ```
   SENSOR_FUSION_READY V7 A_LEFT CORNER_TURN FW73 P45 C22 V76c
   ARDUINO_READY
   ```
   `SENSOR_FAULT: MPU check failed` instead means the gyro was not found or the car moved or was tilted during the calibration: check SDA/SCL/power, then reset again with the car still.
5. Close the Serial Monitor (only one program can use the port) and connect the Uno to the Pi by USB.

Camera tilted only about 6.5° instead of 22°: in the sketch set `PHASE3_TO_PIVOT_MS = 1700` and `PHASE3_IR_MIN_MS = 900` (the 22° values are 1200 and 600). At 6.5° the nearest visible floor is about 19 cm ahead of the lens, at 22° about 11 cm; the octagon's short sides are only 30-44 cm, so the 22° tilt is recommended.

## 2. Raspberry Pi 5

On Raspberry Pi OS:

```
sudo apt install python3-picamera2 python3-opencv python3-numpy python3-serial python3-flask
```

Copy `raspberry_pi/mavp_pi.py` to the Pi and run it:

```
python3 -u mavp_pi.py
```

- It prints `PI FILE V7.6c`, finds the Uno on `/dev/ttyACM*` or `/dev/ttyUSB*`, and waits for `ARDUINO_READY`.
- Dashboard: `http://<pi-address>:5001`. The header shows the Pi version, the tape width and the pace.
- Options: `--tape-cm 4.7` tape width, `--cruise 70` straight-line pace in percent (40-90), `--swap-rb` if red and blue are swapped in the picture, `--loose-tape` if the lighting changed and the black-only tape rules reject the tape, `--colour-test` saves one picture and prints the channel means.
- Ctrl+C stops the motors and ends the program.
- If a previous run is still alive (`pgrep -af python3`), stop it first: two programs cannot share the port.

If it prints `Arduino did not send ARDUINO_READY`: the Uno did not finish its start-up, the Serial Monitor is still open, or another sketch is on the Uno. Look at the Uno's screen or the Serial Monitor for `SENSOR_FAULT`, and make sure the car stood still during the reset.

## 3. First drive

1. Wheels off the floor for the first run. Check forward, sides and reverse as in [WIRING.md](WIRING.md).
2. Put the car on the tape (camera about 22° down, nearest floor in view about 11 cm ahead).
3. Start the Pi program. The car drives by itself. Stop with Ctrl+C.
4. Cruise control: put a flat box or a second car in front, about 45 cm away or closer. The car stops (`!!  BRAKING  !!`). Move the box toward the car and the car backs off straight; move it away and the car follows.

Keep about 1.5 m free behind the car: nothing looks backwards.

## 4. Arduino-only ACC demo (no Pi, no camera, no tape)

For a video of the cruise control alone: upload `firmware/experimental/mavp_acc_demo/mavp_acc_demo.ino`, do not start the Pi program, put the front car straight ahead about 1 m away and press RESET. After the gyro calibration and a 3-second countdown the car cruises, brakes with one beep, backs off straight when the front car comes toward it, and follows when it drives away. This sketch is simulator-tested only: do one dry run before filming. `firmware/acc_standalone/acc_standalone.ino` is the original cruise-control program without the gyro, as a fallback.

## 5. Tuning

| Symptom | Constant (in the sketch) |
|---|---|
| Wobbles while backing off | lower `REVERSE_HOLD_KP` (8) / `REVERSE_HOLD_KI` (25) |
| Backs off too long | lower `MAX_REVERSE_TOTAL_MS` (V7.6c 2500) |
| Wrong side steered | `MOTOR_A_IS_LEFT` |
| Detects the front car too early or late | `DETECTION_LIMIT_CM` (45) |

In the demo sketch the constants are at the top of its "STRAIGHT-HOLD" block (`HOLD_KP`, `HOLD_KI`, `MAX_REVERSE_TOTAL_MS` 5000, `CRUISE_MAX_MS` 20000).

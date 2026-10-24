# Simulator

A closed-loop, software-in-the-loop simulator.

**Real:** the Arduino sketch (compiled for the PC with Arduino stubs and run as a child process), the Raspberry Pi vision and planner program (`../raspberry_pi/mavp_pi.py`, imported unchanged and fed rendered camera pictures), and the serial protocol between them.

**Invented** (a test fixture, not a measurement of the car): camera height and tilt, wheel track, speed against PWM, wheel lag, IR bar position and spacing, latencies, floor brightness, the lead car's size and sonar echo. The runners sweep these over wide ranges so a pass does not depend on one made-up geometry. A pass is a software result, not a road test.

Needs `g++`, `python3`, `numpy` and `opencv-python`. Run everything from this folder.

## Build a sketch for the PC

```
bash build.sh ../firmware/mavp_uno/mavp_uno.ino sim_v76c
bash build.sh ../firmware/experimental/mavp_uno_v76e/mavp_uno_v76e.ino sim_v76e
bash build.sh ../firmware/experimental/mavp_acc_demo/mavp_acc_demo.ino sim_acc_demo
```

## Run

| Test | Command | Time | Expected |
|---|---|---|---|
| Safety checks (STOP, link loss, camera and IR loss, reverse limit, no tape) | `python3 safety_test.py ./sim_v76e` or `python3 safety_test.py ./sim_v76c v76c` | seconds | `ALL PASS` (11 checks) |
| Octagon laps, 4.7 cm tape, camera 22° (13 cases) | `python3 run_octagon.py v76c` | about 10 min | 9 COMPLETED, 4 stopped |
| Lead-car back-off cases S1-S13 | `python3 run_backoff.py v76c v76e` | about 6 min per version | see [../docs/TESTING.md](../docs/TESTING.md) |
| ACC demo cases D1-D5 | `python3 run_demo.py ./sim_acc_demo` | about 2 min | see [../docs/TESTING.md](../docs/TESTING.md) |
| ACC demo extra checks | `python3 run_demo_extra.py cruise` (also `budget`, `sign`, `start`, `ng`) | minutes | see the docstring |
| Buzzer, LED and screen during a back-off | see `alert_check/alert_summary.py` | about 4 min | one beep on BRAKING, beeps every 0.7 s on REVERSING |

Environment variables: `PI` path of the Pi program (default `../raspberry_pi/mavp_pi.py`), `PITCH` camera tilt in degrees (default 22), `CAMH` camera height in m (default 0.10), `NP` worker processes (default 2).

For the `ng` check of the demo, build the two gyro variants:

```
bash build.sh ../firmware/experimental/mavp_acc_demo/mavp_acc_demo.ino sim_acc_demo_gyro_absent  stubs_gyro_absent
bash build.sh ../firmware/experimental/mavp_acc_demo/mavp_acc_demo.ino sim_acc_demo_gyro_shaking stubs_gyro_shaking
```

## Files

| File | What |
|---|---|
| `closedloop.py` | the simulator: car model, camera renderer, IR bar, sonar, lead car, serial bridge to the compiled sketch and the Pi program |
| `sim_fw.cpp`, `stubs/` | host build of a `.ino`: Arduino, Wire (MPU6050) and LCD stubs |
| `stubs_gyro_absent/`, `stubs_gyro_shaking/` | alternative `Wire.h` for the gyro-failure checks |
| `oct.py` | the octagon track used for the lap tests |
| `acc2.py` | one lead-car scenario (cruise, stop, lead comes toward the car, back off, lead drives away, follow) |
| `run_octagon.py`, `run_backoff.py`, `run_demo.py`, `run_demo_extra.py`, `safety_test.py` | the test runners |
| `alert_check/` | the same harness with stubs that count `tone()` calls and record the LCD text |
| `build.sh` | builds a sketch for the PC |

Compiled binaries (`sim_*`) are not part of the repository.

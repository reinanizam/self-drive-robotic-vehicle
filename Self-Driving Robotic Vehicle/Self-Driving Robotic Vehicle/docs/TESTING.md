# Testing and verification

## What has run on the real car, and what has not

| | Real car | Simulator |
|---|---|---|
| **V7.6c** Uno + Pi (main) | Ran on the car. The cruise-control back-off is straight once the motor wiring is correct. | Octagon laps, 13 cases: 9 complete. Back-off S1-S13, safety checks 11/11. |
| V7.6e Uno | Not tested yet | Back-off S1-S13, safety checks 11/11, octagon identical to V7.6c |
| ACC demo | Not tested yet | D1-D5 and extra checks below |

Full octagon laps on the real car are not documented in this repository; the lap results below are simulator results.

## What the simulator is, and why a pass is not enough

The simulator runs the real Uno sketch and the real Pi program, with an invented car around them (camera geometry, motor lag, IR bar, lead car, sonar echo). It finds logic errors well: wrong-way corrections, restart loops, a back-off that never ends. It cannot know how the real motors, the real floor, the real lens and a real toy car's echo behave. A build that passed it behaved worse on the real car, which is why V7.6c stayed the main version and every newer file is marked experimental until it has run on the car.

How to run everything: [../simulator/README.md](../simulator/README.md).

## Build sizes (strict AVR build, Arduino Uno, limit 32,256 B flash and 2,048 B RAM)

| Sketch | Flash | RAM |
|---|---|---|
| `mavp_uno` (V7.6c) | 31,682 B (96.7%) | 1,288 B |
| `mavp_uno_v76e` | 32,178 B (98.2%) | 1,323 B |
| `mavp_acc_demo` | 18,752 B (57.2%) | 1,114 B |
| `acc_standalone` | 13,172 B (40.2%) | 820 B |

## Octagon laps (V7.6c, simulator)

The test track is the octagon: 2 long sides of 1.20 m, 4 diagonals of about 0.44 m, 2 short sides of about 0.30 m, 45° corners. 4.7 cm tape, camera tilted 22°, 7 corners. 13 cases, the car's speed and the IR bar spacing and failures varied:

| Result | Cases |
|---|---|
| Completed (9) | left loop with IR spacing 1.3 / 2.5 / 4.0 cm; right loop with 1.3 / 2.5 cm; start in the middle of a side; start 1.5 cm off and 2° crooked; IR bar flickering (one phase); camera 8 cm high |
| Stopped (4) | IR bar flickering (other phase, after corner 6), IR bar dead (after corner 2), camera 14 cm high, car 30% slower than assumed |

The left loop drifts up to about 18 cm off the tape on the first short side before it finds it again; the right loop stays within about 6 cm. V7.6e gives the same 13 results as V7.6c.

## Lead-car back-off S1-S13 (simulator)

The car drives up to a lead car and stops; the lead car then moves 24 cm toward it (like a car backing into it), stops, and later drives away 40 cm. Per case: how far the car backed off / largest heading change during the back-off / closest gap / did it drive forward again afterwards.

| Case | V7.6c | V7.6e |
|---|---|---|
| S1 default | 34 cm / 0.0° / 19 cm / yes | 33 cm / 0.0° / 18 cm / yes |
| S2 right bank 15% weak | 30 cm / 2.2° / 40 cm / yes | 31 cm / 1.1° / 19 cm / yes |
| S3 left bank 15% weak | 30 cm / 2.3° / 40 cm / yes | 31 cm / 1.1° / 19 cm / yes |
| S4 left bank 30% weak | 29 cm / 8.4° / 41 cm / yes | 31 cm / 1.9° / 40 cm / yes |
| S5 light wide lead (camera view blocked) | 34 cm / 0.0° / 19 cm / yes | 33 cm / 0.0° / 18 cm / yes |
| S6 light lead + right bank 15% weak | 30 cm / 2.2° / 40 cm / yes | 31 cm / 1.1° / 19 cm / yes |
| S7 fast lead 0.2 m/s | 36 cm / 0.0° / 22 cm / yes | 39 cm / 0.0° / 38 cm / yes |
| S8 long back-off 0.9 m (tape behind) | 94 cm / 0.0° / 17 cm / yes | 94 cm / 0.0° / 16 cm / yes |
| S9 2 cm off, 4° crooked, right bank 15% weak | 32 cm / 3.6° / 40 cm / yes | 31 cm / 2.0° / 40 cm / yes |
| S10 camera blind from 8.5 s | **no back-off** / 8 cm / no | 31 cm / 0.0° / 32 cm / no |
| S11 camera blind + left bank 30% weak | **no back-off** / 14 cm / no | 27 cm / 3.0° / 34 cm / no |
| S12 camera blind from 10.2 s (mid back-off) | 17 cm / 0.0° / 28 cm / no | 32 cm / 0.0° / 30 cm / no |
| S13 right bank 30% weak | 29 cm / 8.3° / 41 cm / yes | 31 cm / 1.8° / 40 cm / yes |

`no` in the last column of S10-S12 is intended: in those cases the camera never sees the tape again, and forward driving always needs the tape. V7.6c does not back off when the camera is blind (the cruise control needs a verified track); V7.6e does.

## Safety checks (`safety_test.py`, 11 checks)

`STOP` cuts both motors in the same step and a silent Pi stops them within 0.4 s; the reverse is limited (V7.6c 2.5 s, V7.6e 4 s); with no tape ever confirmed the car does not move even if something approaches; with the track lost the car never drives forward. All 11 pass for V7.6c (`v76c` mode, which expects V7.6c's rule that the back-off stops when the tape is lost) and for V7.6e.

## ACC demo D1-D5 (simulator)

The lead car stands 95 cm ahead, moves 24 cm toward the car at 9 s and away 40 cm at 16 s. Cases: balanced car, one side 5% weaker, one side 15% weaker.

| Case | Result |
|---|---|
| D1 default | backs 33.5 cm, straight (0.0°), closest gap 17 cm, follows forward |
| D2 / D3 one side 5% weaker | backs 32.6 cm, 1.0°, gap 17 cm, follows |
| D4 / D5 one side 15% weaker | backs 31.0 cm, 1.4°, gap 17 cm, follows |

For comparison, the original cruise-control program (`acc_standalone`, no gyro) with a 5% weaker side backed off at 5.9° and then lost the lead and drove about 1.1 m past it; with a 15% weaker side it never saw the lead again.

More checks of the demo:

- Gyro missing, or shaking at start-up: the demo runs as the original program (balanced car: identical result; with a weaker side it drifts like the original).
- Gyro wired the wrong way: a balanced car is fine; with a weaker side the straight-hold falls back to equal PWM after 12°.
- Nothing ahead: the car stops by itself after 20 s of cruising.
- The lead keeps coming faster than the car backs off: the back-off is limited and stops by itself.
- Start 4° crooked: backs off 33 cm holding its starting direction (the car cannot steer toward the lead), closest gap 17 cm. Crooked and with a 15% weaker side it can drift out of the sonar's view of a narrow lead. Start the demo straight and centred.
- A side 30% weaker cannot be corrected (the correction is limited to 45 PWM forward, 60 backward).

## Alerts (buzzer, LED, screen)

`simulator/alert_check` counts the buzzer calls and records the screen text during a back-off. V7.6e and the demo: `!!  BRAKING  !!` with ONE beep when the car stops behind a car, `<< REVERSING <<` with a beep every 0.7 s (0.2 s on) while backing off, `!!  BRAKING  !!` with one beep at every new stop.

## Not verified

- Anything on the real car for V7.6e and the demo.
- The simulator's motor lag, reverse speed and the lead car's echo are assumptions.
- Real sonar echoes of an angled or soft toy car may drop out.
- The car cannot steer toward the lead car in the demo (no camera): it keeps its starting direction.
- The dashboard may show "track lost" while V7.6e backs off blind; the car's screen is the reliable indicator.

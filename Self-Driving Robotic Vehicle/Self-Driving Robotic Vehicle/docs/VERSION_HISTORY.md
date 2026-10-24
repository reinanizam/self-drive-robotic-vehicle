# Version history

The Pi program and the Uno sketch are versioned together. The Uno prints its version in the boot banner (`... V76c`) and the Pi prints `PI FILE V7.6c`.

| Version | Date | What changed |
|---|---|---|
| V7 to V7.4 | 2026-10 | Base design: camera steering with the IR bar as a second source, corners turned on the gyro, a 45° prior for the corner angle, black-only tape rules. V7.1: a dark upright object (a bag) at the end of the strip is no longer read as a corner. |
| V7.5 | 2026-10-22 | Octagon track: the heading reference is re-taken after every corner, no second corner for 1.6 s after a turn, the 45° rule applies when the Pi measures 20-80°, 0.4 s grace for a short dropout of both sources, camera tilted to about 22°. New live dashboard with `/status.json`. |
| V7.6 | 2026-10-22 | Pi: the tape filters know the real tape is 4.7 cm wide (they were written for about 1.9 cm and cut the real tape down to a thin edge strip). |
| V7.6b | 2026-10-23 | The IR bar is support only: it never starts a corner and steers only when the camera does not see the tape (on 4.7 cm tape it had started a false turn on a straight). `--tape-cm` option. Dashboard quality value no longer flickers to 0. |
| **V7.6c** | 2026-10-23 | **Main version, ran on the car.** Pi: about 3 times faster per picture (80 to 27 ms), `--cruise` option, dashboard shows version, tape width and pace. Uno: the cruise-control back-off holds the starting heading with a gyro PI controller and is time limited (2.5 s). |
| V7.6e | 2026-10-24 | Experimental (simulator-tested). Cruise control keeps working when the lead car blocks the lens and the tape disappears; one heading reference for the whole back-off, stronger correction (up to 60 PWM), 4 s total, stops the back-off if the heading drifts more than 25°; braking screen with one beep, REVERSING screen with a beep every 0.7 s; forward driving resumes only on a confirmed tape. |
| ACC demo | 2026-10-24 | Experimental (simulator-tested). The original cruise-control program with the gyro straight-hold, the braking and reversing alerts, a gyro start-up that cannot block the demo, and limits (20 s of cruising with nothing ahead, 5 s of backing off). No Pi, no camera, no tape. |

## Lessons that changed the code

- **A simulator pass is not a road test.** The simulator invents the camera geometry, the motor lag and the lead car's echo. A build that passed it was worse on the car, so every experimental file here is labelled as simulator-tested only.
- **A car that curves in reverse may be a wiring fault.** The straight back-off first looked like a control problem (bank imbalance, gyro gain). On the prototype it was the wires between the motors and the driver. Check [WIRING.md](WIRING.md) before changing the controller.
- **The IR bar and wide tape do not mix.** Under 4.7 cm tape the bar is covered most of the time, so "3-4 channels lit with one edge lit" looks like a corner leg on any straight where the car is slightly off-centre. The bar now supports; it does not decide.
- **The flash limit is a design constraint.** V7.6e needed the telemetry line printed through one helper to fit; the Uno has no room left for new features without removing something.

# Architecture

## Two brains, one rule

```
 camera ──► Raspberry Pi 5 ──────── USB serial, 115200 baud ────────► Arduino Uno ──► L298N ──► motors
            vision, planner,        GUIDE5 / TRACK_LOST / STOP         motor control,
            web dashboard           about 10 lines per second          safety, ACC
                                    ◄──────────────────────────────
                                    SENS telemetry, ARDUINO_READY

 IR bar ─┐
 gyro ───┼──────────────────────────────────────────────────────────► Arduino Uno
 sonar ──┘
```

**The Pi only suggests. The Uno decides.** The Pi never drives a motor. If the Pi stops sending (crash, cable, Ctrl+C), the Uno stops the motors after 0.4 s on its own. Every motor command, including the stop rules, lives in the Uno program.

## Raspberry Pi (`raspberry_pi/mavp_pi.py`)

1. **Capture** 320x240 pictures at 24 per second, each with the timestamp of the exact picture.
2. **Find the tape.** The floor brightness sets the "dark" threshold. The tape is a dark band with floor on both sides, about 4.7 cm wide. Pale marble veins and shadows fail a contrast test and are not tape. The expected band width is scaled for the tape width (`--tape-cm`, default 4.7).
3. **Trace the strip** row by row from the bottom of the picture upwards. This gives the sideways offset near the car, the bend further ahead, a quality value (0-100) and, when a corner is approaching, its direction and angle.
4. **Plan.** A corner is announced only after several consecutive confirming pictures. The planner also tells the Uno the phase: 0 straight, 1 corner approaching, 2 corner entered, 3 corner at the bottom of the picture (the camera can no longer see it).
5. **Send** `GUIDE5,near,turn,quality,bend,pace,phase` to the Uno. `near` is the steering offset (-100..100), `turn` the corner angle in degrees (0 when no bend is seen), `pace` the straight-line speed in percent (`--cruise`, default 70). Without a tape lock it sends `TRACK_LOST`.
6. **Dashboard** (Flask, port 5001): camera picture with the detected tape, the command, the Uno's telemetry, a diagnostics download. Every run is logged under `~/mavp_logs`.

## Arduino Uno (`firmware/mavp_uno/mavp_uno.ino`)

| Block | What it does |
|---|---|
| **Sensors** (`MavpSensors`) | MPU6050 yaw (calibrated at start, the car must stand still and upright; a fault latches until reset), 5-channel IR bar with a debounce, line position and confidence. |
| **Guidance input** | Reads the Pi's lines. A line older than 0.4 s counts as "no Pi": the motors stop. |
| **Steering** | The camera steers when it sees the tape. The IR bar steers only when the camera does not, and never starts a corner. A dropout of both sources shorter than 0.4 s is bridged by going straight; longer ends in a stop. |
| **Corners** | The Pi announces a corner. When it reaches the blind zone the Uno pivots on the spot with the gyro. The turn ends when the camera or the IR bar find the next leg centred, or on a gyro angle limit. The expected angle is the prior `TRACK_LEG_DEG` (45°); the Pi's measured angle and the gyro correct it, and after every corner the heading reference is re-taken from where the car ended up. |
| **Cruise control (ACC)** | A sonar state machine with the intents OBSERVE / STOPPED / FORWARD / REVERSE. Car ahead closer than 45 cm: stop and watch the gap with a filtered sonar. Gap closing: back off. Gap opening: follow. |
| **Straight back-off** | The heading at the start of the reverse is stored; a gyro PI controller (8 PWM per degree + 25 PWM per degree-second) holds it, with the slower motor bank never dropped below the PWM that still turns it. Total reverse time is limited (2.5 s in V7.6c, 4 s in V7.6e); after that the car stops and waits. |
| **Output** | PWM ramps, motor banks, the LCD, the brake LED and the buzzer. |

## Motor sides

Bank A (`ENA`, `IN1`, `IN2`, L298N OUT1/OUT2) is the LEFT side and bank B (`ENB`, `IN3`, `IN4`, OUT3/OUT4) the RIGHT side. One constant, `MOTOR_A_IS_LEFT`, holds this. The steering, the gyro back-off and the corner pivots all depend on it. See [WIRING.md](WIRING.md).

## What the ACC sequence looks like

1. Cruise forward.
2. Car ahead within 45 cm: brake, screen `!!  BRAKING  !!`, brake LED on, one beep.
3. The car ahead comes toward the car (closing gap): back off straight on the gyro, screen `<< REVERSING <<`, beeps every 0.7 s (V7.6e / demo).
4. The car ahead stops: the car stops.
5. The car ahead drives away: the car follows forward.

## Safety rules (all in the Uno)

- No Pi line for 0.4 s: motors off.
- `STOP` from the Pi: both motors off in the same step.
- Gyro fault, a motor spinning the wrong way, or a corner turned the wrong way: stop and report on the screen.
- No tape ever confirmed: the car does not start driving, even if something approaches.
- Reverse is time limited. Nothing looks backwards, so keep free space behind the car.

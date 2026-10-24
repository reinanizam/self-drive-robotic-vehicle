# Wiring

## Arduino Uno pins

| Function | Uno pin | Notes |
|---|---|---|
| Motor bank A, enable (PWM) | D5 (`ENA`) | LEFT side |
| Motor bank A, direction | D4 (`IN1`), D7 (`IN2`) | L298N OUT1 / OUT2. Forward = `IN1` LOW, `IN2` HIGH |
| Motor bank B, enable (PWM) | D6 (`ENB`) | RIGHT side |
| Motor bank B, direction | D8 (`IN3`), D9 (`IN4`) | L298N OUT3 / OUT4. Forward = `IN3` LOW, `IN4` HIGH |
| Sonar HC-SR04 | A0 (`TRIG`), A1 (`ECHO`) | |
| IR line bar, 5 channels | A2, D13, D12, D11, D10 | channel 0 is the leftmost; `INPUT_PULLUP`, black tape reads LOW |
| I2C bus | A4 (SDA), A5 (SCL) | 16x2 LCD at 0x27 and MPU6050 at 0x68 (0x69 also accepted) |
| Brake LED | D2 | |
| Buzzer | D3 | |
| Pi link | USB | 115200 baud |

The MPU6050 sits on the car with its Z axis pointing up. The Uno calibrates it at start-up, so the car must stand still and upright when it is reset.

## Which side is which

The program assumes bank A (`ENA`, `IN1`, `IN2`, L298N OUT1/OUT2) drives the LEFT wheels and bank B (`ENB`, `IN3`, `IN4`, OUT3/OUT4) the RIGHT wheels. It is set by one line:

```cpp
const bool MOTOR_A_IS_LEFT = true;
```

At start-up the boot line shows the setting: `SENSOR_FUSION_READY V7 A_LEFT ...`.

## Check the motors before the first drive (wheels off the floor)

1. **Forward:** with the car lifted, a forward command must turn all wheels forward. If one side runs backwards, swap that motor's two wires at the L298N terminal.
2. **Sides:** with the wheels raised and the Pi program running, hold the tape to the right of the car's centre in the camera's view. The LEFT wheels must run faster (the car steers right, toward the tape). If the RIGHT wheels run faster, the sides are swapped: either swap the two motor connectors between the banks on the L298N, or set `MOTOR_A_IS_LEFT` to `false` and upload again.
3. **Reverse:** the car must back off in a straight line. A car that curves or spins in reverse has a wiring fault (a motor bank with swapped polarity, or the two sides swapped), not a software fault. Check items 1 and 2 first. On the prototype the back-off code was right: the car curved in reverse until the wires between the motors and the motor driver were put in the correct places.

## Power

- Motors: their own battery on the L298N.
- Uno: powered by the Pi over USB, or by a power bank.
- Common ground between the L298N, the Uno and the sensors.

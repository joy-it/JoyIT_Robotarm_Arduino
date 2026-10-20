# Joy-IT Robot arm Arduino

Arduino library for the **Joy-IT Robot02** robot arm with six servos and the **ARD-Moto3 Motorino** servo controller. Control individual joints, move the arm between poses over a specified duration, or operate it using the **SBC-WLGamepad** wireless controller.

## Features

- Six logical servo indices with configurable Motorino channel assignments
- Direct angle commands for individual servos or all six servos
- Individual pulse-width calibration and configurable home positions
- Timed movements using linear angle interpolation
- Optional SBC-WLGamepad control through PsxNewLib and hardware SPI
- Access to the last commanded servo angles

## Requirements

- Joy-IT Robot02: robot arm
- Joy-IT ARD-Moto3 Motorino 
- A suitable servo power supply
- Arduino Uno or a compatible AVR board.
- Optional Joy-IT SBC-WLGamepad with its receiver.

Install these libraries using the Arduino IDE Library Manager:

| Library | Purpose |
| --- | --- |
| [`Adafruit PWM Servo Driver Library`](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/tree/master) | Adafruit PCA9685 PWM Servo Driver Library |
| [`Adafruit BusIO`](https://github.com/adafruit/Adafruit_BusIO) | dependency of `Adafruit PWM Servo Driver Library` |
| [`PsxNewLib`](https://github.com/SukkoPera/PsxNewLib/tree/master) | PlayStation Controller Interface |
| [`DigitalIO`](https://github.com/greiman/DigitalIO) | dependency of `PsxNewLib` |

The controller dependencies are required to compile the current library even when gamepad control is disabled. `Wire` and `SPI` are provided by the Arduino board package.

## Connections

Connect the Motorino to the Arduino's I2C interface and power it according to its hardware documentation. The examples use I2C address **0x40** and Motorino channels **0–5**, ordered from the bottom of the arm to the top. The channels can be changed easily.

The library configures a **25 MHz external oscillator** and a nominal **50 Hz** servo update rate. These settings are intended for the Motorino board; a different PCA9685 board may require different clock settings.

For the SBC-WLGamepad receiver, the Arduino Uno connections are:

| Receiver connection | Arduino Uno |
| --- | --- |
| Pin 1 — Data | D12 / MISO |
| Pin 2 — Command | D11 / MOSI |
| Pin 3 | --- |
| Pin 4 — Ground | GND |
| Pin 5 — Power | 3.3 V |
| Pin 6 — Attention | D10 |
| Pin 7 — Clock | D13 / SCK |
| Pin 8 | --- |
| Pin 9 | --- |

> [!NOTE]
> The Attention pin is fixed to **D10** by `PsxControllerHwSpi<10>` in `Robotarm.h`. On other AVR boards, use that board's hardware SPI pins for Data, Command, and Clock.

See the [SBC-WLGamepad manual](https://joy-it.net/files/files/Produkte/SBC-WLGamepad/SBC-WLGamepad_Manual_2024-03-27.pdf) for receiver pin identification and setup.

## Functions of the repository

### Pulse width of servo motors

By default, **0° maps to 500 µs** and **180° maps to 2500 µs**. With this calibration, 90° corresponds to 1500 µs.

```cpp
// Configure a single servo.
robotarm.setServoCalibration(0, 500, 2500);

// Apply the same calibration to all six servos.
robotarm.setPulseWidthOfAllServos(500, 2500);
```

> [!NOTE]
> These functions configure the pulse-width mapping; they do not move the servos. They are not PWM frequency setters.

### Home positions

The default home angles, in servo-index order, are:

```cpp
{90, 90, 20, 110, 30, 150}
```

Change a home position at runtime and command it as follows:

```cpp
robotarm.setHomePosition(2, 30);
// move servo 2 to newly set home position
robotarm.resetPosition(2);

// Command all servo motors to home positions immediately
robotarm.resetAllPositions();
```

The home positions and calibration changes are stored in RAM and are lost when the Arduino restarts. Home positions are commanded positions, not positions detected by a homing procedure.


> [!NOTE]
> Use pulse widths and angles appropriate for your servos and the assembled arm. The software clamps angles to 0–180° but does not detect mechanical end stops, collisions, or excessive load.

### Timed movements

Use `moveTo()` for one servo or `moveAllTo()` for an entire pose. Both start a movement and return immediately. Call `update()` frequently to generate the intermediate angle commands.

> [!NOTE]
> After initialization, first establish a known commanded position, for example with `resetAllPositions()`. Timed movements return `false` if the required starting positions are unknown.

```cpp
// In setup(), after begin():
robotarm.resetAllPositions();
robotarm.moveTo(0, 110, 2000);

// In loop():
robotarm.update();
```

A complete pose contains six absolute target angles:

```cpp
const uint8_t targetAngles[Robotarm::SERVO_COUNT] = {
    100, 85, 25, 115, 35, 145
};

robotarm.moveAllTo(targetAngles, 3000);
```

Use `isMoving()` to determine whether a timed movement remains active. A duration of `0` applies the target immediately. Direct angle commands cancel timed movement for the affected servo. Avoid long `delay()` calls while a timed movement is in progress, because interpolation only advances when `update()` runs.

### Gamepad control

After `begin()` and the initial position commands, enable the controller:

```cpp
if (!robotarm.useExternalController(true)) {
    Serial.println(F("Gamepad initialization failed."));
}
```

Keep the following in `loop()`:

```cpp
robotarm.update();
```

| Control | Servo index | Direction |
| --- | --- | --- |
| Left stick horizontal | 0 | Right increases the angle |
| Left stick vertical | 1 | Up increases the angle |
| Right stick vertical | 2 | Up increases the angle |
| Right stick horizontal | 3 | Right increases the angle |
| L1 / R1 | 4 | L1 increases; R1 decreases |
| L2 / R2 | 5 | L2 increases; R2 decreases |

The receiver is polled at intervals of at least 20 ms. Stick inputs use a dead zone around their center position. Controller commands change the stored angle incrementally; actual responsiveness also depends on integer angle rounding and how frequently `update()` is called. A controller command overrides a timed movement on the affected servo.

Disable controller input with:

```cpp
robotarm.useExternalController(false);
```

If a controller read fails, the current implementation stops polling it. Call `useExternalController(true)` again to attempt reinitialization. Use analog mode for the sticks; if the controller is not recognized, consult the receiver manual's Mode-button instructions.

### Reading positions

```cpp
uint8_t angle;
if (robotarm.getPositionOfServo(0, angle)) {
    Serial.println(angle);
}

uint8_t angles[Robotarm::SERVO_COUNT] = {};
robotarm.getPositionOfAllServo(angles);
```

Position values represent the **last commanded angles**. The library does not measure physical servo positions. `getPositionOfServo()` returns `false` for an invalid servo index or an unknown position. The bulk getter leaves output entries unchanged when their positions are unknown.

> [!NOTE]
> Direct calls to `motorino.writeMicroseconds()` bypass the library's position tracking. Establish the commanded positions through the Robotarm API before starting relative or timed movements.

## API reference

Servo arguments are logical indices **0–5**, independent of the Motorino channel numbers. Array arguments must provide space for six entries. The constructor copies the channel array; the referenced Motorino object must remain alive for the lifetime of the Robotarm object. Call `begin()` successfully before commanding any movement.

| Method | Return | Description |
| --- | --- | --- |
| `Robotarm(motorino, channels)` | — | Construct an arm using a driver reference and six channel numbers. |
| `begin()` | `bool` | Initialize the Motorino. Returns `false` if driver initialization fails. Does not command home positions. |
| `useExternalController(external = true)` | `bool` | Enable or disable gamepad input. When called after initialization, enabling returns the controller initialization result. Before `begin()`, it only stores the requested mode. |
| `setServoCalibration(servo, minPulseUs, maxPulseUs)` | `bool` | Set pulse widths for 0° and 180°. Rejects an invalid servo index or a maximum not greater than the minimum. |
| `setPulseWidthOfAllServos(minPulseUs, maxPulseUs)` | `bool` | Apply the same calibration to all six servos. |
| `setAngleOfServo(servo, angle)` | `void` | Command one angle immediately. |
| `setAngleOfAllServos(angle)` | `void` | Command the same angle on all six servos. |
| `getPositionOfServo(servo, angle)` | `bool` | Write a known commanded angle into the supplied reference. |
| `getPositionOfAllServo(angles)` | `void` | Copy known commanded angles into a six-element buffer. |
| `setHomePosition(servo, angle)` | `void` | Configure one home angle without moving the servo. |
| `resetPosition(servo)` | `void` | Command one configured home angle immediately. |
| `resetAllPositions()` | `void` | Command all configured home angles immediately. |
| `moveTo(servo, angle, durationMs)` | `bool` | Start a timed movement from a known commanded position. |
| `moveAllTo(angles, durationMs)` | `bool` | Start timed movements for all six servos. Requires six known starting positions. |
| `update()` | `void` | Advance timed movements and poll the enabled controller. |
| `isMoving()` | `bool` | Report whether any timed movement is active. Does not report physical motion or gamepad activity. |

`Robotarm::SERVO_COUNT` is `6`. Angle parameters use `uint8_t`; pass values from 0 to 180. Larger representable angles are clamped, but negative numbers must not be passed to these unsigned parameters.

## Included examples

| Sketch | Purpose |
| --- | --- |
| `example/robotarm_simple_example/robotarm_simple_example.ino` | Alternate servo 0 between 20° and 160° using direct angle commands. |
| `example/robotarm_movement_over_duration/robotarm_movement_over_duration.ino` | Run three poses relative to the home angles with timed movements and pauses. |
| `example/robotarm_with_controller/robotarm_with_controller.ino` | Control all six servos with the gamepad and print commanded positions at 115200 baud. |
| `example/calibrate_motors_mounting/calibrate_motors_mounting.ino` | Output fixed pulse widths to position servos during assembly. |

In the supplied assembly sketch, indices 0–4 receive 1500 µs, then index 5 is set to 1600 µs. Check that assignment against your intended gripper channel before using it for assembly.

## Links

- [GitHub repository](https://github.com/joy-it/JoyIT_Robotarm_Arduino)
- [Joy-IT](https://joy-it.net/)

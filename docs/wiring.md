# Wiring & Pin Map

## ESP32 servo pin map (current)

| GPIO | Joint | Servo | Notes |
|---|---|---|---|
| 33 | Base rotation | MG90S | Default/rest 90°. Metal-geared — move slowly to protect gears. (Replaces earlier wrist-roll assignment.) |
| 32 | Gripper | MG90S | Not yet built |
| 13 | Upper arm | MG996R (converted to positional) | Rest position 160°, never commanded above 160° |
| 14 | Lower arm / shoulder — right | MG996R (converted to positional) | Rest position 10° |
| 27 | Lower arm / shoulder — left | MG996R (converted to positional) | Mirrored: left = 180 − right. Rest position 170° |
| 26 | Gripper twist | MG90S | |
| 25 | Gripper spin | MG90S | |
| 12 | — | — | Deliberately skipped (strapping pin) |

The shoulder pair (GPIO14 + GPIO27) is mounted facing each other so their
shafts share one axis.

## Power

- 5V 10A supply on soldered perfboard rails with bulk capacitors, driving all
  seven servos directly from ESP32 GPIO (no PCA9685)
- ESP32 powered separately from a mobile charger or laptop, with a common
  ground to the servo rail
- Two buck converters used to split load: heavy joints (originally the
  DS3218/MG996R pair) on one rail, light joints + logic on the other, for
  load isolation and spare redundancy

## Control input

- B10K linear potentiometers available on hand for servo testing
- Xbox controller paired to the ESP32 over Bluetooth via Bluepad32 (manual
  mode) — known to disconnect frequently due to Bluetooth dropouts (not
  ESP32 resets)
- D-pad Up/Down = shoulder, Left/Right = upper arm, RB/LB = tilt, RT/LT =
  spin, hold A/B = ease to rest, Y = limp, hold X = base to rest (90°)

## Servo history / gotchas

- Every MG996R-class servo initially received turned out to be
  continuous-rotation (360°) units despite being sold as 180° positional —
  confirmed by opening them up
- One of the four original MG996R units was dead (confirmed via wire/channel
  swap)
- Fix: moved the metal gears from the CR/360 servos into fake 180° plastic
  servos that still had working potentiometers (verified by position-hold
  test and multimeter) — this is how the arm joints became positional again

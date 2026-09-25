# Jeeves

An autonomous bed-making robot built on ESP32: it locates itself using **UWB (ultra-wideband) triangulation**, tracks heading with an **onboard IMU**, drives toward a target with proportional control, and uses a **winch mechanism** to pull and tuck bedding.

## How it works

Four UWB anchors are placed at known positions (e.g. the corners of a bed). The robot polls distance readings from each anchor over UART and solves a least-squares triangulation to estimate its `(x, y)` position. An MPU6050 IMU provides heading, which the robot combines with its position estimate and the bearing to a goal tag to plan movement.

Control runs as a single-threaded state machine in `loop()`:

1. Estimate position (triangulation) and compute distance/bearing to the goal.
2. If not yet arrived, turn to face the goal using proportional heading control.
3. Drive forward in proportional-length steps (duration scales with remaining distance).
4. Actuate the winch and wheel-lift servos to perform the bed-making motion.
5. Repeat until within the arrival threshold, then hold and continue winching.

An earlier version of this project used a FreeRTOS split (a UWB/triangulation task feeding a motor-control task over a queue); the current firmware consolidates this into the single-threaded loop above for simpler tuning of the drive/turn control loops.

## Hardware

- **MCU**: ESP32
- **Localization**: UWB ranging module (RYUW), 4 fixed anchors
- **Heading**: MPU6050 IMU
- **Drive**: TB6612FNG dual motor driver + 2 servos (wheel lift/wiggle)
- **Bed-making mechanism**: 4-wire stepper-driven winch (2048 steps/rev)

## Repository layout

| Path | Purpose |
|---|---|
| `src/`, `include/` | PlatformIO firmware: `Robot`, `Anchor`, `Triangulation`, `Motors`, `Winch`, `MPU6050`, `Filter` |
| `TriangulationVisualizer.py` | Plots live `(x, y)` position streamed over serial, for debugging triangulation |
| `.ino Files/` | Earlier Arduino IDE sketches (`Anchor`, `Robot`, `AnchorRobot`), superseded by the PlatformIO project but kept for reference |

## Building

This is a [PlatformIO](https://platformio.org/) project targeting the `esp32dev` board.

```bash
pio run              # build
pio run -t upload    # flash
pio device monitor    # serial output (115200 baud)
```

To visualize position output live, run `TriangulationVisualizer.py` while the robot streams `x,y` over serial.

## Status / roadmap

- [ ] Tune proportional drive/turn gains for reliable navigation
- [ ] Replace raw triangulation fixes with a Kalman filter to reduce noise
- [ ] Re-enable multi-anchor triangulation for full-course navigation (current build steps off a single tag's distance for the final approach)

# Anchor Robot (FreeRTOS)

Single ESP32 sketch that runs **UWB triangulation** and **motor control** as separate FreeRTOS tasks, connected by a position queue.

## Architecture

```
    UWB (Serial poll)          Position Queue           Task_Motor
         │                          │                        │
         ▼                          ▼                        ▼
   Task_UWB  ──►  (x, y) valid  ──►  overwrite  ──►  pursueTarget()
   (lower prio)   triangulate()      queue len 1     (higher prio, 50 ms)
```

- **Task_UWB** (priority 1): Polls 4 tags over RYUW UWB, runs least-squares triangulation, sends `(x, y)` to the queue with `xQueueOverwrite` so the motor always gets the latest position.
- **Task_Motor** (priority 2): Every 50 ms reads the latest position (non-blocking), updates `robotX`/`robotY`, runs `pursueTarget()` toward `goalX`/`goalY`.

Interrupts are **not** used for triangulation; all UWB and math run inside the UWB task. If you add a UWB "data ready" interrupt later, it should only set a flag or give a semaphore; the task should do the actual read and triangulation.

## Hardware

- Same UWB (RYUW) and motor (TB6612FNG) wiring as in `Anchor/` and `Robot/`.
- Flash this sketch on the **anchor** ESP32 that has both UWB and motors.

## Configuration

- **Tag positions** (bed corners): edit the `anchors[]` array in `setup()` to match your bed layout (these are the known positions used for triangulation).
- **Goal**: `goalX`, `goalY` are set in `setup()` (e.g. to one tag corner). You can later drive toward each tag in sequence by changing goal from another task or over Serial.

## Building

- Open `AnchorRobot` in Arduino IDE (or open `AnchorRobot.ino`).
- Board: ESP32 (any variant).
- Compile and upload.

## Optional: UWB data-ready interrupt

To avoid polling Serial in a loop, you can:

1. Use a GPIO or UART RX interrupt when UWB has data.
2. In the ISR: set a flag or `xSemaphoreGiveFromISR(dataReadySem)`.
3. In `taskUWB`: block on the semaphore instead of spinning; when signaled, read one response and continue. Keep all parsing and triangulation in the task.

This keeps ISRs short and leaves timing and complexity in the RTOS tasks.

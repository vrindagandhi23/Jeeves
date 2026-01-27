# Jeeves
## Anchor
Anchor holds all the code for UWB and triangulation control. We currently have code to triangulate the position of a UWB tag and visualize it in Python.
## Robot
Robot holds all the code for controlling and moving the robot. We currently have code to move the wheels up and down and drive the robot around.
## To Do
1. Combine anchor and robot into one thing, so we can give the robot a position and get the robot to drive to that position.
2. Implement RTOS to thread triangulation, movement, and other robot tasks
3. Implement Kalman Filter to reduce noise on triangulation code.


# Robot Arm H25 v2

A robot arm that picks up an object from the left or right side and drops it in the middle.

Model: [MOC-81936 by Benoit72](https://rebrickable.com/mocs/MOC-81936/Benoit72/robot-arm/#details) (31313 version)

Code: Pybricks version of the LEGO EV3 Education Robot Arm H25 program, adapted by Jim Bolanos.

## Hardware

- Brick: EV3 (Pybricks beta firmware)
- Port A: medium motor (gripper)
- Port B: large motor (elbow)
- Port C: large motor (base)
- Port S1: touch sensor (elbow home)
- Port S2: touch sensor (base home)

## How it works

1. The arm calibrates itself (elbow, base and gripper).
2. The screen shows "?" and the light blinks orange.
3. Press RIGHT or LEFT to choose a side.
4. Press CENTER to confirm.
5. The arm picks up the object on that side and drops it in the middle.

To stop the program, press UP + DOWN together.

## Code

- `main.py`: main program (settings at the top to tune speeds and positions)

## Build instructions

See the [Rebrickable page](https://rebrickable.com/mocs/MOC-81936/Benoit72/robot-arm/#details).

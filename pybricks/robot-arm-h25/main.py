# Robot Arm H25 v2 (MOC-81936 by Benoit72, 31313 version)
# Pybricks (EV3 beta firmware)
# Author: Jim Bolanos
#
# How it works:
#   1. The arm calibrates itself (elbow, base and gripper).
#   2. The screen shows "?" and the light blinks orange.
#   3. Press RIGHT or LEFT to choose a side.
#   4. Press CENTER to confirm.
#   5. The arm picks up the object on that side and drops it in the middle.
#   To stop the program, press UP + DOWN together.
#
# Ports:
#   A = gripper (medium motor)     S1 = touch sensor (elbow home)
#   B = elbow (large motor)        S2 = touch sensor (base home)
#   C = base (large motor)

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor
from pybricks.parameters import Port, Stop, Button, Color
from pybricks.tools import wait


# ------------------------------------------------------------------
# Setup
# ------------------------------------------------------------------

ev3 = EV3Brick()

gripper_motor = Motor(Port.A)
elbow_motor = Motor(Port.B)
base_motor = Motor(Port.C)

elbow_switch = TouchSensor(Port.S1)
base_switch = TouchSensor(Port.S2)

# CENTER is used to confirm, so the program stops with UP + DOWN instead
ev3.system.set_stop_button((Button.UP, Button.DOWN))


# ------------------------------------------------------------------
# Settings (change these to tune the arm)
# ------------------------------------------------------------------

# Speeds in degrees per second, measured at the motor
GRIPPER_SPEED = 450
ELBOW_SPEED = 500
BASE_SPEED = 500

# Slower speed used only at the start, when the arm looks for its
# touch sensors, so it does not hit them too hard
CALIBRATION_SPEED = 250

# How fast the motors speed up and slow down (degrees per second squared)
ACCELERATION = 900

elbow_motor.control.limits(speed=ELBOW_SPEED, acceleration=ACCELERATION)
base_motor.control.limits(speed=BASE_SPEED, acceleration=ACCELERATION)

# Elbow positions, in motor degrees from the top (touch sensor)
ELBOW_UP = 0
ELBOW_DOWN = 174

# Base positions, in motor degrees from the touch sensor
RIGHT = -120
MIDDLE = -300
LEFT = -480

# How far the gripper opens, in degrees from fully closed
GRIPPER_OPEN = 150

# Thickness of the lines used to draw the symbols on the screen
LINE = 10


# ------------------------------------------------------------------
# Screen
# The biggest font is only 16 pixels tall, so the symbols are
# drawn with thick lines. The screen is 178 x 128 pixels.
# ------------------------------------------------------------------

def show_question():
    # Draw a big "?"
    ev3.screen.clear()
    ev3.screen.draw_line(64, 24, 114, 24, LINE)    # top
    ev3.screen.draw_line(114, 24, 114, 60, LINE)   # right side
    ev3.screen.draw_line(114, 60, 89, 60, LINE)    # middle
    ev3.screen.draw_line(89, 60, 89, 82, LINE)     # stem
    ev3.screen.draw_circle(89, 102, 7, fill=True)  # dot


def show_arrow(side):
    # Draw a big ">" for right or "<" for left
    ev3.screen.clear()
    if side == RIGHT:
        ev3.screen.draw_line(64, 24, 114, 64, LINE)
        ev3.screen.draw_line(114, 64, 64, 104, LINE)
    else:
        ev3.screen.draw_line(114, 24, 64, 64, LINE)
        ev3.screen.draw_line(64, 64, 114, 104, LINE)


# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------

def wait_button(button):
    # Wait until the button is pressed and then released
    while button not in ev3.buttons.pressed():
        wait(10)
    while button in ev3.buttons.pressed():
        wait(10)


def initialize():
    # Elbow: move up until it presses its touch sensor.
    # That position becomes angle 0.
    elbow_motor.run(-CALIBRATION_SPEED)
    while not elbow_switch.pressed():
        wait(10)
    elbow_motor.reset_angle(0)
    elbow_motor.hold()

    # Base: turn until it presses its touch sensor.
    # That position becomes angle 0.
    base_motor.run(CALIBRATION_SPEED)
    while not base_switch.pressed():
        wait(10)
    base_motor.reset_angle(0)
    base_motor.hold()

    # Gripper: close until it can't move anymore (angle 0), then open
    gripper_motor.run_until_stalled(-GRIPPER_SPEED, then=Stop.COAST, duty_limit=50)
    gripper_motor.reset_angle(0)
    gripper_motor.run_target(GRIPPER_SPEED, GRIPPER_OPEN)

    # Wait in the middle
    base_motor.run_target(BASE_SPEED, MIDDLE)

    # Three beeps: ready
    for i in range(3):
        ev3.speaker.beep()
        wait(100)


def robot_pick(position):
    # Go to the position, lower the arm, grab the object, raise the arm
    base_motor.run_target(BASE_SPEED, position)
    elbow_motor.run_target(ELBOW_SPEED, ELBOW_DOWN)
    gripper_motor.run_until_stalled(-GRIPPER_SPEED, then=Stop.HOLD, duty_limit=50)
    elbow_motor.run_target(ELBOW_SPEED, ELBOW_UP)


def robot_release(position):
    # Go to the position, lower the arm, open the gripper, raise the arm
    base_motor.run_target(BASE_SPEED, position)
    elbow_motor.run_target(ELBOW_SPEED, ELBOW_DOWN)
    gripper_motor.run_target(GRIPPER_SPEED, GRIPPER_OPEN)
    elbow_motor.run_target(ELBOW_SPEED, ELBOW_UP)


# ------------------------------------------------------------------
# Main program
# ------------------------------------------------------------------

initialize()

while True:
    # Ask the user to choose a side
    show_question()
    ev3.light.blink(Color.ORANGE, [300, 300])

    side = None
    while side is None:
        pressed = ev3.buttons.pressed()
        if Button.RIGHT in pressed:
            side = RIGHT
        if Button.LEFT in pressed:
            side = LEFT
        wait(10)

    # Show the chosen side and wait for CENTER to confirm
    ev3.light.off()
    ev3.speaker.beep(800, 200)
    show_arrow(side)
    wait_button(Button.CENTER)

    # Pick up the object on that side and drop it in the middle
    ev3.light.on(Color.GREEN)
    robot_pick(side)
    robot_release(MIDDLE)
    ev3.light.off()
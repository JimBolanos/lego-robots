#!/usr/bin/env pybricks-micropython
# DOODLEBOT (LEGO 42069 + EV3 31313) - Pybricks port of the EV3-G programs
# Author: Jim Bolanos

from math import sin, cos, acos, atan, sqrt, radians, degrees
from random import randint
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, InfraredSensor
from pybricks.parameters import Port, Button, Color
from pybricks.tools import wait, StopWatch

# 1 = move the pen with the IR remote      (DOODLEBOT 01)
# 2 = pen + face with the IR remote        (DOODLEBOT 02)
# 3 = draws a face by itself               (DOODLEBOT 03)
MODE = 3

ev3 = EV3Brick()
head = Motor(Port.A)    # medium motor: face and pen lift
arm1 = Motor(Port.B)    # large motors: the two arms
arm2 = Motor(Port.C)
touch = TouchSensor(Port.S1)
color = ColorSensor(Port.S3)
ir = InfraredSensor(Port.S4)

# Arm geometry and calibration (same values as the "Initialize" block)
L1, L2, L3, L4 = 17, 20, 20, 17        # link lengths
P1X, P1Y, P5X, P5Y = 25, 5, 41, 5      # arm pivots
J1_OFFSET, J2_OFFSET = 34, -3
MIN_X, MAX_X, MIN_Y, MAX_Y = 22, 44, 23, 42   # pen limits
OFFSET_X = 18
SPEED = 2       # drawing speed (1 to 5)
CYCLE = 20      # control cycle in ms

# Face number for each IR direction (dx, dy)
FACE = {(0, 0): 1, (0, -1): 2, (-1, -1): 3, (-1, 0): 4, (-1, 1): 5,
        (0, 1): 6, (1, 1): 7, (1, 0): 8, (1, -1): 9}

x = y = 0                 # pen position
j1 = j2 = 90              # target angle of each arm
head_pos = cur_head_pos = target_pos = 0   # 0 = pen up, 1-9 = pen down + face
head_ok = reg_done = False
ticks = 0
timer = StopWatch()


def clamp(v, low, high):
    return max(low, min(high, v))


def cosd(a):
    return cos(radians(a))


def sind(a):
    return sin(radians(a))


def angle(a, b, c):
    """Angle in degrees between sides a and b, opposite to side c."""
    v = (a * a + b * b - c * c) / (2 * a * b)
    return degrees(acos(v)) if -1 <= v <= 1 else float("nan")


def forward_kinematics(a1, a2):
    """Arm angles -> pen position."""
    e1x, e1y = P1X + L1 * cosd(a1), P1Y + L1 * sind(a1)
    e4x, e4y = P5X + L4 * cosd(a2), P5Y + L4 * sind(a2)
    d = sqrt((e4x - e1x) ** 2 + (e4y - e1y) ** 2)
    a = (L2 * L2 + d * d - L3 * L3) / (2 * d)
    h = sqrt(L2 * L2 - a * a)
    px = e1x + a / d * (e4x - e1x) - h / d * (e4y - e1y)
    py = e1y + a / d * (e4y - e1y) + h / d * (e4x - e1x)
    return px, py


def inverse_kinematics(px, py):
    """Pen position -> arm angles (nan if the pen can't reach it)."""
    d1 = sqrt((px - P1X) ** 2 + (py - P1Y) ** 2)
    d5 = sqrt((px - P5X) ** 2 + (py - P5Y) ** 2)
    d15 = sqrt((P5X - P1X) ** 2 + (P5Y - P1Y) ** 2)
    a1 = angle(d1, d15, d5) + angle(L1, d1, L2)
    a2 = 180 - angle(d5, L4, L3) - angle(d15, d5, d1)
    return a1, a2


def move_pen(nx, ny):
    """Set a new pen target. Beep and undo it if it is out of bounds."""
    global x, y, j1, j2
    old = (x, y, j1, j2)
    x, y = nx, ny
    j1, j2 = inverse_kinematics(x, y)
    inside = (MIN_X <= x <= MAX_X and MIN_Y <= y <= MAX_Y
              and 0 <= j1 <= 180 and 0 <= j2 <= 180)
    if not inside:
        ev3.speaker.beep(440, 50)
        x, y, j1, j2 = old


def control_arms():
    """Move both arms to their targets (P controller with a dead band)."""
    global reg_done
    moving = False
    for motor, target, offset, gear in ((arm1, j1, J1_OFFSET, 3),
                                        (arm2, j2, J2_OFFSET, -3)):
        error = ((target - offset) * gear - motor.angle()) * 1.5
        speed = clamp(error, -50, 50)
        if abs(speed) > 1:
            motor.run(speed * 10)
            moving = True
        else:
            motor.brake()
    reg_done = not moving


def control_head():
    """Move the head to the position of the current face."""
    global cur_head_pos, target_pos, head_ok
    if head_pos != cur_head_pos:
        cur_head_pos = head_pos
        target_pos = head_pos * 36 * 3
    speed = clamp(target_pos - head.angle(), -50, 50)
    if abs(speed) > 1:
        head.run(speed * 10)
        head_ok = False
    else:
        head.stop()
        head_ok = True


def show_position():
    ev3.screen.clear()
    ev3.screen.draw_text(10, 30, "X: {:.1f}".format(x - MIN_X))
    ev3.screen.draw_text(10, 70, "Y: {:.1f}".format(y - MIN_Y))


def wait_cycle():
    wait(max(0, CYCLE - timer.time()))
    timer.reset()


def tick():
    """One control cycle: wait, run the motor controllers, update the screen."""
    global ticks
    wait_cycle()
    control_arms()
    if MODE > 1:
        control_head()
    ticks += 1
    if ticks % 5 == 0:
        show_position()


def wait_ticks(ms):
    t = StopWatch()
    while t.time() < ms:
        tick()


def wait_arms():
    tick()
    while not reg_done:
        tick()


def reset_arms():
    arm2.dc(35)
    arm1.dc(-25)
    wait(1000)
    while not touch.pressed():
        wait(10)
    wait(500)
    arm2.reset_angle(0)
    arm2.brake()
    ev3.speaker.beep(550, 200)
    arm1.dc(-35)
    wait(1000)
    while abs(arm1.speed()) > 50:     # wait until the arm hits its stop
        wait(10)
    arm1.reset_angle(0)
    wait(500)
    arm1.brake()
    ev3.speaker.beep(550, 200)
    arm1.run_angle(250, 170, wait=False)    # go to the start position
    arm2.run_angle(250, -280)


def reset_head():
    head.run(-250)
    count = 0
    while count <= 100:               # wait until the sensor sees red
        count = count + 1 if color.color() == Color.RED else 0
        wait(10)
    head.brake()
    ev3.speaker.beep(550, 200)
    head.run_angle(250, -475)
    head.reset_angle(0)
    ev3.speaker.beep(550, 200)


def lift_pen(lift):
    global head_pos
    if lift:
        if head_pos == 0 and head_ok:
            return
        head_pos = 0
    else:
        if head_pos != 0 and head_ok:
            return
        head_pos = randint(1, 9)      # a random face each time the pen goes down
    wait_ticks(500)
    while not head_ok:
        tick()


def feed_line(x1, y1, x2, y2, lift):
    """Move the pen from (x1, y1) to (x2, y2) at a constant speed."""
    dx, dy = x2 - x1, y2 - y1
    if abs(dx) < 0.1 and abs(dy) < 0.1:
        return
    lift_pen(lift)
    steps = max(1, round(sqrt(dx * dx + dy * dy) / SPEED / (CYCLE / 1000)))
    for i in range(1, steps + 1):
        tick()
        move_pen(x1 + dx * i / steps, y1 + dy * i / steps)
    wait_arms()


def draw_line(x1, y1, x2, y2):
    x1, x2 = x1 + OFFSET_X, x2 + OFFSET_X
    y1, y2 = y1 + MIN_Y, y2 + MIN_Y
    feed_line(x, y, x1, y1, True)     # go to the start with the pen up
    feed_line(x1, y1, x2, y2, False)  # draw


def draw_rectangle(rx, ry, w, h):
    draw_line(rx, ry, rx + w, ry)
    draw_line(rx + w, ry, rx + w, ry + h)
    draw_line(rx + w, ry + h, rx, ry + h)
    draw_line(rx, ry + h, rx, ry)


def draw_circle(cx, cy, r):
    cx, cy = cx + OFFSET_X, cy + MIN_Y
    feed_line(x, y, cx, cy - r, True)  # go to the bottom of the circle
    lift_pen(False)
    step = degrees(atan(SPEED / r)) * CYCLE / 1000   # degrees per cycle
    steps = round(360 / step)
    for i in range(1, steps + 1):
        tick()
        a = 360 * i / steps - 90
        move_pen(cx + cosd(a) * r, cy + sind(a) * r)
    wait_arms()


def ir_control():
    """Move the pen with the IR remote. The beacon button lifts/lowers it."""
    global head_pos
    last_beacon = False
    while True:
        b = ir.buttons(1)
        red_up = Button.LEFT_UP in b          # remote button 1
        red_down = Button.LEFT_DOWN in b      # remote button 2
        blue_up = Button.RIGHT_UP in b        # remote button 3
        blue_down = Button.RIGHT_DOWN in b    # remote button 4
        dx = clamp(red_up + red_down - blue_up - blue_down, -1, 1)
        dy = clamp(red_down + blue_down - red_up - blue_up, -1, 1)
        if MODE == 2:
            beacon = Button.BEACON in b
            if beacon and not last_beacon:
                head_pos = 1 if cur_head_pos == 0 else 0
            last_beacon = beacon
            if head_pos != 0:
                head_pos = FACE[(dx, dy)]
        tick()
        if dx or dy:
            move_pen(x + dx * SPEED / 15, y + dy * SPEED / 15)


# ---------- main program ----------
if MODE > 1:
    reset_head()
reset_arms()
x, y = forward_kinematics(j1, j2)

if MODE == 3:
    draw_circle(15, 10, 6.5)
    draw_rectangle(11, 12.5, 3, 2)
    draw_rectangle(16, 10.5, 3, 2)
    draw_line(11, 8, 19, 8)
    draw_line(2, 2, 2, 2)             # move the pen out of the way
    head_pos = 9
    wait_ticks(5000)
else:
    ir_control()

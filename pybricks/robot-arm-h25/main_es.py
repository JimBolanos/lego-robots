# Robot Arm H25 v2 (MOC-81936 de Benoit72, version 31313)
# Pybricks (firmware beta para EV3)
# Autor: Jim Bolanos
#
# Como funciona:
#   1. El brazo se calibra solo (codo, base y pinza).
#   2. La pantalla muestra "?" y la luz parpadea en naranja.
#   3. Presiona DERECHA o IZQUIERDA para elegir un lado.
#   4. Presiona CENTRO para confirmar.
#   5. El brazo recoge el objeto de ese lado y lo suelta en el centro.
#   Para detener el programa, presiona ARRIBA + ABAJO al mismo tiempo.
#
# Puertos:
#   A = pinza (motor mediano)       S1 = sensor de toque (inicio del codo)
#   B = codo (motor grande)         S2 = sensor de toque (inicio de la base)
#   C = base (motor grande)

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor
from pybricks.parameters import Port, Stop, Button, Color
from pybricks.tools import wait


# ------------------------------------------------------------------
# Configuracion inicial
# ------------------------------------------------------------------

ev3 = EV3Brick()

gripper_motor = Motor(Port.A)
elbow_motor = Motor(Port.B)
base_motor = Motor(Port.C)

elbow_switch = TouchSensor(Port.S1)
base_switch = TouchSensor(Port.S2)

# CENTRO se usa para confirmar, asi que el programa se detiene con ARRIBA + ABAJO
ev3.system.set_stop_button((Button.UP, Button.DOWN))


# ------------------------------------------------------------------
# Ajustes (cambia estos valores para afinar el brazo)
# ------------------------------------------------------------------

# Velocidades en grados por segundo, medidas en el motor
GRIPPER_SPEED = 450
ELBOW_SPEED = 500
BASE_SPEED = 500

# Velocidad mas lenta, usada solo al inicio cuando el brazo busca sus
# sensores de toque, para no golpearlos fuerte
CALIBRATION_SPEED = 250

# Que tan rapido aceleran y frenan los motores (grados por segundo al cuadrado)
ACCELERATION = 900

elbow_motor.control.limits(speed=ELBOW_SPEED, acceleration=ACCELERATION)
base_motor.control.limits(speed=BASE_SPEED, acceleration=ACCELERATION)

# Posiciones del codo, en grados del motor desde arriba (sensor de toque)
ELBOW_UP = 0
ELBOW_DOWN = 174

# Posiciones de la base, en grados del motor desde el sensor de toque
RIGHT = -120
MIDDLE = -300
LEFT = -480

# Que tanto se abre la pinza, en grados desde totalmente cerrada
GRIPPER_OPEN = 150

# Grosor de las lineas usadas para dibujar los simbolos en la pantalla
LINE = 10


# ------------------------------------------------------------------
# Pantalla
# La fuente mas grande mide solo 16 pixeles de alto, asi que los
# simbolos se dibujan con lineas gruesas. La pantalla es de 178 x 128 pixeles.
# ------------------------------------------------------------------

def show_question():
    # Dibuja un "?" grande
    ev3.screen.clear()
    ev3.screen.draw_line(64, 24, 114, 24, LINE)    # parte de arriba
    ev3.screen.draw_line(114, 24, 114, 60, LINE)   # lado derecho
    ev3.screen.draw_line(114, 60, 89, 60, LINE)    # parte del medio
    ev3.screen.draw_line(89, 60, 89, 82, LINE)     # tallo
    ev3.screen.draw_circle(89, 102, 7, fill=True)  # punto


def show_arrow(side):
    # Dibuja un ">" grande para la derecha o un "<" para la izquierda
    ev3.screen.clear()
    if side == RIGHT:
        ev3.screen.draw_line(64, 24, 114, 64, LINE)
        ev3.screen.draw_line(114, 64, 64, 104, LINE)
    else:
        ev3.screen.draw_line(114, 24, 64, 64, LINE)
        ev3.screen.draw_line(64, 64, 114, 104, LINE)


# ------------------------------------------------------------------
# Funciones auxiliares
# ------------------------------------------------------------------

def wait_button(button):
    # Espera a que el boton se presione y luego se suelte
    while button not in ev3.buttons.pressed():
        wait(10)
    while button in ev3.buttons.pressed():
        wait(10)


def initialize():
    # Codo: sube hasta presionar su sensor de toque.
    # Esa posicion se convierte en el angulo 0.
    elbow_motor.run(-CALIBRATION_SPEED)
    while not elbow_switch.pressed():
        wait(10)
    elbow_motor.reset_angle(0)
    elbow_motor.hold()

    # Base: gira hasta presionar su sensor de toque.
    # Esa posicion se convierte en el angulo 0.
    base_motor.run(CALIBRATION_SPEED)
    while not base_switch.pressed():
        wait(10)
    base_motor.reset_angle(0)
    base_motor.hold()

    # Pinza: se cierra hasta que ya no puede moverse (angulo 0) y luego se abre
    gripper_motor.run_until_stalled(-GRIPPER_SPEED, then=Stop.COAST, duty_limit=50)
    gripper_motor.reset_angle(0)
    gripper_motor.run_target(GRIPPER_SPEED, GRIPPER_OPEN)

    # Espera en el centro
    base_motor.run_target(BASE_SPEED, MIDDLE)

    # Tres pitidos: listo
    for i in range(3):
        ev3.speaker.beep()
        wait(100)


def robot_pick(position):
    # Va a la posicion, baja el brazo, agarra el objeto y sube el brazo
    base_motor.run_target(BASE_SPEED, position)
    elbow_motor.run_target(ELBOW_SPEED, ELBOW_DOWN)
    gripper_motor.run_until_stalled(-GRIPPER_SPEED, then=Stop.HOLD, duty_limit=50)
    elbow_motor.run_target(ELBOW_SPEED, ELBOW_UP)


def robot_release(position):
    # Va a la posicion, baja el brazo, abre la pinza y sube el brazo
    base_motor.run_target(BASE_SPEED, position)
    elbow_motor.run_target(ELBOW_SPEED, ELBOW_DOWN)
    gripper_motor.run_target(GRIPPER_SPEED, GRIPPER_OPEN)
    elbow_motor.run_target(ELBOW_SPEED, ELBOW_UP)


# ------------------------------------------------------------------
# Programa principal
# ------------------------------------------------------------------

initialize()

while True:
    # Pide al usuario que elija un lado
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

    # Muestra el lado elegido y espera a que CENTRO confirme
    ev3.light.off()
    ev3.speaker.beep(800, 200)
    show_arrow(side)
    wait_button(Button.CENTER)

    # Recoge el objeto de ese lado y lo suelta en el centro
    ev3.light.on(Color.GREEN)
    robot_pick(side)
    robot_release(MIDDLE)
    ev3.light.off()

from machine import Pin, I2C, PWM
from time import sleep
from vl53l0x import VL53L0X

# =========================
# KONFIGURACJA
# =========================

# I2C: GP0 = SDA, GP1 = SCL
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=10000)

# XSHUT sensorów
xshut_left = Pin(12, Pin.OUT)
xshut_right = Pin(13, Pin.OUT)

DEFAULT = 0x29
ADDR_LEFT = 0x30
ADDR_RIGHT = 0x31

# 100 mm = 10 cm
THRESHOLD_MM = 100

# Prędkości silników: zakres 0-65535
FORWARD_SPEED = 35000
TURN_SPEED = 35000

# =========================
# L298N — SILNIKI
# =========================

# Kanał A — lewy silnik
ena = PWM(Pin(6))
in1 = Pin(2, Pin.OUT)
in2 = Pin(3, Pin.OUT)

# Kanał B — prawy silnik
enb = PWM(Pin(7))
in3 = Pin(4, Pin.OUT)
in4 = Pin(5, Pin.OUT)

ena.freq(1000)
enb.freq(1000)


def set_speed(left_speed, right_speed):
    ena.duty_u16(left_speed)
    enb.duty_u16(right_speed)


def stop():
    set_speed(0, 0)

    in1.value(0)
    in2.value(0)
    in3.value(0)
    in4.value(0)


def forward(speed=FORWARD_SPEED):
    in1.value(1)
    in2.value(0)

    in3.value(1)
    in4.value(0)

    set_speed(speed, speed)


def backward(speed=FORWARD_SPEED):
    in1.value(0)
    in2.value(1)

    in3.value(0)
    in4.value(1)

    set_speed(speed, speed)


def turn_left(speed=TURN_SPEED):
    in1.value(0)
    in2.value(1)

    in3.value(1)
    in4.value(0)

    set_speed(speed, speed)


def turn_right(speed=TURN_SPEED):
    in1.value(1)
    in2.value(0)

    in3.value(0)
    in4.value(1)

    set_speed(speed, speed)


# =========================
# VL53L0X — SENSORY
# =========================

def scan():
    return i2c.scan()


def change_address(old_addr, new_addr):
    i2c.writeto_mem(old_addr, 0x8A, bytes([new_addr & 0x7F]))
    sleep(0.5)


def set_sensor_address(sensor, addr):
    if hasattr(sensor, "_address"):
        sensor._address = addr
    if hasattr(sensor, "address"):
        sensor.address = addr
    if hasattr(sensor, "addr"):
        sensor.addr = addr


def init_sensors():
    xshut_left.value(0)
    xshut_right.value(0)
    sleep(1)

    xshut_left.value(1)
    sleep(1)

    change_address(DEFAULT, ADDR_LEFT)
    sleep(0.5)

    if ADDR_LEFT not in scan():
        stop()
        raise SystemExit

    xshut_right.value(1)
    sleep(1)

    devices = scan()
    if DEFAULT not in devices or ADDR_LEFT not in devices:
        stop()
        raise SystemExit

    change_address(DEFAULT, ADDR_RIGHT)
    sleep(0.5)

    devices = scan()
    if ADDR_LEFT not in devices or ADDR_RIGHT not in devices:
        stop()
        raise SystemExit

    left_sensor = VL53L0X(i2c, ADDR_LEFT)
    right_sensor = VL53L0X(i2c, ADDR_RIGHT)

    set_sensor_address(left_sensor, ADDR_LEFT)
    set_sensor_address(right_sensor, ADDR_RIGHT)

    sleep(1)

    return left_sensor, right_sensor


# =========================
# GŁÓWNY PROGRAM
# =========================

stop()

left_sensor, right_sensor = init_sensors()

while True:
    try:
        left_distance = left_sensor.read()
        sleep(0.15)

        right_distance = right_sensor.read()
        sleep(0.15)

        left_blocked = left_distance < THRESHOLD_MM
        right_blocked = right_distance < THRESHOLD_MM

        if right_blocked and not left_blocked:
            turn_left()

        elif left_blocked and not right_blocked:
            turn_right()

        elif left_blocked and right_blocked:
            stop()

        else:
            forward()

        sleep(0.2)

    except OSError:
        stop()
        sleep(1)

    except KeyboardInterrupt:
        stop()
        break
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
xshut_front = Pin(11, Pin.OUT)

DEFAULT = 0x29
ADDR_LEFT = 0x30
ADDR_RIGHT = 0x31
ADDR_FRONT = 0x32

# 100 mm = 10 cm
THRESHOLD_MM = 100

# Prędkości silników: zakres 0-65535
PWM_MAX = 65535
FORWARD_SPEED = 50000

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
    
def set_speed_fraction(left_speed_frac, right_speed_frac):
    ls = int(FORWARD_SPEED*left_speed_frac)
    rs = int(FORWARD_SPEED*right_speed_frac)
    ls = max(0, min(ls, PWM_MAX))
    rs = max(0, min(rs, PWM_MAX))
    set_speed(ls,rs)


def stop():
    set_speed(0, 0)

    in1.value(0)
    in2.value(0)
    in3.value(0)
    in4.value(0)

def set_forward():
    in1.value(1)
    in2.value(0)

    in3.value(1)
    in4.value(0)

def set_backward():
    in1.value(0)
    in2.value(1)

    in3.value(0)
    in4.value(1)


def set_left():
    in1.value(0)
    in2.value(1)

    in3.value(1)
    in4.value(0)

def set_right():
    in1.value(1)
    in2.value(0)

    in3.value(0)
    in4.value(1)

MAX_FRONT_DIST = 300
MIN_FRONT_DIST = 60
SIDE_DIST_OPT = 150
MAX_SIDE_DIST = 150
MIN_SIDE_DIST = 70
SCALING_SIDE = 0.01

def sum_power_func(front_dist):
    P = (front_dist - 60) * 2/240
    P = min(P,1)
    P = max(0,P)
    return P

def diff_power_func(side_dist):
    dist_error = side_dist - SIDE_DIST_OPT
    P = SCALING_SIDE * dist_error
    P = max(-1,P)
    P = min(1,P)
    return P

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
    print("left succ init")
    
    xshut_right.value(1)
    sleep(1)

    devices = scan()
    if DEFAULT not in devices or ADDR_LEFT not in devices:
        stop()
        raise SystemExit

    change_address(DEFAULT, ADDR_RIGHT)
    sleep(0.5)
    print("right succ init")
    
    xshut_front.value(1)
    sleep(0.5)
    
    #devices = scan()
    #if ADDR_LEFT not in devices or ADDR_RIGHT not in devices or DEFAULT not in devices:
    #    
    devices = scan()
    print(devices)
    if ADDR_FRONT not in devices and DEFAULT in devices:
        change_address(DEFAULT, ADDR_FRONT)
    
    if ADDR_FRONT not in devices and DEFAULT in devices:
        stop()
        raise SystemExit
    
    print(devices)
    print("all succ init")
      
    RR_sensor = VL53L0X(i2c, ADDR_LEFT)
    forward_sensor = VL53L0X(i2c, ADDR_RIGHT)
    RF_sensor = VL53L0X(i2c, ADDR_FRONT)

    set_sensor_address(RF_sensor, ADDR_FRONT)
    set_sensor_address(RR_sensor, ADDR_LEFT)
    set_sensor_address(forward_sensor, ADDR_RIGHT)

    sleep(1)

    return RR_sensor, forward_sensor, RF_sensor


# =========================
# GŁÓWNY PROGRAM
# =========================

stop()
set_forward()

RR_sensor, forward_sensor, RF_sensor = init_sensors()
from collections import deque

N_dist_avg = 3
RR_dists = deque([], N_dist_avg)
forward_dists = deque([], N_dist_avg)
RF_dists = deque([], N_dist_avg)

BACK_SPEED = 0.4
FRONT_FULL = 0.6
FRONT_HALF = 0.4
while True:
    try:
        RR_read = RR_sensor.read()
        RR_dists.append(RR_read)
        RR_distance = sum(RR_dists)/N_dist_avg
        #sleep(0.15)

        forward_read = forward_sensor.read()
        forward_dists.append(forward_read)
        forward_distance = sum(forward_dists)/N_dist_avg
        #sleep(0.15)
        #print(sum(RR_dists)/N_dist_avg, sum(forward_dists)/N_dist_avg)
        
        RF_read = RF_sensor.read() - 25
        RF_dists.append(RF_read)
        RF_distance = sum(RF_dists)/N_dist_avg
        
        print(RR_distance, RF_distance, forward_distance)
        
        right_distance = min(RR_distance, RF_distance)
        
        if forward_distance < MIN_FRONT_DIST:
            set_backward()
            P_L, P_R = (BACK_SPEED + 0.1, BACK_SPEED)
        
        else:
            set_forward()
            
            if right_distance < MIN_SIDE_DIST:
                P_L, P_R = (FRONT_HALF, FRONT_FULL)
            elif right_distance > MAX_SIDE_DIST:
                P_L, P_R = (FRONT_FULL, FRONT_HALF)
            else:
                P_L, P_R = (FRONT_FULL, FRONT_FULL)
                
        
        set_speed_fraction(P_L, P_R)
        print(P_L, P_R)
        
        #P_forward = sum_power_func(forward_disantce)
        #P_diff = diff_power_func(right_distance)
        
        #P_L = P_forward*(1+P_diff)/2
        #P_R = P_forward*(1-P_diff)/2
        #print(P_forward, P_diff, P_L, P_R)
        
        #set_speed_fraction(P_L, P_R)
        

    except OSError as e:
        print(e)
        stop()
        #sleep(1)

    except KeyboardInterrupt:
        stop()
        break


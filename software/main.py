# ================================================
# IMPORTS
# ================================================


from machine import Pin, I2C, PWM   # tools for talking to pins, sensors, motors
from time import sleep              # sleep(seconds) pauses the program
from vl53l0x import VL53L0X         # the driver that reads the laser sensors
from collections import deque       # deque: a list that automatically discards its oldest entries
from math import atan2, cos, degrees  # NEW: trig used by the angle-aware steering


# ================================================
# SENSORS
# ================================================


# --- The sensor "bus" (I2C) -------------------------------------------------
# The Pico talks to the laser sensors over a shared pair of wires called an
# I2C bus: one wire carries data (SDA) and one carries a timing signal (SCL).
# Here we say: use I2C unit 0, with SDA on GP0 and SCL on GP1.
# `freq` is how fast the chatter runs. 10 000 Hz (10 kHz) is deliberately slow
# but very reliable, which is useful on a breadboard with imperfect connections.
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=10000)

# --- The sensors' on/off switches (XSHUT) -----------------------------------
# Each VL53L0X laser sensor has a "shutdown" pin called XSHUT.
#   XSHUT = 0 (LOW)  ->  sensor is asleep and will not respond on the bus
#   XSHUT = 1 (HIGH) ->  sensor is awake and ready
# We need individual on/off control because all three sensors arrive from the
# factory sharing the SAME I2C address (0x29). If more than one is awake at
# the same time before we rename them, the Pico cannot tell them apart and the
# bus jams. The solution is to wake and rename them ONE AT A TIME.
xshut_RR  = Pin(11, Pin.OUT)        # XSHUT for the RIGHT-REAR  sensor (RR)
xshut_RF = Pin(12, Pin.OUT)         # XSHUT for the RIGHT-FRONT sensor (RF)
xshut_FORWARD = Pin(13, Pin.OUT)    # XSHUT for the FORWARD     sensor


# --- Sensor I2C addresses ---------------------------------------------------
# Every device on an I2C bus must have a unique "address"so the Pico can address each 
# one individually. All VL53L0X sensors leave the factory on address 0x29, so we 
# reassign them during start-up (see init_sensors()).
DEFAULT    = 0x29 
ADDR_RR  = 0x30
ADDR_RF = 0x32 
ADDR_FORWARD = 0x31  

# NOTE: Sensor readings are accessed via `RR_sensor.read()` etc.

def scan():
    """Ask the I2C bus 'who is out there?' and return the list of found addresses.

    Useful for confirming that a sensor has successfully woken up and accepted
    its new address before we try to read from it.
    """
    return i2c.scan()


def change_address(old_addr, new_addr):
    """Tell the sensor on old_addr to switch to new_addr from now on.

    Register 0x8A is the VL53L0X's internal "my I2C address" slot. Writing a
    new value there causes the sensor to respond on the new address immediately.
    The `& 0x7F` masks off the top bit, which the sensor ignores anyway; it is
    a safety measure to avoid accidentally setting a reserved bit.
    A short sleep afterwards lets the sensor settle before we talk to it again.
    """
    i2c.writeto_mem(old_addr, 0x8A, bytes([new_addr & 0x7F]))
    sleep(0.5)


def set_sensor_address(sensor, addr):
    """Update the software object so it knows the sensor's new address.

    Different versions of the VL53L0X MicroPython driver store the address
    under slightly different attribute names (_address, address, or addr).
    We try all three so this works regardless of which driver version is
    installed. If none of the attributes exists, nothing happens — this
    function is just a safety belt.
    """
    if hasattr(sensor, "_address"):
        sensor._address = addr
    if hasattr(sensor, "address"):
        sensor.address = addr
    if hasattr(sensor, "addr"):
        sensor.addr = addr


def init_sensors():
    """Wake all three sensors one at a time and assign each a unique address.

    WHY ONE AT A TIME?
    All three VL53L0X sensors share the same factory address (0x29 = DEFAULT).
    If two or more were awake simultaneously before renaming, any write aimed
    at 0x29 would hit both at once, corrupting the address assignment. The
    solution:
        1. Put ALL sensors to sleep via XSHUT.
        2. Wake ONE, rename it, confirm it answers on its new address.
        3. Repeat for the next sensor (it safely wakes on 0x29 because the
           first sensor has already moved away).
        4. Repeat once more for the third sensor.

    The wake order is: RR (right-rear) -> RF (right-front) -> forward.
    Returns the three ready-to-use sensor objects:
        (RR_sensor, forward_sensor, RF_sensor)
    """

    # 1) Put ALL sensors to sleep so the I2C bus starts completely quiet.
    xshut_RR.value(0)    # RR  sensor -> sleep
    xshut_RF.value(0)   # RF  sensor -> sleep
    xshut_FORWARD.value(0)   # forward sensor -> sleep     # NOTE: This line was not here before.
    sleep(0.1)

    # 2) Wake ONLY the RR (right-rear) sensor and give it ADDR_LEFT (0x30).
    xshut_RR.value(1)    # RR sensor wakes up on DEFAULT (0x29)
    sleep(0.1)
    change_address(DEFAULT, ADDR_RR)   # rename it to 0x30
    sleep(0.1)
    devices = scan()
    if ADDR_RR not in devices:
        # The sensor did not appear at its new address — something went wrong.
        raise SystemExit
    print("RR sensor initialised")  

    # 3) Wake the RF (right-front) sensor. It comes up on DEFAULT (0x29),
    #    which is safe now because RR has already moved to 0x30.
    xshut_RF.value(1)   # RF sensor wakes up on DEFAULT (0x29)
    sleep(0.1)
    devices = scan()
    if DEFAULT not in devices or ADDR_RR not in devices: # We expect to see both 0x29 (RF, just woken) and 0x30 (RR, renamed).
        raise SystemExit
    change_address(DEFAULT, ADDR_RF)   # rename RF to ADDR_RIGHT (0x31)


    sleep(0.1)
    print("RF sensor initialised")

    # 4) Wake the FORWARD sensor. It comes up on DEFAULT (0x29).
    xshut_FORWARD.value(1)   # forward sensor wakes up on DEFAULT (0x29)
    devices = scan()
    if DEFAULT not in devices or ADDR_RR not in devices or ADDR_RF not in devices:
        raise SystemExit
    change_address(DEFAULT, ADDR_FORWARD)   # rename forward to ADDR_FRONT (0x32)
    sleep(0.1)


    devices = scan()                     
    if DEFAULT in devices:
        print("Error: DEFAULT address still present after renaming sensors.")
        raise SystemExit
    elif ADDR_RR not in devices or ADDR_RF not in devices or ADDR_FORWARD not in devices:
        print("Error: One or more sensors did not respond at their new addresses.")
        raise SystemExit

    print(f"RR:{ADDR_RR}, RF:{ADDR_RF}, Forward:{ADDR_FORWARD}") 
    print("All sensors initialised successfully")

    # 5) Create the three Python objects we will use to read distances.

    RR_sensor      = VL53L0X(i2c, ADDR_RR)
    FORWARD_sensor = VL53L0X(i2c, ADDR_FORWARD)
    RF_sensor      = VL53L0X(i2c, ADDR_RF)

    # Make sure each software object also carries the correct address internally
    # (see set_sensor_address() above for why this is needed).
    set_sensor_address(RF_sensor,      ADDR_RF)
    set_sensor_address(RR_sensor,      ADDR_RR)
    set_sensor_address(FORWARD_sensor, ADDR_FORWARD)

    sleep(0.1)

    return RR_sensor, FORWARD_sensor, RF_sensor


def counts_to_mm(counts):   #TODO: Add a translation function/table
    """Convert the VL53L0X's raw "counts" reading into millimetres.

    The VL53L0X returns a raw number of "counts" that is proportional to the
    distance to the nearest object. The datasheet specifies that 1 count = 1 mm, but 
    in reality it is not certain.
    """
    def clamp(value, min_value, max_value):
        """Clamp a value to a specified range."""
        return max(min_value, min(value, max_value))
    return clamp(counts, 30, 500)  


# ================================================
# MOTORS (via the L298N driver chip)
# ================================================


# The L298N requires three control wires per motor: one for speed (PWM) and two for direction.
# Channel A of the L298N drives the LEFT wheel.
ena = PWM(Pin(6))      # SPEED wire for the left wheel  (GP6, PWM output)
in1 = Pin(2, Pin.OUT)  # left wheel DIRECTION wire 1    (GP2, simple on/off)
in2 = Pin(3, Pin.OUT)  # left wheel DIRECTION wire 2    (GP3, simple on/off)

# Channel B of the L298N drives the RIGHT wheel.
enb = PWM(Pin(7))      # SPEED wire for the right wheel (GP7, PWM output)
in3 = Pin(4, Pin.OUT)  # right wheel DIRECTION wire 1   (GP4, simple on/off)
in4 = Pin(5, Pin.OUT)  # right wheel DIRECTION wire 2   (GP5, simple on/off)

# Set the PWM switching frequency to 1000 Hz (1 kHz) for both channels.
# The motor only "feels" the average power, so the exact frequency is not
# critical — 1 kHz is a standard, safe default.
ena.freq(1000)
enb.freq(1000)

# The maximum value duty_u16() accepts (see idea 2: 0 = off, 65535 = full on).
# Giving it a name prevents accidentally exceeding the hardware limit.
PWM_MAX = 65535 #65535


def set_forward():
    """Point both wheels FORWARDS (direction wires only — does not set speed).

    Using the direction rule from idea 3:
        in1 ON  + in2 OFF  ->  left wheel spins forwards
        in3 ON  + in4 OFF  ->  right wheel spins forwards
    Speed remains whatever it was; call set_speed() or set_speed_fraction()
    afterwards (or before) to actually move.
    """
    in1.value(1)
    in2.value(0)
    in3.value(1)
    in4.value(0)


def set_backward():
    """Point both wheels BACKWARDS — the exact opposite of set_forward().

    Each direction pair is flipped relative to set_forward(), so both wheels
    spin in reverse.
    """
    in1.value(0)
    in2.value(1)
    in3.value(0)
    in4.value(1)


def set_left():
    """Spin the robot LEFT on the spot (left wheel back, right wheel forward).

    This sets direction wires only; speed must be set separately.
    Useful for sharp point-turns, e.g. navigating a corner.
    """
    in1.value(0)
    in2.value(1)   # left wheel backwards
    in3.value(1)
    in4.value(0)   # right wheel forwards


def set_right():
    """Spin the robot RIGHT on the spot (left wheel forward, right wheel back).

    This sets direction wires only; speed must be set separately.

    """
    in1.value(1)
    in2.value(0)   # left wheel forwards
    in3.value(0)
    in4.value(1)   # right wheel backwards


def set_speed(left_speed, right_speed):
    """Send a raw PWM duty value (0..65535) directly to each wheel.

    0 = stopped, 65535 = full speed (see idea 2).
    This function sets SPEED ONLY; call set_forward() / set_backward() first
    to choose the direction.
    """
    ena.duty_u16(left_speed)
    enb.duty_u16(right_speed)


def set_speed_fraction(left_speed_frac, right_speed_frac): #TODO it should also prevent from setting too low speed taht would stall the motors
    """Set wheel speeds as fractions of the normal cruising speed (FORWARD_SPEED).

    Using fractions (0.0 to 1.0) is more intuitive than raw 0..65535 numbers:
        0.0 = stopped
        1.0 = normal cruising speed (FORWARD_SPEED)
        Values > 1.0 would exceed FORWARD_SPEED but are clamped to PWM_MAX.
    The fractions are multiplied by FORWARD_SPEED and clamped to the safe range.
    Negative fractions are clamped to 0 (i.e. stopped); reverse is set via
    set_backward(), not via a negative fraction here.
    """
    FORWARD_SPEED = int(PWM_MAX * FORWARD_SPEED_FRACTION)  # Convert the cruising speed fraction to a raw PWM value.
    ls = int(FORWARD_SPEED * left_speed_frac)
    rs = int(FORWARD_SPEED * right_speed_frac)
    ls = max(0, min(ls, PWM_MAX))   # clamp to [0, PWM_MAX]
    rs = max(0, min(rs, PWM_MAX))
    set_speed(ls, rs)


def set_stop():
    """Stop completely: zero speed AND all direction wires off.

    Zeroing only the speed (PWM duty) would leave the direction wires in
    whatever state they were, which can cause the motor driver to brake or
    draw unnecessary current. Clearing all four direction wires ensures the
    driver is fully idle.
    """
    set_speed(0, 0)
    in1.value(0)
    in2.value(0)
    in3.value(0)
    in4.value(0)


def normalise_to_cruise(left_motor_speed_fraction, right_motor_speed_fraction):
    # Make sure that algorithms never a speed higher than the set cruising speed, even if they calculate a higher one.
    if left_motor_speed_fraction > FORWARD_SPEED_FRACTION or right_motor_speed_fraction > FORWARD_SPEED_FRACTION:
        max_fraction = max(left_motor_speed_fraction, right_motor_speed_fraction)
        normalisation_factor = FORWARD_SPEED_FRACTION / max_fraction
        left_motor_speed_fraction *= normalisation_factor
        right_motor_speed_fraction *= normalisation_factor
    return left_motor_speed_fraction, right_motor_speed_fraction

# ================================================
# AUTONOMOUS CONTROL CONSTANTS
# ================================================


# --- Speed Constraints ------------------------------------------------------
FORWARD_SPEED_FRACTION = 0.7      # Maximum speed (cruising speed)
MIN_SPEED_FRACTION = 0.3           # Minimum speed fraction not to stall.
# --- Rolling sensor avarages ------------------------------------------------
AVG_binsize = 1         # Readings bin size: More readings -> smoother but slower
                        # Theoretical max is 30 Hz, but at i2c running at 10 kHz (good for reliability)
                        # it drops to ~8 Hz, with 3 sensors read sequentially, we get ~3Hz               XXX I am not 100% sure about that.
                        # AVG_binsize further divides the effective update rate.
RR_dists      = deque([], AVG_binsize)   # right-rear  sensor history
RF_dists      = deque([], AVG_binsize)   # right-front sensor history
FORWARD_dists = deque([], AVG_binsize)   # forward     sensor history
# --- Front clearance thresholds ---------------------------------------------
FRONT_MIN_DISTANCE = 100        # Below this, stop and reverse.
FRONT_CRUISE_threshold = 300    # Above this, allow full cruising speed.
# --- Reverse behaviour ------------------------------------------------------
REVERSE_SPEED_FRACTION = MIN_SPEED_FRACTION
REVERSE_DURATION = 1            # Duration to reverse when too close to the front wall (in seconds)

# --- Wall-following geometry + control gains --------------------------------
# These three feed the NEW angle-aware steering (see wall_follow_steer()).
SENSOR_SPACING_MM = 125         # <-- MEASURE THIS. Longitudinal gap between the
                                #     RF and RR sensors along the robot body, in
                                #     mm. The angle estimate is only as good as
                                #     this number, so measure it on the real bot.
RIGHT_TARGET_DISTANCE = 100     # The wall distance we want to HOLD (mm).
DIST_GAIN  = 0.01              # Steering produced per mm of distance error.  (tune)
ANGLE_GAIN = 0.6                # Steering produced per radian of heading error. (tune)
STEER_LIMIT = 0.6               # Max steering authority, 0..1 (keeps a wheel from
                                # being told to reverse or to over-spin).


# ================================================
# WALL-FOLLOWING GEOMETRY + STEERING  (NEW)
# ================================================
#
# We follow a wall on the robot's RIGHT using the two right-side sensors:
#   RF = right-FRONT  (call it d_front)
#   RR = right-REAR   (call it d_rear)
#
# ASSUMPTION: both right sensors point straight out, perpendicular to the body,
# and sit SENSOR_SPACING_MM apart along the body. If yours are splayed at an
# angle instead, the tan() formula below needs adjusting for that offset.
#
#        front of robot
#            ^
#            |                 wall  ─────────────────────
#         [RF] ─ ─ ─ ─ ─ ─ ─ ─ ─►   d_front
#            |
#            |  SENSOR_SPACING_MM
#            |
#         [RR] ─ ─ ─ ─ ─ ─ ─ ─ ─►   d_rear
#            |
#
# If the robot runs perfectly parallel to the wall, d_front == d_rear. If the
# nose drifts away from the wall, d_front > d_rear (and vice-versa). That
# difference, divided by the known spacing, is the tangent of the heading
# angle to the wall — the piece of information the old min()-based code could
# not see, and the reason it could only react to distance, never to attitude.


def wall_geometry(d_front, d_rear, spacing):
    """Turn the two right-side readings into (perpendicular distance, angle).

    Returns:
        perp_distance : the true straight-line distance from the robot to the
                        wall (mm), corrected for the robot's heading.
        angle         : robot heading relative to the wall, in RADIANS.
                          angle > 0  ->  nose pointing AWAY from the wall
                          angle < 0  ->  nose pointing TOWARD the wall
                          angle = 0  ->  travelling parallel to the wall

    Maths (two parallel beams, 'spacing' mm apart along the body):
            tan(angle)    = (d_front - d_rear) / spacing
            perp_distance = cos(angle) * (d_front + d_rear) / 2
    The cos() factor removes the slant: a slanted robot reads a longer beam
    than its true perpendicular distance, and cos(angle) corrects for it.
    """
    angle = atan2(d_front - d_rear, spacing)
    perp_distance = cos(angle) * (d_front + d_rear) / 2.0
    return perp_distance, angle


def wall_follow_steer(d_front, d_rear, base_speed):
    """Steer to hold a FIXED wall distance while staying parallel to the wall.

    This is a small PD-style controller acting on two errors at once:
      * distance_error : how far we are from the target distance  -> the 'P' term
      * angle          : our heading relative to the wall         -> the 'D' / damping term

    Combined steering signal (POSITIVE => steer TOWARD the wall, i.e. to the right):

        steer = DIST_GAIN * distance_error  +  ANGLE_GAIN * angle

    Why the angle term is the whole point:
      Suppose we are too far out, so the distance term tells us to cut back
      toward the wall. As the robot turns in, its nose now points at the wall,
      so 'angle' goes negative and SUBTRACTS from steer. That straightens the
      robot out *before* it reaches the target distance, so it settles running
      parallel instead of weaving in-and-out forever. The old three-branch
      if/elif/else had no notion of heading, so it could only ever overshoot
      and oscillate.

    Returns (S_L, S_R, perp_distance, angle); the last two are handy for debug.
    """
    perp_distance, angle = wall_geometry(d_front, d_rear, SENSOR_SPACING_MM)

    # Positive when we are TOO FAR from the wall and need to move closer.
    distance_error = perp_distance - RIGHT_TARGET_DISTANCE

    # One combined correction, then clamp so a wheel is never told to reverse.
    steer = DIST_GAIN * distance_error + ANGLE_GAIN * angle
    steer = max(-STEER_LIMIT, min(steer, STEER_LIMIT))

    # Steering toward the wall (right) = LEFT wheel faster, RIGHT wheel slower.
    # This matches the wheel convention used by set_right() elsewhere in the file.
    S_L = base_speed * (1 + steer)
    S_R = base_speed * (1 - steer)
    return S_L, S_R, perp_distance, angle


# ================================================
# CONTROL LOGIC
# ================================================


set_stop()         # ensure nothing is moving before initialisation begins
set_speed(0,0)     # ensure motors are stopped before initialisation begins

RR_sensor, FORWARD_sensor, RF_sensor = init_sensors()   # Initialise the sensors and get the ready-to-use sensor objects.


set_forward()      # start moving forward immediately; we will adjust speed and direction as we go
set_speed_fraction(MIN_SPEED_FRACTION, MIN_SPEED_FRACTION)
sleep(1)
set_backward()     # start moving backward immediately; we will adjust speed and direction as we go
set_speed_fraction(MIN_SPEED_FRACTION, MIN_SPEED_FRACTION)
sleep(1)
set_stop()         # stop before entering the main loop


while True:   # repeat forever (Ctrl-C to exit cleanly)
    try:


        # --- 1. Read all three sensors and append to their rolling histories ----------------

        RR_dists.append(counts_to_mm(RR_sensor.read()))
        RR_distance = sum(RR_dists) / AVG_binsize  

        RF_dists.append(counts_to_mm(RF_sensor.read()))
        RF_distance = sum(RF_dists) / AVG_binsize

        FORWARD_dists.append(counts_to_mm(FORWARD_sensor.read()))
        FORWARD_distance = sum(FORWARD_dists) / AVG_binsize

        print(f"Rolling distances (RR, RF, FORWARD): {RR_distance}, {RF_distance}, {FORWARD_distance}")

        # --- 2. Decide what to do based on the FORWARD clearance ----------------
        if FORWARD_distance < FRONT_MIN_DISTANCE:
            # Wall dead ahead: stop, back off briefly, then re-assess next loop.
            set_stop()
            set_backward()
            set_speed_fraction(REVERSE_SPEED_FRACTION*1.1, REVERSE_SPEED_FRACTION) # 
            sleep(REVERSE_DURATION)  # Reverse for a short duration before reassessing
            set_stop()  # Stop after reversing
        else:
            set_forward()

            # --- 2a. Base forward speed from the FORWARD clearance ----------------------
            if FORWARD_distance >= FRONT_CRUISE_threshold:
                # Open road ahead: cruise at full speed.
                base_speed = FORWARD_SPEED_FRACTION
            else:
                # Between FRONT_MIN_DISTANCE and FRONT_CRUISE_threshold:
                # ramp speed linearly from MIN_SPEED_FRACTION up to FORWARD_SPEED_FRACTION.
                span = FRONT_CRUISE_threshold - FRONT_MIN_DISTANCE
                progress = (FORWARD_distance - FRONT_MIN_DISTANCE) / span   # 0.0 .. 1.0
                base_speed = MIN_SPEED_FRACTION + progress * (FORWARD_SPEED_FRACTION - MIN_SPEED_FRACTION)

            # --- 2b. Angle-aware wall following (replaces the old if/elif/else) ---------
            # Feed the RIGHT-FRONT reading as d_front and RIGHT-REAR as d_rear.
            # The controller estimates both our distance AND our heading to the
            # wall, then returns left/right wheel speeds that hold the target
            # distance while keeping us parallel.
            S_L, S_R, wall_dist, wall_angle = wall_follow_steer(
                RF_distance, RR_distance, base_speed)

            S_Ln, S_Rn = normalise_to_cruise(S_L, S_R)
            print(f"wall_dist={wall_dist:.0f}mm  angle={degrees(wall_angle):.1f}deg  "
                  f"S_L={S_L:.2f} S_R={S_R:.2f} -> S_Ln={S_Ln:.2f} S_Rn={S_Rn:.2f}\n")
            set_speed_fraction(S_Ln, S_Rn)
            #sleep(1)



    except OSError as e:
        # A sensor read failed — typically a transient I2C hiccup on a
        # breadboard. Print the error, stop the motors, and immediately retry
        print(e)
        sleep(0.05)  # brief pause to avoid flooding the console with errors

    except KeyboardInterrupt:
        # Ctrl-C pressed: stop the wheels and exit the loop cleanly.
        set_stop()
        break


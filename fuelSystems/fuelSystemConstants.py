from enum import Enum
from utils.units import in2m

# These are just temporary/example names. We should do better when we decide on actual positions.
class shooterTargetCmd(Enum):
    MANUALMODE = 0 # Direct control with operator joysticks
    CORNERONE = 1 
    CORNERTWO = 2

# How far the shooter is from the center of the robot in meters
SHOOTER_OFFSET = in2m(-3) # Random number right now
PITCH_MOTOR_BELT_RATIO = 62
# HOOD_ANGLE_OFFSET = 81

# This is actually the reciprical of the ratio, so in the motor call it will be 1/2 or 1/1
HOOD_MOTOR_BELT_RATIO = 1
MAIN_MOTOR_BELT_RATIO = 2

SHOOTER_HOOD_WHEEL_RADIUS = in2m(1)
SHOOTER_MAIN_WHEEL_RADIUS = in2m(2)

GRAVITY = -9.8 # m/s

SHOOTER_ACTIVATOR_TARGET_PERCENT = 0.05

ROBOT_CYCLE_TIME = 0.04

# TURRET_MAX_YAW = math.pi
# TURRET_MIN_YAW = -math.pi

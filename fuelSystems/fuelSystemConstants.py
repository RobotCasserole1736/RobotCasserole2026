from enum import Enum
import math
from utils.units import deg2Rad 

#Constants file

class shooterTargetCmd(Enum): #These are just temporary/example names. We should do better when we decide on actual positions.
    MANUALMODE = 0 #Basically, tell turret to be under the direct control of the operator joysticks.
    CORNERONE = 1 
    CORNERTWO = 2

SHOOTER_OFFSET = -0.0002 #How far the shooter is from the center of the robot in meters.
HOOD_ANGLE_OFFSET = deg2Rad(85)

#My current unerstanding is that the turret will be in the back center of the robot.

SHOOTER_HOOD_WHEEL_RADIUS = 0.0254 # meters converted from 1 inch  

GRAVITY = -9.8 #Meters/second

TURRET_MAX_YAW = math.pi #For now I'm assuming that facing in the same direction as the robot is zero? 
TURRET_MIN_YAW = -math.pi #

SHOOTER_ACTIVATOR_TARGET_PERCENT = 0.05
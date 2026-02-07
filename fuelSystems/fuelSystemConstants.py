from enum import Enum
import math
from wpimath.geometry import Translation2d
from utils.constants import blueHubLocation

# Constants file

class shooterTargetCmd(Enum): # These are just temporary/example names. We should do better when we decide on actual positions.
    MANUALMODE = 0 # Turret is directly controlled by operator joysticks
    CORNERONE = 1
    CORNERTWO = 2

class shooterTarget():

    def __init__(self, name: str, targetPos: Translation2d, targetMaxHeight: float,
                 heightOffset: float, spinOffset=0, hubSpin=False, maximizeRoll=False) -> None:
        """
        Docstring for __init__

        :param self: instantiate this for every target position.
        :param name: name of the target.
        :param targetPos: based on blue alliance position for the target.
        :type targetPos: translation2d
        :param targetMaxHeight: meters- the maximum height you want the ball to reach
        :type targetHeight: float
        :param heightOffset: meters- how far from the target you want the ball to reach its maximum height
        :type heightOffset: float
        :param spinOffset: how much spin you want -- experimental right now, haven't implemented spin yet
        :param hubSpin: if you need to use the spin function
        :param maximizeRoll: if you want the ball to roll as far as possible when it reaches the ground. -- for stuff like feeding human player.
        """
        self.name = name
        self.targetPos = targetPos
        self.targetHeight = targetMaxHeight
        self.heightOffset = heightOffset
        self.spinOffset = spinOffset
        self.hubSpin = hubSpin
        self.maximizeRoll = maximizeRoll

# Define each target as constants here
hubTarget = shooterTarget("hub", blueHubLocation, 2.25, .304)

# Throw them in a list here?

SHOOTER_OFFSET = 0.5 # How far the shooter is from the center of the robot in meters.
# My current unerstanding is that the turret will be in the back center of the robot.

SHOOTER_WHEEL_RADIUS = 0.0254 # meters converted from 2 inches

GRAVITY = -9.8 # Meters/second

# For now I'm assuming that facing in the same direction as the robot is zero?
TURRET_MAX_YAW = math.pi
TURRET_MIN_YAW = -math.pi

SHOOTER_ACTIVATOR_TARGET_PERCENT = 0.05

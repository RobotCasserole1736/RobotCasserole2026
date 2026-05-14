from enum import Enum
from utils.allianceTransformUtils import transform
from utils.constants import blueHubLocation
from utils.units import deg2Rad
from wpimath.geometry import Translation2d
from wpimath.units import inchesToMeters

# Shooting Distance Commands
class shooterDistance(Enum):
    NONE = 0
    SHORT = 1
    LONG = 2
    SHORTERFORAUTO = 3

# Target Commands
class shooterTarget(Enum):
    NONE = Translation2d()
    HUB = transform(blueHubLocation)
    OUTPOSTCORNER = transform(Translation2d(0,0))
    DEPCORNER = transform(Translation2d(0,inchesToMeters(316.64)))

LONG_SHOT_DIST_M = 3.89214408

# For shooter offset, measure from center of robot directly
# to center of turret in inches
SHOOTER_OFFSET = inchesToMeters(2)

# Turret Constants
TURRET_ENABLE = False
TURRET_MAX_YAW_RAD = deg2Rad(135)
TURRET_MIN_YAW_RAD = deg2Rad(-135)
YAW_MOTOR_RATIO = 1

# Intake Wrist States
class intakeWristState(Enum):
    NONE = 0
    STOW = 1
    GROUND = 2

# Intake Wrist Encoder ffset
INTAKE_WRIST_ABS_ENC_OFFSET_RAD = deg2Rad(230)

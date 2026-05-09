from enum import Enum
from utils.units import deg2Rad

# States to switch intended distance of shot
class shooterDistance(Enum):
    NONE = 0
    SHORT = 1
    LONG = 2
    SHORTERFORAUTO = 3

# States for wrist positions
class intakeWristState(Enum):
    NONE = 0
    STOW = 1
    GROUND = 2

# Intake wrist encoder offset
INTAKE_WRIST_ABS_ENC_OFFSET_RAD = deg2Rad(230)

LONG_SHOT_DIST_M = 3.89214408

# Turret Constants
TURRET_ENABLE = False
TURRET_MAX_YAW_RAD = deg2Rad(135)
TURRET_MIN_YAW_RAD = deg2Rad(-135)
YAW_MOTOR_RATIO = 1

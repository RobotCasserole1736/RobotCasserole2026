from enum import Enum

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

# Offset for intake wrist absolute encoder
INTAKE_ANGLE_ABS_POS_ENC_OFFSET_DEG = 230

# 
LONG_SHOT_DIST_M = 3.89214408

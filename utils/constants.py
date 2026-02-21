# Constants we may need
# Just starting with the minimum stuff we need
# The math conversions are under units.py

from wpimath.geometry import Translation2d
from enum import Enum

#######################################################################################
## FIELD DIMENSIONS
#######################################################################################

FIELD_X_M = 17.548 # "Length"
FIELD_Y_M = 8.062  # "Width"

# Blue Reef Location
_REEF_LOC_X_M = 4.502
_REEF_LOC_Y_M = 4.0363
blueReefLocation = Translation2d(_REEF_LOC_X_M, _REEF_LOC_Y_M)
redReefLocation = Translation2d(FIELD_X_M - _REEF_LOC_X_M, _REEF_LOC_Y_M)

#######################################################################################
## CAN ID'S
#######################################################################################
# Reserved_CANID = 0
FUEL_WRIST_CANID = 21
DT_FR_WHEEL_CANID = 2
DT_FR_AZMTH_CANID = 3
DT_FL_WHEEL_CANID = 4
DT_FL_AZMTH_CANID = 5
DT_BR_WHEEL_CANID = 6
DT_BR_AZMTH_CANID = 7
DT_BL_WHEEL_CANID = 8
DT_BL_AZMTH_CANID = 9
LONG_HOOK_CANID = 10
CLIMBER_PIVOT_CANID = 11
SHORT_HOOK_CANID = 12
UNUSED_R_CANID = 13
INTAKE_CONTROL_CANID = 14
INTAKE_WHEELS_CANID = 15
FUEL_WRIST_CANID = 16
CLIMB_CANID = 19
HOPPER_CANID = 20

#######################################################################################
## PWM Bank
#######################################################################################

# Unused = 0
# Unused = 1
# Unused = 2
# Unused = 3
# Unused = 4
# Unused = 5
# Unused = 6
# Unused = 8
# Unused = 7
LED_STACK_LIGHT_CTRL_PWM = 9


#######################################################################################
## DIO Bank
#######################################################################################

DT_FR_AZMTH_ENC_PORT = 0
DT_BR_AZMTH_ENC_PORT = 1
DT_BL_AZMTH_ENC_PORT = 2
DT_FL_AZMTH_ENC_PORT = 3
FUEL_GAME_PIECE_PORT = 4
ELEV_TOF_CANID = 6
HEARTBEAT_LED_PIN = 7
FIX_ME_LED_PIN = 8
INTAKE_ENC_PORT = 9
########################################################################################################
## Enums
########################################################################################################
class ClimberSteps(Enum):
    STEP0_IDLE = 0
    STEP1_LONGHOOK_DOWN_SHORTHOOK_UP = 1
    STEP2_SHORTHOOK_LATCHES_ONTO_BAR = 2
    STEP3_LONGHOOK_DISENGAGES_FROM_BAR = 3
    STEP4_LONGHOOK_BACK_DOWN = 4
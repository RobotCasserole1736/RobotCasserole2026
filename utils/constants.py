# Constants we may need
# Just starting with the minimum stuff we need
# The math conversions are under units.py

from wpimath.geometry import Translation2d
from enum import Enum

#######################################################################################
## FIELD DIMENSIONS
#######################################################################################
FIELD_X_M = 16.541 # "Length"
FIELD_Y_M = 8.069 # "Width"

# Blue Hub Location
_HUB_LOC_X_M = 4.626
_HUB_LOC_Y_M = 4.035
blueHubLocation = Translation2d(_HUB_LOC_X_M, _HUB_LOC_Y_M)
redHubLocation = Translation2d(FIELD_X_M - _HUB_LOC_X_M, _HUB_LOC_Y_M)

# Blue Tower Location (to center of ladder)
_TOW_LOC_X_M = 1.056
_TOW_LOC_Y_M = 3.745
_TOW_LOC_Y_RED_M = 4.324
blueTowerLocation = Translation2d(_TOW_LOC_X_M, _TOW_LOC_Y_M)
redTowerLocation = Translation2d(FIELD_X_M - _TOW_LOC_X_M, _TOW_LOC_Y_RED_M)

#######################################################################################
## CAN ID'S
#######################################################################################
# Reserved_CANID = 0
DT_FR_WHEEL_CANID = 2
DT_FR_AZMTH_CANID = 3
DT_FL_WHEEL_CANID = 4
DT_FL_AZMTH_CANID = 5
DT_BR_WHEEL_CANID = 6
DT_BR_AZMTH_CANID = 7
DT_BL_WHEEL_CANID = 8
DT_BL_AZMTH_CANID = 9
TURRET_FEED_CANID = 10
TURRET_PITCH_CANID = 11
HOOD_SHOOTER_CANID = 12
MAIN_SHOOTER_CANID = 13
TURRET_YAW_CANID = 14
INTAKE_WHEELS_CANID = 15
INDEXER_CANID = 16
INTAKE_CONTROL_CANID = 17
POWER_FLOOR_CANID = 18
CLIMB_CANID = 19
HOPPER_CANID = 20
LEFT_HOOK_CANID = 21
RIGHT_HOOK_CANID = 22

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
# Unused = 7
# Unused = 8
LED_STACK_LIGHT_CTRL_PWM = 9

#######################################################################################
## DIO Bank
#######################################################################################
DT_BR_AZMTH_ENC_PORT = 0
DT_FL_AZMTH_ENC_PORT = 1
DT_BL_AZMTH_ENC_PORT = 2
DT_FR_AZMTH_ENC_PORT = 3
PITCH_ENC_PORT = 4
FUEL_GAME_PIECE_PORT = 5
ELEV_TOF_CANID = 6
HEARTBEAT_LED_PIN = 7
FIX_ME_LED_PIN = 8
INTAKE_ENC_PORT = 9

########################################################################################################
## Enums
########################################################################################################
class ClimberSteps(Enum):
    STEP0_IDLE = 0
    STEP1_LEFTHOOK_DOWN_RIGHTHOOK_UP = 1
    STEP2_RIGHTHOOK_LATCHES_ONTO_BAR = 2
    STEP3_LEFTHOOK_DISENGAGES_FROM_BAR = 3
    STEP4_LEFTHOOK_BACK_DOWN = 4
    STEP5_CLIMB_DOWN = 5
    STEP6_AUTO_CLIMB = 6

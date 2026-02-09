# Constants we may need
# Just starting with the minimum stuff we need
# The math conversions are under units.py

from wpimath.geometry import Translation2d

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
#CORAL_IN_CANID = 21
DT_FR_WHEEL_CANID = 2
DT_FR_AZMTH_CANID = 3
DT_FL_WHEEL_CANID = 4
DT_FL_AZMTH_CANID = 5
DT_BR_WHEEL_CANID = 6
DT_BR_AZMTH_CANID = 7
DT_BL_WHEEL_CANID = 8
DT_BL_AZMTH_CANID = 9
TURRET_PITCH_CANID = 11
TURRET_FEED_CANID = 10
MAIN_SHOOTER_CANID = 12 #Top and bottom shooter motors may be swapped, check once we have code to apply spin -- MAIN/LARGER
HOOD_SHOOTER_CANID = 13  # -- HOOD
UNASSIGNED14 = 14
#TURRET_YAW_CANID = 1
#UNASSIGNED15 = 15
UNASSIGNED16 = 16
UNASSIGNED19 = 19
UNASSIGNED18 = 20

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

DT_BR_AZMTH_ENC_PORT = 0
DT_FL_AZMTH_ENC_PORT = 1
DT_BL_AZMTH_ENC_PORT = 2
DT_FR_AZMTH_ENC_PORT = 3

CORAL_GAME_PIECE_B_PORT = 4
CORAL_GAME_PIECE_F_PORT = 5
ELEV_TOF_CANID = 6
HEARTBEAT_LED_PIN = 7
FIX_ME_LED_PIN = 8
ALGAE_ENC_PORT = 9

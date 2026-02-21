from enum import Enum

#Constants file

class shooterTargetCmd(Enum): #These are just temporary/example names. We should do better when we decide on actual positions.
    MANUALMODE = 0 #Basically, tell turret to be under the direct control of the operator joysticks.
    CORNERONE = 1 
    CORNERTWO = 2

SHOOTEROFFSET = 0

class IntakeWristState(Enum):
    NOTHING = 0
    GROUND = 1
    STOW = 2

ALGAE_ANGLE_ABS_POS_ENC_OFFSET = 45.0 #degrees, adjust as needed to make 0 degrees horizontal

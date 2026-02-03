from wpimath.geometry import Pose2d, Rotation2d, Transform2d
from utils.constants import blueTowerLocation, redTowerLocation
from wpimath.units import inchesToMeters
from drivetrain.drivetrainPhysical import WHEEL_BASE_HALF_LENGTH_M, BUMPER_THICKNESS_M
from utils.allianceTransformUtils import onRed

"""
Constants related to navigation
Measurements from 
https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
"""

# Our convention for scoring position
#       Y
#       ^
#       |
# Blue1 |
#       |
#    ---|
#       |
# Blue2 |                _____
#       |               /     \
#    ---|-----|        /  Hub  \
#       |Tower|        \       /
#    ---|-----|         \_____/
#       |
# Blue3 |
#       |
# <-----+------------------------------> X
#       |
#       v

#####################################################################################
# Fudge Factors
# Theoretically, these are the only thing you tweak in this file
# Nominally, these are all zero. Make them not-zero to tweak for specific score positions
# and account for field assembly questions.

# X axis fudge factor for BLUE Hub
FUDGE_DIST_X_BLUE_HUB = inchesToMeters(0.0)

# Y axis fudge factor for BLUE Hub
FUDGE_DIST_Y_BLUE_HUB = inchesToMeters(0.0)

# X axis fudge factor for RED Hub
FUDGE_DIST_X_RED_HUB = inchesToMeters(0.0)

# Y axis fudge factor for RED Hub
FUDGE_DIST_Y_RED_HUB = inchesToMeters(0.0)

# X axis fudge factor for BLUE TOWER
FUDGE_DIST_X_BLUE_TOW = inchesToMeters(0.0)

# Y axis fudge factor for BLUE TOWER
FUDGE_DIST_Y_BLUE_TOW = inchesToMeters(0.0)

# X axis fudge factor for RED TOWER
FUDGE_DIST_X_RED_TOW = inchesToMeters(0.0)

# Y axis fudge factor for RED TOWER
FUDGE_DIST_Y_RED_TOW = inchesToMeters(0.0)

#####################################################################################

# Rotations that we should be at when climbing
# Must be in order ??? Probably some kind of order
# Note: not sure if multiple rotations are needed depending where we are climbing
# on the tower
TOWER_ROTS = [
Rotation2d.fromDegrees(180.0), # Left side of tower?
Rotation2d.fromDegrees(180.0), # Right side of tower?
]

# Radius from center of hub to center of the face
# This might not be needed; also, is it inaccurate
HUB_RADIUS = inchesToMeters(47.00)

# Inches from center of tower to end of overhang
TOWER_HALF_LENGTH = inchesToMeters(23.00)

# Distance needed to position climbing mechanism
CLIMBER_DIST_IN_ROBOT = inchesToMeters(12) # Needs to be measured when robot is built

# Distance we want to be from the tower while scoring
# Sum of tower over hang and robot size.
# This might need to change since I don't know how exactly the climb
# works at time of writing
CLIMB_DIST_FROM_TOWER_CENTER = \
    TOWER_HALF_LENGTH + \
    WHEEL_BASE_HALF_LENGTH_M + \
    BUMPER_THICKNESS_M

# Pre-calculate blue tower positions
_goalListCacheBlue = []
for idx, rot in enumerate(TOWER_ROTS):
    # start at the reef location, pointed in the right direction.
    tmp = Pose2d(blueTowerLocation, rot)
    # Transform to the score locations

    # Nominal Y distance from center of tower
    yOffset = -1.0 * CLIMB_DIST_FROM_TOWER_CENTER

    # Fudged
    yOffset += FUDGE_DIST_Y_BLUE_TOW

    # Nominal X distance
    xOffset = CLIMBER_DIST_IN_ROBOT

    # Fudge the climber mechanism distance
    xOffset += FUDGE_DIST_X_BLUE_TOW

    # Does this need to just be a translation?
    tmp = tmp.transformBy(Transform2d(xOffset, yOffset, Rotation2d()))
    _goalListCacheBlue.append(tmp)


# Pre-calculate red tower positions
_goalListCacheRed = []
for idx, rot in enumerate(TOWER_ROTS):
    # start at the reef location, pointed in the right direction.
    rot = Rotation2d.fromDegrees(180) + rot # Invert for other side of the field
    tmp = Pose2d(redTowerLocation, rot)
    # Transform to the score locations

    # Nominal distance from center
    yOffset = -1.0 * CLIMB_DIST_FROM_TOWER_CENTER

    # Fudged
    yOffset += FUDGE_DIST_Y_RED_TOW

    # Nominal X distance
    xOffset = CLIMBER_DIST_IN_ROBOT

    # Fudge the left/right distance
    xOffset += FUDGE_DIST_X_RED_TOW

    tmp = tmp.transformBy(Transform2d(xOffset, yOffset, Rotation2d()))
    _goalListCacheRed.append(tmp)

# NOTE - This function returns goals ALREADY transformed to the correct side.
# You do NOT need to call transform again on the result of poses
def getTransformedGoalList() -> list[Pose2d]:
    if(onRed()):
        return _goalListCacheRed
    else:
        return _goalListCacheBlue

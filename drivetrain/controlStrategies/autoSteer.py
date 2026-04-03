# from drivetrain.drivetrainControl import DrivetrainControl
from drivetrain.poseEstimation.drivetrainPoseEstimator import DrivetrainPoseEstimator
from wpilib import Timer
# from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from drivetrain.drivetrainCommand import DrivetrainCommand
# from drivetrain.drivetrainPhysical import MAX_ROTATE_ACCEL_RAD_PER_SEC_2
from utils.allianceTransformUtils import transformX
from utils.calibration import Calibration
from utils.constants import _HUB_LOC_X_M, _HUB_LOC_Y_M
from utils.signalLogging import addLog
from utils.singleton import Singleton

class AutoSteer(metaclass=Singleton):
    def __init__(self):
        self.hubAlignActive = False
        self.returnDriveTrainCommand = DrivetrainCommand()
        self.rotKp = Calibration(name="Auto Align Rotation Kp",default=2.0) # 2 and 2
        self.maxRotSpd = Calibration(name="Auto Align Max Rotate Speed",default=2.0)

        # Previous Rotation Speed and time for calculating derivative
        self.prevDesAngle = 0
        self.prevTimeStamp = Timer.getFPGATimestamp()

        # Set speaker coordinates
        self.hubX = transformX(_HUB_LOC_X_M)
        self.hubY = _HUB_LOC_Y_M

        self.desiredAngle = 0

    def setHubAutoAlignCmd(self, shouldAutoAlign: bool):
        self.hubAlignActive = shouldAutoAlign

    def update(self, cmdIn: DrivetrainCommand, curPose: Pose2d) -> DrivetrainCommand:
        if self.hubAlignActive:
           # self.getDesiredSingerAngle(curPose)
            return self.calcHubDrivetrainCommand(curPose, cmdIn)
        else:
            return cmdIn

    def getRotationAngle(self, curPose: Pose2d) -> Rotation2d:
        targetLocation = Translation2d(transformX(self.hubX),self.hubY)
        robotToTargetTrans = targetLocation - curPose.translation()
        return  Rotation2d(-1 * robotToTargetTrans.X(), -1 * robotToTargetTrans.Y())#Shooter is on back of robot.

    def calcHubDrivetrainCommand(self, curPose: Pose2d, cmdIn: DrivetrainCommand) -> DrivetrainCommand:
        # Find difference between robot angle and angle facing the speaker
        rotError = self.getRotationAngle(curPose) - curPose.rotation()

        # Check to see if we are making a really small correction
        # If we are, don't worry about it. We only need a certain level of accuracy
        if abs(rotError.radians()) <= 0.05:
            rotError = 0
        else:
            rotError = rotError.radians()
        
        if abs(rotError*self.rotKp.get()) < self.maxRotSpd.get():
            self.returnDriveTrainCommand.velT = rotError*self.rotKp.get() 
        elif rotError >= 0:
            self.returnDriveTrainCommand.velT = self.maxRotSpd.get() 
        else: #when the rotation error is negative
            self.returnDriveTrainCommand.velT = -1 * self.maxRotSpd.get()
        self.returnDriveTrainCommand.velX = cmdIn.velX # Set the X vel to the original X vel
        self.returnDriveTrainCommand.velY = cmdIn.velY # Set the Y vel to the original Y vel
        return self.returnDriveTrainCommand

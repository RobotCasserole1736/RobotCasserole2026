from Autonomous.commands.drivePathCommand import DrivePathCommand
from AutoSequencerV2.mode import Mode
from utils.allianceTransformUtils import transform
from utils.autonomousTransformUtils import flip
from Autonomous.commands.shooterCommand import ShootFuelCommand
from Autonomous.commands.driveForwardSlowCommand import DriveForwardSlowCommand
from Autonomous.commands.driveBackwardSlowCommand import DriveBackwardSlowCommand
from AutoSequencerV2.parallelCommandGroup import ParallelCommandGroup
from AutoSequencerV2.sequentialCommandGroup import SequentialCommandGroup
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from utils.units import deg2Rad
from wpilib import DriverStation

# Just shoots. That's all.
class JustShoot(Mode):
    def __init__(self):
        #This is naming the mode, in this case "Just Shoot"
        Mode.__init__(self, f"Just Shoot")

        if (DriverStation.getAlliance() == DriverStation.Alliance.kRed) == True:
            self.moveBackwardCmd = DriveBackwardSlowCommand(duration=1.75,speed= 0.8)
        else:
            self.moveBackwardCmd = DriveBackwardSlowCommand(duration=1.75,speed= -0.8)
        self.scoreCmd = ShootFuelCommand()
        # self.cmdGroup = SequentialCommandGroup([self.moveBackwardCmd, self.scoreCmd])
        self.cmdGroup = ParallelCommandGroup([self.moveBackwardCmd, self.scoreCmd])

    def getCmdGroup(self):
        # Just return the path command normally, since we're only doing one path. 
        # When changing to the return self.pathCmd, get rid of the pass
        return self.cmdGroup

    def getInitialDrivetrainPose(self):
        # Use the path command to specify the starting pose, using getInitialPose()
        return flip(transform(Pose2d(Translation2d(3.486, 4.0445), Rotation2d(deg2Rad(7.2525)))))

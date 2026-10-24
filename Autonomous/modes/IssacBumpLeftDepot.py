
from AutoSequencerV2.mode import Mode
from AutoSequencerV2.parallelCommandGroup import ParallelCommandGroup
from AutoSequencerV2.sequentialCommandGroup import SequentialCommandGroup
from Autonomous.commands.shooterCommand import ShootFuelCommand
from Autonomous.commands.drivePathCommand import DrivePathCommand
from Autonomous.commands.IntakingCommand import IntakeBallCommand
from utils.allianceTransformUtils import transform
from utils.autonomousTransformUtils import flip

class IssacBumpLeftDepot(Mode):
    def __init__(self):
        Mode.__init__(self, f"Left Bump")
        
        self.pathCmd1 = DrivePathCommand("IssacGoofyBump")
        self.pathCmd2 = DrivePathCommand("IssacGoofyMiddle_SHHOOOTTT")
        self.intake = IntakeBallCommand()
        self.shoot = ShootFuelCommand()
        self.group1 = ParallelCommandGroup([self.intake,self.pathCmd1])
        self.wholeCmd = SequentialCommandGroup([self.group1,self.pathCmd2,self.shoot])

    def getCmdGroup(self):
        # Just return the path command normally, since we're only doing one path. 
        # When changing to the return self.pathCmd, get rid of the pass
        return self.wholeCmd

    def getInitialDrivetrainPose(self):
        # Use the path command to specify the starting pose, using getInitialPose()
        return flip(transform(self.pathCmd1.path.get_initial_pose()))

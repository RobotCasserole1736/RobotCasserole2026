
from AutoSequencerV2.mode import Mode
from AutoSequencerV2.parallelCommandGroup import ParallelCommandGroup
from AutoSequencerV2.sequentialCommandGroup import SequentialCommandGroup
from Autonomous.commands.shooterCommand import ShootFuelCommand
from Autonomous.commands.drivePathCommand import DrivePathCommand
from Autonomous.commands.IntakingCommand import IntakeBallCommand
from utils.allianceTransformUtils import transform
from utils.autonomousTransformUtils import flip

class CCycleL1(Mode):
    def __init__(self):
        Mode.__init__(self, f"C Cycle L1")
        
        self.pathCmd1 = DrivePathCommand("fullcycleL1")
        self.pathCmd2 = DrivePathCommand("fullcycleL2")
        self.pathCmd3 = DrivePathCommand("fullcycleL3")
        self.pathCmd4 = DrivePathCommand("fullcycleL4")
        self.score = ShootFuelCommand()
        self.intake = IntakeBallCommand()
        self.group = SequentialCommandGroup([self.score,self.pathCmd1,self.group2,self.pathCmd3,self.group2a])
        self.group2 = ParallelCommandGroup([self.intake,self.pathCmd2])
        self.group2a = ParallelCommandGroup([self.score,self.pathCmd4])

    def getCmdGroup(self):
        # Just return the path command normally, since we're only doing one path. 
        # When changing to the return self.pathCmd, get rid of the pass
        return self.group

    def getInitialDrivetrainPose(self):
        # Use the path command to specify the starting pose, using getInitialPose()
        return flip(transform(self.pathCmd1.path.get_initial_pose()))

from Autonomous.commands.drivePathCommand import DrivePathCommand
from AutoSequencerV2.mode import Mode
from utils.allianceTransformUtils import transform
from utils.autonomousTransformUtils import flip
from Autonomous.commands.shooterCommand import ShootFuelCommand

# Just shoots. That's all.
class JustShoot(Mode):
    def __init__(self):
        #This is naming the mode, in this case "Just Shoot"
        Mode.__init__(self, f"Just Shoot")

        self.score = ShootFuelCommand()

    def getCmdGroup(self):
        # Just return the path command normally, since we're only doing one path. 
        # When changing to the return self.pathCmd, get rid of the pass
        return self.score

    def getInitialDrivetrainPose(self):
        # Use the path command to specify the starting pose, using getInitialPose()
        return None

import os
import choreo
import choreo.trajectory
from wpilib import Timer
from drivetrain.controlStrategies.trajectory import Trajectory
from climber.Climber import Climber 
from AutoSequencerV2.command import Command
from utils.allianceTransformUtils import transform
from utils.autonomousTransformUtils import flip
from utils.constants import ClimberSteps

class ClimberCommand(Command):
    def __init__(self, pathFile, extraAlignTime_s = 0.5):
        self.name = pathFile
        self.Climber = Climber()
        self.trajCtrl = Trajectory()
        self.startClimb = False
        # Get the internal path file
        absPath = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "..",
                "deploy",
                "choreo",
                pathFile,
            )
        )

        self.path = choreo.load_swerve_trajectory(absPath)
        self.done = False
        self.startTime = (
            -1
        )
       
        # we'll populate these for real later, just declare they'll exist
        self.duration = self.path.get_total_time() + extraAlignTime_s
        

    def initialize(self):
        self.startTime = Timer.getFPGATimestamp()
       
       
    def execute(self):
        self.Climber.setStep(ClimberSteps.STEP6_AUTO_CLIMB)
        
    def isDone(self):
        return self.done
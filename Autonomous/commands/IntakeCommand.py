import os
import choreo
from AutoSequencerV2.command import Command
from fuelSystems import intakeControl
from utils.allianceTransformUtils import transform
from utils.autonomousTransformUtils import flip
from fuelSystems.intakeControl import IntakeControl

class IntakeCommand(Command):
    def __init__(self, pathFile, extraAlignTime_s =0.5):
        self.name = pathFile
        self.intakeControl = IntakeControl()
        self.done = False
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
        
        def intakeControl():
            self.intakeControl = True 
            #to be worked on further.

    def isDone(self):
        return self.done
         
     
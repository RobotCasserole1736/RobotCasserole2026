from AutoSequencerV2.sequentialCommandGroup import SequentialCommandGroup
from AutoSequencerV2.builtInCommands.waitCommand import WaitCommand
from AutoSequencerV2.command import Command
from climber.Climber import Climber
#wait until in branch with climbing
class ClimbingCommand(Command):
    def __init__(self, goingToL1=False):
        self.duration = .5
        self.startTime = 0

    def initialize(self):
        Climber().setStep
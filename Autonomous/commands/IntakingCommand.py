from AutoSequencerV2.command import Command
from fuelSystems.intakeControl import IntakeControl
from fuelSystems.fuelSystemConstants import intakeWristState
from wpilib import Timer
#change because we need to make this once we are in a branch with fuel system stuff
class IntakeBallCommand(Command):
    def __init__(self):
        self.startTime = 0

    def initialize(self):
        IntakeControl().setIntakeWristState(intakeWristState.GROUND)

    def execute(self):
        IntakeControl().operatorEnableIntakeWheels(True)

    def isDone(self):
        IntakeControl().operatorEnableIntakeWheels(False)
        return True

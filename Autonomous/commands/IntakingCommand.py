from AutoSequencerV2.command import Command
from fuelSystems.intakeControl import IntakeControl
from fuelSystems.fuelSystemConstants import intakeWristState
from wpilib import Timer
#change because we need to make this once we are in a branch with fuel system stuff
class IntakeBallCommand(Command):
    def __init__(self):
        self.hasPiece = True
        self.startTime = 0

    def initialize(self):
        IntakeControl().setIntakeWristState(intakeWristState.GROUND)
        self.hasPiece = IntakeControl().getIntakeWheelsState()
        self.hadPiece = False

    def execute(self):
        self.hasPiece = IntakeControl().getIntakeWheelsState()
        if self.hasPiece and not self.hadPiece:
            self.startTime = Timer.getFPGATimestamp()
            self.hadPiece = True

    def isDone(self):
        return self.isDone

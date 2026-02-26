

from wpilib import Timer
from AutoSequencerV2.sequentialCommandGroup import SequentialCommandGroup
from AutoSequencerV2.builtInCommands.waitCommand import WaitCommand
from AutoSequencerV2.command import Command
from fuelSystems.fuelSystemConstants import shooterTargetCmd
class ShootFuelCommand(Command):
    def __init__(self):
        self.duration = .5
        self.hadFuel = True
        self.startTime = 0

    def initialize(self):
     shooterTargetCmd = shooterTargetCmd.HUB
     
      
      

    def execute(self):
        # Eject

        self.hasPiece = shooterTargetCmd.getCheckGamePiece()
        if not self.hasPiece and not self.hadPiece:
            self.startTime = Timer.getFPGATimestamp()
            self.hadPiece = True

    def maxDuration(self, duration):
        self.duration = duration + 1

    def isDone(self):
       
       # return Timer.getFPGATimestamp() - self.startTime >= self.duration
        return not self.hasPiece and self.hadPiece and Timer.getFPGATimestamp() - self.startTime >= self.duration


    def end(self,interrupt):
       shooterTargetCmd().setCoralCmd(CoralManState.DISABLED)

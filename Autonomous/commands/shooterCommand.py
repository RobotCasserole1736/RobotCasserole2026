from wpilib import Timer
from AutoSequencerV2.sequentialCommandGroup import SequentialCommandGroup
from AutoSequencerV2.builtInCommands.waitCommand import WaitCommand
from AutoSequencerV2.command import Command
from fuelSystems.shooterControl import shooterTargetCmd
class ShootFuelCommand(Command):
    def __init__(self):
        self.duration = .5
        self.hadFuel = True
        self.startTime = 0
  
    def initialize(self):
        self.startTime = Timer.getFPGATimestamp()
        self.hadFuel = True
        self.hadShot = False

    def execute(self):
        shooterTargetCmd.AUTOTARGET
        if self.hadFuel and not self.hadShot:
            self.startTime = Timer.getFPGATimestamp()
            self.hadShot = True

    def isDone(self):
        return self.hadShot and (Timer.getFPGATimestamp() - self.startTime >= self.duration)
from wpilib import Timer
from AutoSequencerV2.sequentialCommandGroup import SequentialCommandGroup
from AutoSequencerV2.builtInCommands.waitCommand import WaitCommand
from AutoSequencerV2.command import Command
from fuelSystems.shooterControl import ShooterControl, shooterDistance
from fuelSystems.intakeControl import IntakeControl
from fuelSystems.indexerControl import IndexerControl
class prepareToShoot(Command):
    def __init__(self):
        self.duration = 10
        self.hadFuel = True
        self.startTime = 0
  
    def initialize(self):
        self.startTime = Timer.getFPGATimestamp()
        self.hadFuel = True
        self.hadShot = False

    def execute(self):
        ShooterControl().enableShooting(shooterDistance.SHORT)
        IntakeControl().driverEnableIntakeWheels(False)
        IndexerControl().setIndexerIntake(True)
        if self.hadFuel and not self.hadShot:
            self.startTime = Timer.getFPGATimestamp()
            self.hadShot = True

    def isDone(self):
       return self.isDone 
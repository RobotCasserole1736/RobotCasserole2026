from wpilib import Timer
from AutoSequencerV2.command import Command
from fuelSystems.shooterControl import ShooterControl, shooterDistance
from fuelSystems.intakeControl import IntakeControl
from fuelSystems.indexerControl import IndexerControl
from utils.units import rad2Deg
class ShootFuelCommand(Command):
    def __init__(self):
        self.duration = 13
        self.startTime = 0

    def initialize(self):
        self.startTime = Timer.getFPGATimestamp()
        self.hadShot = False

    def execute(self):
        ShooterControl().enableShooting(shooterDistance.SHORTERFORAUTO)
        # if not self.hadShot:
        #     self.startTime = Timer.getFPGATimestamp()
        #     self.hadShot = True
        # if abs(Timer.getFPGATimestamp() - self.startTime) >= 1:
        IndexerControl().setIndexerIntake(True)
        IntakeControl().operatorEnableIntakeWheels(True)

    def isDone(self):
       return self.isDone

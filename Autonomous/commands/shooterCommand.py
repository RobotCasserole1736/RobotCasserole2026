
from wpilib import Timer
from AutoSequencerV2.command import Command
from utils.allianceTransformUtils import transform
from utils.autonomousTransformUtils import flip
from fuelSystems.shooterControl import ShooterController
class ShooterCommand(Command):
  
          

    def __init__(self, shootTime_s = 2.0):
            self.name = "ShooterCommand"
            self.shootTime_s = shootTime_s
            self.startTime = 1.0
            self.done = False
            self.shooter = ShooterController()
                

    def initialize(self):
            self.startTime = Timer.getFPGATimestamp()
            self.shooter.toldToShoot = True # Set shooter speed to full power


    def execute(self):
        curTime = Timer.getFPGATimestamp() - self.startTime
        if curTime >= self.shootTime_s:
         self.shooter.toldToShoot = False # Stop the shooter
         self.done = True

    def isDone(self):
        return self.done
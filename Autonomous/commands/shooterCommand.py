import os
import choreo
import choreo.trajectory
from AutoSequencerV2.command import Command
from utils.allianceTransformUtils import transform
from utils.autonomousTransformUtils import flip
from fuelSystems.shooterControl import ShooterController
class ShooterCommand(Command):
  
          

    def __init__(self, shootTime_s = 2.0):
            self.name = "ShooterCommand"
            self.done = False
            self.shooter = ShooterController()
                

    def initialize(self):
            self.shooter.toldToShoot = True # Set shooter speed to full power


    def execute(self):
        if not self.shooter.toldToShoot:
            self.done = True

    def isDone(self):
        return self.done
import os
import choreo
from wpilib import Timer
from climber.Climber import Climber 
from AutoSequencerV2.command import Command
from utils.allianceTransformUtils import transform
from utils.autonomousTransformUtils import flip
from utils.constants import ClimberSteps, LEFT_HOOK_CANID, RIGHT_HOOK_CANID
from wrappers.wrapperedSparkMax import WrapperedSparkMax

class ClimbingCommand(Command):
    def __init__(self, pathFile):
        self.name = pathFile
        self.Climber = Climber()  
        self.startClimb = False
        self.done = False
        self.lefthook_motor = WrapperedSparkMax(LEFT_HOOK_CANID, "LongHookMotor", brakeMode=True)
        self.righthook_motor = WrapperedSparkMax(RIGHT_HOOK_CANID, "SmallHookMotor", brakeMode=True)
    def initialize(self): 
     self.lefthook_motor.setPosCmd(0)
     self.righthook_motor.setPosCmd(0)
    
    def execute(self):
        self.Climber.setStep(ClimberSteps.STEP6_AUTO_CLIMB)
        
    def isDone(self):
        return self.done
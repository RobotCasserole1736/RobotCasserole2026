import os
import choreo
from wpilib import Timer
from climber.Climber import Climber 
from AutoSequencerV2.command import Command
from utils.allianceTransformUtils import transform
from utils.autonomousTransformUtils import flip
from utils.constants import ClimberSteps, LONG_HOOK_CANID, SHORT_HOOK_CANID
from wrappers.wrapperedSparkMax import WrapperedSparkMax

class ClimbingCommand(Command):
    def __init__(self, pathFile):
        self.name = pathFile
        self.Climber = Climber()  
        self.startClimb = False
        self.done = False
        self.longhook_motor = WrapperedSparkMax(LONG_HOOK_CANID, "LongHookMotor", brakeMode=True)
        self.smallhook_motor = WrapperedSparkMax(SHORT_HOOK_CANID, "SmallHookMotor", brakeMode=True)
    def initialize(self): 
     self.longhook_motor.setPosCmd(0)
     self.smallhook_motor.setPosCmd(0)
    
    def execute(self):
        self.Climber.setStep(ClimberSteps.STEP6_AUTO_CLIMB)
        
    def isDone(self):
        return self.done
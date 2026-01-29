# Use the local WrapperedSparkMax defined below in this file to avoid import/type conflicts
# Neos???
import math
from unittest import case
from rev import SparkMax, SparkMaxConfig, SparkBaseConfig
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from utils.constants import LONG_HOOK_CANID, CLIMBER_PIVOT_CANID, SHORT_HOOK_CANID, ClimberSteps
from wpilib import XboxController
class Climber:

    def Init(self):
        """Runs once when the robot starts."""
        self.longhook_motor = WrapperedSparkMax(LONG_HOOK_CANID, "LongHookMotor", brakeMode=True)
        self.pivot_motor = WrapperedSparkMax(CLIMBER_PIVOT_CANID, "PivotMotor", brakeMode=True)
        self.smallhook_motor = WrapperedSparkMax(SHORT_HOOK_CANID, "SmallHookMotor", brakeMode=True)
        self.is_moving = False
        self.move_duration = 0
        self.speed = 0
        self.distance = 0
        self.gearing = 0 
        self.step = ClimberSteps.STEP0_IDLE
    def setStep(self, newStep):
        self.step = newStep
        self.xboxDPad = XboxController(0)
    def getStep(self):
        return self.step 
           
    def setGearing(self, newGearing):
        self.gearing = newGearing
        
    def setSpeed(self, newSpeed):
        self.speed = newSpeed

    def getSpeed(self):
        return self.speed
    def getDistance(self):
        return self.distance
   
    def setDistance(self, newDistance):
        self.distance = newDistance
    def startClimb(self):
        
        # pivot motor is fine for now
        self.pivot_motor.setPosCmd(-0)
        # ensure step handling with properly indented match/case blocks
        match self.step:
            case ClimberSteps.STEP0_IDLE:
                # do nothing while idle
                pass
            case ClimberSteps.STEP1_LONGHOOK_DOWN_SHORTHOOK_UP:
                # step 1: long hook down, short hook up
                self.longhook_motor.setPosCmd(3)
                self.smallhook_motor.setPosCmd(1)
            case ClimberSteps.STEP2_SHORTHOOK_LATCHES_ONTO_BAR:
                # step 2: short hook latches onto bar
                self.smallhook_motor.setPosCmd(3)
                self.longhook_motor.setPosCmd(3)
            case ClimberSteps.STEP3_LONGHOOK_DISENGAGES_FROM_BAR:
                # step 3: long hook goes up and disengages from bar
                self.longhook_motor.setPosCmd(0)
                self.smallhook_motor.setPosCmd(3)
            case ClimberSteps.STEP4_LONGHOOK_BACK_DOWN:
                # step 4: long hook goes back down
                self.longhook_motor.setPosCmd(3)
                self.smallhook_motor.setPosCmd(3)
            
        #self.longhook_motor.getMotorPositionRad()

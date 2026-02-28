from utils.calibration import Calibration
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from utils.constants import LONG_HOOK_CANID, SHORT_HOOK_CANID, ClimberSteps
class Climber():

    def __init__(self):
        self.longhook_motorKP = Calibration('long hook motor kP', default = 0.0, units="Volts/RadPerSec")
        self.longhook_motorKS = Calibration('long hook motor KS', default=0.0)
        self.longhook_motorKG = Calibration('long hook motor KG', default=0.0, units="Volts/Rad")
        self.shorthook_motorKP = Calibration('short hook motor KP', default = 0.0, units="Volts/RadPerSec")
        self.shorthook_motorKS = Calibration('short hook motor KS', default=0.0)
        self.shorthook_motorKG = Calibration('short hook motor KG', default=0.0, units="Volts/Rad")
        self.longhook_motor = WrapperedSparkMax(LONG_HOOK_CANID, "LongHookMotor", brakeMode=True)
        self.smallhook_motor = WrapperedSparkMax(SHORT_HOOK_CANID, "SmallHookMotor", brakeMode=True)
        self.is_moving = False
        self.speed = 0
        self.distance = 0
        self.gearing = 0 
        self.step = ClimberSteps.STEP0_IDLE
      #getters and setters for Step.
    def setStep(self, newStep):
        self.step = newStep
       
    def getStep(self):
        return self.step 
    #Setters for Gearing.
    def setGearing(self, newGearing):
        self.gearing = newGearing
   #getters and setters for speed
    def setSpeed(self, newSpeed):
        self.speed = newSpeed

    def getSpeed(self):
        return self.speed
   #getters and setters for distance
    def getDistance(self):
        return self.distance
   
    def setDistance(self, newDistance):
        self.distance = newDistance
 #start of climb sequance.
    def startClimb(self):
        
        match self.step:
            case ClimberSteps.STEP0_IDLE:
                # do nothing while idle
                pass
            case ClimberSteps.STEP1_LONGHOOK_DOWN_SHORTHOOK_UP:
                # step 1: long hook down, short hook up
                self.longhook_motor.setPosCmd(3)
                self.smallhook_motor.setPosCmd(0)
                if (self.longhook_motor.getMotorPositionRad() >= 3 and self.smallhook_motor.getMotorPositionRad() <= 0):
                    self.setStep(ClimberSteps.STEP2_SHORTHOOK_LATCHES_ONTO_BAR )
            case ClimberSteps.STEP2_SHORTHOOK_LATCHES_ONTO_BAR:
                # step 2: short hook latches onto bar
                self.smallhook_motor.setPosCmd(3)
                self.longhook_motor.setPosCmd(3)
                if (self.smallhook_motor.getMotorPositionRad() >= 3 and self.longhook_motor.getMotorPositionRad() >= 3):
                    self.setStep(ClimberSteps.STEP3_LONGHOOK_DISENGAGES_FROM_BAR)
            case ClimberSteps.STEP3_LONGHOOK_DISENGAGES_FROM_BAR:
                # step 3: long hook goes up and disengages from bar
                self.longhook_motor.setPosCmd(0)
                self.smallhook_motor.setPosCmd(3)
                if (self.longhook_motor.getMotorPositionRad() <= 0 and self.smallhook_motor.getMotorPositionRad() >= 3):
                    self.setStep(ClimberSteps.STEP4_LONGHOOK_BACK_DOWN)
            case ClimberSteps.STEP4_LONGHOOK_BACK_DOWN:
                # step 4: long hook goes back down
                self.longhook_motor.setPosCmd(3)
                self.smallhook_motor.setPosCmd(3)
                if (self.longhook_motor.getMotorPositionRad() >= 3 and self.smallhook_motor.getMotorPositionRad() >= 3):
                    self.setStep(ClimberSteps.STEP0_IDLE)
            case ClimberSteps.STEP5_CLIMB_DOWN:
                # step 5: climb down
                self.longhook_motor.setPosCmd(0)
                self.smallhook_motor.setPosCmd(0)
                if (self.longhook_motor.getMotorPositionRad() <= 0 and self.smallhook_motor.getMotorPositionRad() <= 0):                
                    self.setStep(ClimberSteps.STEP0_IDLE)
                    #end of normal climb.
                    # step 6: auto climb, it will be used on its own and not with the other steps.
            case ClimberSteps.STEP6_AUTO_CLIMB:
                self.longhook_motor.setPosCmd(3)
                if (self.longhook_motor.getMotorPositionRad() >= 3):
                    self.setStep(ClimberSteps.STEP5_CLIMB_DOWN)
        
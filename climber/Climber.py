from utils.calibration import Calibration
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from utils.constants import LEFT_HOOK_CANID, RIGHT_HOOK_CANID, ClimberSteps
class Climber():

    def __init__(self):
        self.lefthook_motorKP = Calibration('long hook motor kP', default = 0.0, units="Volts/RadPerSec")
        self.lefthook_motorKS = Calibration('long hook motor KS', default=0.0)
        self.lefthook_motorKG = Calibration('long hook motor KG', default=0.0, units="Volts/Rad")
        self.righthook_motorKP = Calibration('short hook motor KP', default = 0.0, units="Volts/RadPerSec")
        self.righthook_motorKS = Calibration('short hook motor KS', default=0.0)
        self.righthook_motorKG = Calibration('short hook motor KG', default=0.0, units="Volts/Rad")
        self.lefthook_motor = WrapperedSparkMax(LEFT_HOOK_CANID, "LongHookMotor", brakeMode=True)
        self.righthook_motor = WrapperedSparkMax(RIGHT_HOOK_CANID, "SmallHookMotor", brakeMode=True)
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
                self.lefthook_motor.setPosCmd(3)
                self.righthook_motor.setPosCmd(0)
                if (self.lefthook_motor.getMotorPositionRad() >= 3 and self.righthook_motor.getMotorPositionRad() <= 0):
                    self.setStep(ClimberSteps.STEP2_SHORTHOOK_LATCHES_ONTO_BAR )
            case ClimberSteps.STEP2_SHORTHOOK_LATCHES_ONTO_BAR:
                # step 2: short hook latches onto bar
                self.righthook_motor.setPosCmd(3)
                self.lefthook_motor.setPosCmd(3)
                if (self.righthook_motor.getMotorPositionRad() >= 3 and self.lefthook_motor.getMotorPositionRad() >= 3):
                    self.setStep(ClimberSteps.STEP3_LONGHOOK_DISENGAGES_FROM_BAR)
            case ClimberSteps.STEP3_LONGHOOK_DISENGAGES_FROM_BAR:
                # step 3: long hook goes up and disengages from bar
                self.lefthook_motor.setPosCmd(0)
                self.righthook_motor.setPosCmd(3)
                if (self.lefthook_motor.getMotorPositionRad() <= 0 and self.righthook_motor.getMotorPositionRad() >= 3):
                    self.setStep(ClimberSteps.STEP4_LONGHOOK_BACK_DOWN)
            case ClimberSteps.STEP4_LONGHOOK_BACK_DOWN:
                # step 4: long hook goes back down
                self.lefthook_motor.setPosCmd(3)
                self.righthook_motor.setPosCmd(3)
                if (self.lefthook_motor.getMotorPositionRad() >= 3 and self.righthook_motor.getMotorPositionRad() >= 3):
                    self.setStep(ClimberSteps.STEP0_IDLE)
            case ClimberSteps.STEP5_CLIMB_DOWN:
                # step 5: climb down
                self.lefthook_motor.setPosCmd(0)
                self.righthook_motor.setPosCmd(0)
                if (self.lefthook_motor.getMotorPositionRad() <= 0 and self.righthook_motor.getMotorPositionRad() <= 0):                
                    self.setStep(ClimberSteps.STEP0_IDLE)
                    #end of normal climb.
                    # step 6: auto climb, it will be used on its own and not with the other steps.
            case ClimberSteps.STEP6_AUTO_CLIMB:
                self.lefthook_motor.setPosCmd(3)
                if (self.lefthook_motor.getMotorPositionRad() >= 3):
                    self.setStep(ClimberSteps.STEP5_CLIMB_DOWN)
        
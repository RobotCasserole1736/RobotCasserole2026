# from utils.calibration import Calibration
# from wrappers.wrapperedSparkMax import WrapperedSparkMax
# from utils.constants import LEFT_HOOK_CANID, RIGHT_HOOK_CANID, ClimberSteps
# class smallClimber():
# #SMALL CLIMBER CODE IF WE NEED TO DOWNSIZE FOR WEIGHT
#  def __init__(self):
#   self.lefthook_motor = WrapperedSparkMax(LEFT_HOOK_CANID, "LongHookMotor", brakeMode=True)
#   self.lefthook_motorKP = Calibration('long hook motor kP', default = 0.0, units="Volts/RadPerSec")
#   self.lefthook_motorKS = Calibration('long hook motor KS', default=0.0)
#   self.lefthook_motorKG = Calibration('long hook motor KG', default=0.0, units="Volts/Rad")
#   self.is_moving = False
#   self.speed = 0
#   self.distance = 0
#   self.gearing = 0 
#   self.step = ClimberSteps.STEP0_IDLE
#       #getters and setters for Step.
# def setStep(self, newStep):
#         self.step = newStep
       
# def getStep(self):
#         return self.step 
#     #Setters for Gearing.
# def setGearing(self, newGearing):
#         self.gearing = newGearing
#    #getters and setters for speed
# def setSpeed(self, newSpeed):
#         self.speed = newSpeed

# def getSpeed(self):
#         return self.speed
#    #getters and setters for distance
# def getDistance(self):
#         return self.distance
   
# def setDistance(self, newDistance):
#         self.distance = newDistance
# def startClimb(self):
#       match self.step:
#             case ClimberSteps.STEP0_IDLE:
#                 # do nothing while idle
#                 pass
#             case ClimberSteps.STEP5_CLIMB_DOWN:
#                 # step 5: climb down
#                 self.lefthook_motor.setPosCmd(0)
#                 if (self.lefthook_motor.getMotorPositionRad() <= 0):                
#                     self.setStep(ClimberSteps.STEP0_IDLE)
#                     #end of normal climb.
#                     # step 6: auto climb, it will be used on its own and not with the other steps.
#             case ClimberSteps.STEP6_AUTO_CLIMB:
#                 self.lefthook_motor.setPosCmd(3)
#                 if (self.lefthook_motor.getMotorPositionRad() >= 3):
#                     self.setStep(ClimberSteps.STEP5_CLIMB_DOWN)
        
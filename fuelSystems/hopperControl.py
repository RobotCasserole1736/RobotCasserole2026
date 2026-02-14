from utils.constants import HOPPER_CANID
from utils.calibration import Calibration
from utils.singleton import Singleton
from wrappers.wrapperedSparkMax import WrapperedSparkMax
class HopperControl(metaclass=Singleton):
    def __init__(self):
        self.intakeCommandState = False
        self.ejectCommandState = False
       
        self.hopperMotor = WrapperedSparkMax(HOPPER_CANID, "HopperMotor", brakeMode=True, currentLimitA=30)

        self.intakeVoltageCal = Calibration("Hopper Manipulator IntakeVoltage", 12, "V")
        self.ejectVoltageCal = Calibration("Hopper Manipulator EjectVoltage", -12, "V")

        #addLog("Algae Manipulator intake cmd",lambda:self.intakeCommandState,"Bool")
        #addLog("Algae Manipulator  cmd",lambda:self.ejectCommandState,"Bool")
        #addLog("Has Game Piece", self.getHasGamePiece, "Bool")

    def update(self):

        if self.intakeCommandState:
            self.updateIntake(True)
        elif self.ejectCommandState:
            self.updateEject(True)
        else:
            self.hopperMotor.setVoltage(0)

    def setInput(self, intakeBool, ejectBool):
        self.intakeCommandState = intakeBool
        self.ejectCommandState = ejectBool

    def updateIntake(self, run):
        voltage = self.intakeVoltageCal.get()

        if run:
            self.hopperMotor.setVoltage(voltage)
        else: 
            self.hopperMotor.setVoltage(0)
    
    def updateEject(self, run):
        voltage = self.ejectVoltageCal.get()

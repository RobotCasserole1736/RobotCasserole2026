from utils.constants import HOPPER_CANID
from utils.calibration import Calibration
from utils.singleton import Singleton
from wrappers.wrapperedSparkMax import WrapperedSparkMax
class HopperControl(metaclass=Singleton):
    def __init__(self):
        self.intakeCommandState = False
        self.ejectCommandState = False
        self.hopperMotor = WrapperedSparkMax(HOPPER_CANID, "HopperMotor", brakeMode=True, currentLimitA=30)
        self.motorVoltCal = Calibration("Hopper Manipulator IntakeVoltage", 12, "V")

        #addLog("Algae Manipulator intake cmd",lambda:self.intakeCommandState,"Bool")
        #addLog("Algae Manipulator  cmd",lambda:self.ejectCommandState,"Bool")
        #addLog("Has Game Piece", self.getHasGamePiece, "Bool")

    def update(self):
        if self.intakeCommandState:
            self.hopperMotor.setVoltage(self.motorVoltCal.get())
        elif self.ejectCommandState:
            self.hopperMotor.setVoltage(-self.motorVoltCal.get())
        else:
            self.hopperMotor.setVoltage(0)

    def enableHopperIntake(self):
        self.intakeCommandState = True

    def disableHopperIntake(self):
        self.intakeCommandState = False

    def enableHopperEject(self):
        self.ejectCommandState = True

    def disableHopperEject(self):
        self.ejectCommandState = False

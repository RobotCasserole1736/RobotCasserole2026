from utils.calibration import Calibration
from utils.constants import TURRET_FEED_CANID
from utils.singleton import Singleton
from wrappers.wrapperedSparkMax import WrapperedSparkMax

class IndexerController(metaclass=Singleton):
    def __init__(self):
        self.intakeCommand = False
        self.ejectCommand = False
        self.indexerMotor = WrapperedSparkMax(TURRET_FEED_CANID, "IndexerMotor", brakeMode=True, currentLimitA=20)
        self.motorVoltCal = Calibration("Indexer Voltage", 12, "V")

    def update(self):
        if self.intakeCommand:
            self.indexerMotor.setVoltage(self.motorVoltCal.get())
        elif self.ejectCommand:
            self.indexerMotor.setVoltage(-self.motorVoltCal.get())
        else:
            self.indexerMotor.setVoltage(0)

    def enableIndexerIntake(self):
        self.intakeCommand = True

    def enableIndexerEject(self):
        self.ejectCommand = True

    def disableIndexer(self):
        self.intakeCommand = False
        self.ejectCommand = False

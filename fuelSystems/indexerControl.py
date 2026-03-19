from utils.calibration import Calibration
from utils.constants import INDEXER_CANID
from utils.singleton import Singleton
from wrappers.wrapperedSparkMax import WrapperedSparkMax

class IndexerControl(metaclass=Singleton):
    def __init__(self):
        self.intakeCommand = False
        self.ejectCommand = False
        self.indexerMotor = WrapperedSparkMax(INDEXER_CANID, "IndexerMotor", brakeMode=False, currentLimitA=20)
        self.motorVoltCal = Calibration(name="Indexer Voltage", default=12, units="V")

    def update(self):
        if self.ejectCommand:
            self.indexerMotor.setVoltage(-self.motorVoltCal.get())
        elif self.intakeCommand:
            self.indexerMotor.setVoltage(self.motorVoltCal.get())
        else:
            self.indexerMotor.setVoltage(0)

    def setIndexerIntake(self, cmd):
        self.intakeCommand = cmd

    def setIndexerEject(self, cmd):
        self.ejectCommand = cmd

    def disableIndexer(self):
        self.intakeCommand = False
        self.ejectCommand = False

from utils.calibration import Calibration
from utils.constants import INDEXER_CANID
from utils.singleton import Singleton
from wrappers.wrapperedSparkMax import WrapperedSparkMax

class IndexerControl(metaclass=Singleton):
    def __init__(self):
        self.intakeCommand = False
        self.ejectCommand = False
        self.indexerMotor = WrapperedSparkMax(INDEXER_CANID, "IndexerMotor", brakeMode=False, currentLimitA=20)
        self.indexerMotorkP = Calibration(name="Indexer kP", default= 0.0)
        self.indexerMotorkV = Calibration(name="Indexer kV", default= 0.005)
        self.motorVelCal = Calibration(name="Indexer Velocity", default=12, units="rad/s")

    def update(self):
        if self.indexerMotorkP.isChanged() or self.indexerMotorkV.isChanged():
            self.indexerMotor.setPIDF(
            kP=self.indexerMotorkP.get(),
            kI=0,
            kD=0,
            kFF=self.indexerMotorkV.get())
        if self.ejectCommand:
            self.indexerMotor.setVelCmd(-self.motorVelCal.get())
        elif self.intakeCommand:
            self.indexerMotor.setVelCmd(self.motorVelCal.get())
        else:
            self.indexerMotor.setVoltage(0)

    def setIndexerIntake(self, cmd):
        self.intakeCommand = cmd

    def setIndexerEject(self, cmd):
        self.ejectCommand = cmd

    def disableIndexer(self):
        self.intakeCommand = False
        self.ejectCommand = False

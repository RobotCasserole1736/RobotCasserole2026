from utils.calibration import Calibration
from utils.constants import INDEXER_CANID, POWER_FLOOR_CANID
from utils.singleton import Singleton
from utils.units import RPM2RadPerSec
from wrappers.wrapperedSparkMax import WrapperedSparkMax

class IndexerControl(metaclass=Singleton):
    def __init__(self):
        self.intakeCommand = False
        self.ejectCommand = False
        self.powerFloorMotor = WrapperedSparkMax(POWER_FLOOR_CANID, "PowerFloorMotor", brakeMode=False, currentLimitA=20)
        self.indexerMotor = WrapperedSparkMax(INDEXER_CANID, "IndexerMotor", brakeMode=False, currentLimitA=20)
        self.indexerMotorkP = Calibration(name="Indexer kP", default= 0.0)
        self.indexerMotorkV = Calibration(name="Indexer kV", default= 0.0)
        self.powerFloorMotorkP = Calibration(name="Power floor kP", default=0.0)
        self.powerFloorMotorkV = Calibration(name="Power floor kV", default=0.0)
        self.motorVelCal = Calibration(name="Indexer Velocity", default=RPM2RadPerSec(115), units="RPM")
        self.floorVelCal = Calibration(name="Power Floor Velocity", default=RPM2RadPerSec(115), units="RPM")

    def update(self):
        if (self.indexerMotorkP.isChanged() or self.indexerMotorkV.isChanged() or
            self.powerFloorMotorkP.isChanged() or self.powerFloorMotorkV.isChanged()):
            self._updateAllPIDs()
        if self.ejectCommand:
            self.indexerMotor.setVelCmd(-self.motorVelCal.get())
            self.powerFloorMotor.setVelCmd(-self.motorVelCal.get())
        elif self.intakeCommand:
            self.indexerMotor.setVelCmd(self.motorVelCal.get())
            self.powerFloorMotor.setVelCmd(self.motorVelCal.get())
        else:
            self.indexerMotor.setVoltage(0)
            self.powerFloorMotor.setVoltage(0)

    def setIndexerIntake(self, cmd: bool):
        self.intakeCommand = cmd
        
    def setIndexerEject(self, cmd: bool):
        self.ejectCommand = cmd

    def disableIndexer(self):
        self.intakeCommand = False
        self.ejectCommand = False

    def _updateAllPIDs(self):
        self.indexerMotor.setPIDF(
            kP=self.indexerMotorkP.get(),
            kI=0,
            kD=0,
            kFF=self.indexerMotorkV.get())
        self.powerFloorMotor.setPIDF(
            kP=self.powerFloorMotorkP.get(),
            kI=0,
            kD=0,
            kFF=self.powerFloorMotorkV.get())

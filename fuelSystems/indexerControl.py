from utils.calibration import Calibration
from utils.constants import INDEXER_CANID, POWER_FLOOR_CANID
from utils.signalLogging import addLog
from utils.singleton import Singleton
from utils.units import RPM2RadPerSec, radPerSec2RPM
from wrappers.wrapperedSparkMax import WrapperedSparkMax

class IndexerControl(metaclass=Singleton):
    def __init__(self):
        self.intakeCommand = False
        self.ejectCommand = False
        self.powerFloorMotor = WrapperedSparkMax(POWER_FLOOR_CANID, "PowerFloorMotor", brakeMode=False, currentLimitA=30)
        self.powerFloorMotorkP = Calibration(name="Power floor kP", default=0.0)
        self.powerFloorMotorkV = Calibration(name="Power floor kV", default=0.008)
        self.powerFloorVel = Calibration(name="Power Floor Velocity", default=115, units="RPM")
        self.indexerMotor = WrapperedSparkMax(INDEXER_CANID, "IndexerMotor", brakeMode=False, currentLimitA=30)
        self.indexerMotorkP = Calibration(name="Indexer kP", default= 0.0)
        self.indexerMotorkV = Calibration(name="Indexer kV", default= 0.15)
        self.indexerMotorVel = Calibration(name="Indexer Velocity", default=115, units="RPM")
        self._updateAllPIDs()

        # Power Floor Speed Logs
        addLog("Power Floor Motor Desired Speed",
               lambda: self.powerFloorVel.get(), "RPM")
        addLog("Power Floor Motor Actual Speed",
               lambda: radPerSec2RPM(self.powerFloorMotor.getMotorVelocityRadPerSec()), "RPM")
        # Indexer Speed Logs
        addLog("Indexer Motor Desired Speed",
               lambda: self.indexerMotorVel.get(), "RPM")
        addLog("Indexer Motor Actual Speed",
               lambda: radPerSec2RPM(self.indexerMotor.getMotorVelocityRadPerSec()), "RPM")

    def update(self):
        # Update PIDs
        if (self.indexerMotorkP.isChanged() or self.indexerMotorkV.isChanged() or
            self.powerFloorMotorkP.isChanged() or self.powerFloorMotorkV.isChanged()):
            self._updateAllPIDs()
        # Eject
        if self.ejectCommand:
            self.indexerMotor.setVelCmd(RPM2RadPerSec(-self.indexerMotorVel.get()))
            self.powerFloorMotor.setVelCmd(RPM2RadPerSec(-self.powerFloorVel.get()))
        # Intake
        elif self.intakeCommand:
            self.indexerMotor.setVelCmd(RPM2RadPerSec(self.indexerMotorVel.get()))
            self.powerFloorMotor.setVelCmd(RPM2RadPerSec(self.powerFloorVel.get()))
        # Disable
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

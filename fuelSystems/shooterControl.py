from fuelSystems.fuelSystemConstants import shooterDistance
from utils.calibration import Calibration
from utils.constants import TURRET_FEED_CANID, MAIN_SHOOTER_CANID
from utils.signalLogging import addLog
from utils.units import RPM2RadPerSec, radPerSec2RPM
from utils.singleton import Singleton
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.wrapperedKraken import WrapperedKraken

class ShooterControl(metaclass=Singleton):
    def __init__(self):
        # Shooter Motor
        self.shooterMainMotor = WrapperedKraken(MAIN_SHOOTER_CANID, "ShooterMotorMain", brakeMode=True)
        self.shooterMainMotor.setInverted(True)
        self.shooterMainShotType = shooterDistance.NONE
        self.shooterMainMotorkP = Calibration("shooterMain motor KP", default=0.2, units="Volts/RadPerSec")
        self.shooterMainMotorKS = Calibration("shooterMain motor KS", default=0)
        self.shooterMainMotorKV = Calibration("shooterMain motor KV", default=0.2)
        self.shooterMainMotorKA = Calibration("shooterMain motor KA", default=3.8)
        self.shooterMainShortVelocity = Calibration("shooterMain Short Velocity", default=1400, units="RPM")
        self.shooterMainLongVelocity = Calibration("shooterMain Long Velocity", default=2200, units="RPM")

        # Feed Motor
        self.feedMotor = WrapperedSparkMax(TURRET_FEED_CANID, "TurretMotorFeed", brakeMode=True)
        self.feedMotorkP = Calibration(name="Feed Motor kP", default= 0.0)
        self.feedMotorkV = Calibration(name="Feed Motor kV", default= 0.01)
        self.feedMotorVelocity = Calibration("Feeder Motor Velocity", default=115, units="RPM")

        # Set PID parameters
        self._updateAllPIDs()

        self.toldToShoot = False
        self.desMainShooterVelRad = 0.0

        # Shooter Motor Logs
        addLog("Desired Main Shooter Speed",
                lambda: radPerSec2RPM(self.desMainShooterVelRad), units="RPM")
        addLog("Actual Main Shooter Speed",
                lambda: radPerSec2RPM(self.shooterMainMotor.getMotorVelocityRadPerSec()), units="RPM")

        addLog("Desired Feed Motor Speed",
               lambda: self.feedMotorVelocity.get(), units="RPM")
        addLog("Actual Feed Motor Speed",
               lambda: radPerSec2RPM(self.feedMotor.getMotorVelocityRadPerSec()), units="RPM")

    def update(self):
        # Update PIDs if calibrations have changed
        if (self.shooterMainMotorkP.isChanged() or self.shooterMainMotorKS.isChanged() or
            self.shooterMainMotorKV.isChanged() or self.shooterMainMotorKA.isChanged() or
            self.feedMotorkP.isChanged() or self.feedMotorkV.isChanged()):
            self._updateAllPIDs()

        if self.toldToShoot:
            # Run Feed Motor
            self.feedMotor.setVelCmd(RPM2RadPerSec(self.feedMotorVelocity.get()))

            # Determine Main Shooter Speed and Run
            if self.shooterMainShotType == shooterDistance.SHORT:
                self.desMainShooterVelRad = RPM2RadPerSec(self.shooterMainShortVelocity.get())
                self.shooterMainMotor.setVelCmd(RPM2RadPerSec(self.shooterMainShortVelocity.get()),
                                               self.shooterMainMotorKS.get())
            elif self.shooterMainShotType == shooterDistance.LONG:
                self.shooterMainMotor.setVelCmd(RPM2RadPerSec(self.shooterMainLongVelocity.get()),
                                               self.shooterMainMotorKS.get())

            self.shooterMainMotor.setVelCmd(self.desMainShooterVelRad)

        # Otherwise disable feed and shoot motors
        else:
            self.feedMotor.setVoltage(0)
            self.shooterMainMotor.setVoltage(0)

    def enableShooting(self, distance: shooterDistance):
        self.toldToShoot = True
        self.shooterMainShotType = distance

    def disableShooting(self):
        self.toldToShoot = False

    def enableTargeting(self):
        self.toldToTarget = True

    def disableTargeting(self):
        self.toldToTarget = False

    def _updateAllPIDs(self):
        self.shooterMainMotor.setPID(
            kP=self.shooterMainMotorkP.get(),
            kI=0.0,
            kD=0.0,
            kV=self.shooterMainMotorKV.get(),
            kA=self.shooterMainMotorKA.get())
        self.feedMotor.setPIDF(
            kP=self.feedMotorkP.get(),
            kI=0,
            kD=0,
            kFF=self.feedMotorkV.get())

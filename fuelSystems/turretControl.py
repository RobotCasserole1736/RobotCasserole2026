from fuelSystems.fuelSystemConstants import \
    YAW_MOTOR_RATIO, \
    TURRET_MAX_YAW_RAD, \
    TURRET_MIN_YAW_RAD, \
    TURRET_ENABLE
from utils.calibration import Calibration
from utils.constants import TURRET_YAW_CANID
from utils.signalLogging import addLog
from utils.singleton import Singleton
from utils.units import rad2Deg
from wrappers.wrapperedSparkMax import WrapperedSparkMax

class TurretControl(metaclass=Singleton):
    def __init__(self) -> None:
        # Calibration for testing turret control
        self.yawTestCmdDeg = Calibration("Yaw Test Command", default=10)

        # Yaw motor for turret
        self.yawMotorkS = Calibration("Yaw motor KS", default=0.0)
        self.yawMotorkP = Calibration("Yaw motor KP", default=0.0)
        self.yawMotor = WrapperedSparkMax(
            TURRET_YAW_CANID, "TurretMotorYaw", brakeMode=True, currentLimitA=20.0)
        self.yawMotor.setPID(self.yawMotorkP.get(),0.0,0.0)

        self.desYawRad = 0.0
        self.turretActive = False

        addLog("Desired Yaw Angle",
               lambda: rad2Deg(self.desYawRad / YAW_MOTOR_RATIO), units="deg")
        addLog("Actual Yaw Angle",
                lambda: rad2Deg(self.getYawRad()), units="deg")

    def update(self) -> None:
        # Update kP if needed
        if self.yawMotorkP.isChanged():
            self.yawMotor.setPID(self.yawMotorkP.get(),0.0,0.0)

        # Command turret to desired position if needed
        if TURRET_ENABLE and self.turretActive:
            self.yawMotor.setPosCmd(self.desYawRad * YAW_MOTOR_RATIO)
        else:
            self.yawMotor.setVoltage(0)

    def enableTurret(self) -> None:
        self.turretActive = True

    def disableTurret(self) -> None:
        self.turretActive = False

    def setYawPos(self,yawCmdRad: float) -> None:
        self.desYawRad = min(TURRET_MAX_YAW_RAD, max(TURRET_MIN_YAW_RAD, yawCmdRad))

    def getYawRad(self) -> float:
        return self.yawMotor.getMotorPositionRad() / YAW_MOTOR_RATIO

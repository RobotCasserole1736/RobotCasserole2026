from fuelSystems.fuelSystemConstants import \
    YAW_MOTOR_RATIO, \
    TURRET_MAX_YAW_RAD, \
    TURRET_MIN_YAW_RAD, \
    TURRET_ENABLE
from utils.calibration import Calibration
from utils.constants import TURRET_PITCH_CANID, TURRET_YAW_CANID
from utils.signalLogging import addLog
from utils.singleton import Singleton
from utils.units import rad2Deg
from wrappers.wrapperedSparkMax import WrapperedSparkMax

class TurretControl(metaclass=Singleton):
    def __init__(self) -> None:
        if TURRET_ENABLE:
            # Calibration for testing turret control
            self.yawTestCmdDeg = Calibration("Yaw Test Command", default=10)

            # Pitch Motor
            self.pitchMotor = WrapperedSparkMax(
                TURRET_PITCH_CANID, "TurretMotorPitch", brakeMode=True, currentLimitA=20.0)
            self.pitchMotorkS = Calibration("Pitch Motor KS", default=0.0)
            self.pitchMotorkP = Calibration("Pitch Motor KP", default=0.0)
            self.pitchMotor.setPID(self.pitchMotorkP.get(),0.0,0.0)

            # Yaw Motor
            self.yawMotor = WrapperedSparkMax(
                TURRET_YAW_CANID, "TurretMotorYaw", brakeMode=True, currentLimitA=20.0)
            self.yawMotorkS = Calibration("Yaw Motor KS", default=0.0)
            self.yawMotorkP = Calibration("Yaw Motor KP", default=0.0)
            self.yawMotor.setPID(self.yawMotorkP.get(),0.0,0.0)

            self.desPitchRad = 0.0
            self.desYawRad = 0.0
            self.turretActive = False

            addLog("Desired Yaw Angle",
                lambda: rad2Deg(self.desYawRad / YAW_MOTOR_RATIO), units="deg")
            addLog("Actual Yaw Angle",
                lambda: rad2Deg(self._getYawRad()), units="deg")
        else:
            pass

    def update(self) -> None:
        if TURRET_ENABLE:
            # Update kP if needed
            if self.pitchMotorkP.isChanged or self.yawMotorkP.isChanged():
                self.pitchMotor.setPID(self.pitchMotorkP.get(),0.0,0.0)
                self.yawMotor.setPID(self.yawMotorkP.get(),0.0,0.0)

            # Command turret to desired position if needed
            if TURRET_ENABLE and self.turretActive:
                self.yawMotor.setPosCmd(self.desYawRad * YAW_MOTOR_RATIO)
            else:
                self.yawMotor.setVoltage(0)
        else:
            pass

    def enableTurret(self,cmd) -> None:
        self.turretActive = cmd

    def setPitch(self,angleRad: float) -> None:
        self.desPitchRad = angleRad

    def setYawPos(self,yawCmdRad: float) -> None:
        self.desYawRad = min(TURRET_MAX_YAW_RAD, max(TURRET_MIN_YAW_RAD, yawCmdRad))

    def _getYawRad(self) -> float:
        return self.yawMotor.getMotorPositionRad() / YAW_MOTOR_RATIO

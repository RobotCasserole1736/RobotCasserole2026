from fuelSystems.fuelSystemConstants import INTAKE_ANGLE_ABS_POS_ENC_OFFSET, intakeWristState
from utils.calibration import Calibration
from utils.signalLogging import addLog
from utils.singleton import Singleton
from utils.constants import INTAKE_CONTROL_CANID, INTAKE_WHEELS_CANID,INTAKE_ENC_PORT
from utils.units import deg2Rad, rad2Deg
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.wrapperedThroughBoreHexEncoder import WrapperedThroughBoreHexEncoder

class IntakeControl(metaclass=Singleton):

    def __init__(self):
        # Encoder and Motors
        self.intakeAbsEnc = WrapperedThroughBoreHexEncoder(
            port = INTAKE_ENC_PORT,
            name="Intake_Wrist_enc",
            mountOffsetRad = deg2Rad(INTAKE_ANGLE_ABS_POS_ENC_OFFSET),
            dirInverted = True)
        self.intakeWristMotor = WrapperedSparkMax(
            INTAKE_CONTROL_CANID,
            name = "Intake Wrist Motor",
            brakeMode = True,
            currentLimitA = 30.0)
        self.intakeWheelsMotor = WrapperedSparkMax(
            INTAKE_WHEELS_CANID,
            "Intake Wheels Motor"
            )

        # PID Calibrations
        self.kP = Calibration(
            name="Intake Wrist kP",
            default = 0.0,
            units="V/degErr")
        self.maxV = Calibration(
            name="Intake Wrist maxV",
            default = 6.0,
            units="V")
        self.deadzone = Calibration(
            name="Intake Wrist deadzone",
            default = 4.0,
            units="deg")

        # position calibrations
        # an angle in degrees. Assumingt 0 is horizontal, - is down, etc.
        self.groundPos = Calibration(
            name = "Intake Wrist Intake Off Ground Position",
            default = 165.1,
            units="deg")
        self.stowPos = Calibration(
            name="Intake Wrist Stow Position",
            default = 78.0,
            units="deg")

        #positions
        self.actualPos = 0
        self.curPosCmdDeg = self.stowPos.get()

        # starts in stow position but doesn't know that until first update
        self.curWristState = intakeWristState.NONE
        self.driverIntakeEnabled = False
        self.operatorIntakeEnabled = False
        self.operatorIntakeReversedEnabled = False
        self.isFast = False
        self.motorSpeedCal = Calibration(name="Intake  Slow Voltage", default=3000, units="RPM")
        self.motorFastSpeedCal = Calibration(name="Intake Fast Voltage", default=5000, units="RPM")
        self.intakeMainMotorkP = Calibration("shooterMain motor KP", default=0, units="Volts/RadPerSec")
        self.intakeMainMotorKS = Calibration("shooterMain motor KS", default=0)
        self.intakeMainMotorkFF = Calibration("shooterMain motor KV", default=0)

        addLog("Intake Wrist Desired Angle",
               lambda: self.curPosCmdDeg,
               "deg")
        addLog("Intake Wrist Actual Angle",
               lambda: rad2Deg(self._getAngleRad()),
                "deg")
        
        addLog("Intake Wrist Volt Command",
               lambda: (self.curPosCmdDeg - self.actualPos)*self.kP.get(),
               "V")

    def update(self):

        if (self.intakeMainMotorkP.isChanged() or
            self.intakeMainMotorKS.isChanged() or self.intakeMainMotorkFF.isChanged()):
            self._updateAllPIDs()

        # Update intake wheels
        if self.operatorIntakeReversedEnabled:
            self.intakeWheelsMotor.setVelCmd(self.motorSpeedCal.get())
        elif (self.driverIntakeEnabled or self.operatorIntakeEnabled) and not self.isFast:
            self.intakeWheelsMotor.setVelCmd(-self.motorSpeedCal.get())
        elif (self.driverIntakeEnabled or self.operatorIntakeEnabled) and self.isFast:
            self.intakeWheelsMotor.setVelCmd(-self.motorFastSpeedCal.get())
        else:
            self.intakeWheelsMotor.setVoltage(0)

        # Update wrist motor
        if (self.intakeAbsEnc.isFaulted()):
            vCmd = 0.0 # faulted, so stop
        else:
            self.intakeAbsEnc.update()
            self.actualPos = rad2Deg(self._getAngleRad())

            # If in deadzone or nothing commanded, do nothing
            err = self.curPosCmdDeg - self.actualPos
            if (abs(err) <= self.deadzone.get() or
                self.curWristState == intakeWristState.NONE):
                vCmd = 0
            # Error outside deadzone and command is given
            else:
                # Adjust error so that it's offset by the deadzone
                if (err > 0):
                    err = err - self.deadzone.get()
                else:
                    err = err + self.deadzone.get()

                vCmd = self.kP.get() * err
                vCmd = min(self.maxV.get(), max(-self.maxV.get(), vCmd))
                self.intakeWristMotor.setVoltage(vCmd)

    # Helper functions for intake wheels
    def driverEnableIntakeWheels(self, isFast : bool):
        self.driverIntakeEnabled = True
        self.isFast = isFast

    def driverDisableIntakeWheels(self):
        self.driverIntakeEnabled = False

    def getDriverIntakeWheelsState(self):
        return self.driverIntakeEnabled

    def operatorEnableIntakeWheels(self, isFast : bool):
        self.operatorIntakeEnabled = True
        self.isFast = isFast

    def operatorDisableIntakeWheels(self):
        self.operatorIntakeEnabled = False

    def getOperatorIntakeWheelsState(self):
        return self.operatorIntakeEnabled

    def operatorIntakeReversed(self):
        self.operatorIntakeReversedEnabled = True

    def operatorIntakeReversedDisabled(self):
        self.operatorIntakeReversedEnabled = False

    # Helper functions for intake wrist
    def extendIntake(self) -> None:
        self.curWristState = intakeWristState.GROUND
        self.curPosCmdDeg = self.groundPos.get()

    def stowIntake(self) -> None:
        self.curWristState = intakeWristState.STOW
        self.curPosCmdDeg = self.stowPos.get()

    def getIntakeWristState(self):
        return self.curWristState

    def _getAngleRad(self):
        return self.intakeAbsEnc.getAngleRad()
    
    def _updateAllPIDs(self):
        self.intakeWheelsMotor.setPIDF(
        self.intakeMainMotorkP.get(),0,0,
        self.intakeMainMotorkFF.get())

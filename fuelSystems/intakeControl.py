from fuelSystems.fuelSystemConstants import INTAKE_ANGLE_ABS_POS_ENC_OFFSET, intakeWristState
from math import cos
from utils.calibration import Calibration
from utils.signalLogging import addLog
from utils.singleton import Singleton
from utils.constants import INTAKE_CONTROL_CANID, INTAKE_WHEELS_CANID,INTAKE_ENC_PORT
from utils.units import deg2Rad, rad2Deg, RPM2RadPerSec, radPerSec2RPM
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.wrapperedThroughBoreHexEncoder import WrapperedThroughBoreHexEncoder
import time

class IntakeControl(metaclass=Singleton):

    def __init__(self):
        # Encoder and Motors
        # Encoder offset should make reading 90 degrees in stow position
        # and 0 degrees in ground position
        self.intakeAbsEnc = WrapperedThroughBoreHexEncoder(
            port=INTAKE_ENC_PORT, name="Intake_Wrist_enc",
            mountOffsetRad=deg2Rad(INTAKE_ANGLE_ABS_POS_ENC_OFFSET), dirInverted=True)
        self.intakeWristMotor = WrapperedSparkMax(
            INTAKE_CONTROL_CANID, name="Intake Wrist Motor", brakeMode=True, currentLimitA = 30.0)
        self.intakeWristMotor.setInverted(True)
        self.intakeWheelsMotor = WrapperedSparkMax(INTAKE_WHEELS_CANID, "Intake Wheels Motor")

        # Intake Wrist Control Calibrations
        self.kP = Calibration(name="Intake Wrist kP", default=0.04, units="V/degErr")
        self.kI = Calibration(name="Intake Wrist kI", default=0.0, units="V/degErr/s")
        self.kG = Calibration(name="Intake Wrist kG", default=0.9, units="V/cos(deg)")
        self.maxV = Calibration(name="Intake Wrist maxV", default=12.0, units="V")
        self.upHelpV = Calibration(name="Intake Wrist Up Voltage", default=1.5, units="V")
        self.deadzone = Calibration(name="Intake Wrist deadzone", default=4.0, units="deg")

        # Intake Wrist Position Calibrations
        self.groundPos = Calibration(name="Intake Wrist Intake Off Ground Position", default=0.0, units="deg")
        self.stowPos = Calibration(name="Intake Wrist Stow Position", default=90.0, units="deg")

        # Intake Wrist Position Variable
        self.actualPos = 0
        self.curPosCmdDeg = self.stowPos.get()
        # integral accumulator for kI term (units: deg * s)
        self._int_err = 0.0
        self._last_update_time = time.monotonic()

        # Starts in stow position but doesn't know that until first update
        self.curWristState = intakeWristState.NONE
        self.driverIntakeEnabled = False
        self.operatorIntakeEnabled = False
        self.operatorIntakeReversedEnabled = False

        # Intake Wheels Calibrations
        self.intakeWheelsMotorSpd = Calibration(name="Intake Wheels Motor Speed", default=4000, units="RPM")
        self.intakeWheelskFF = Calibration("Intake Wheels Motor KFF", default=0.00016)
        self.intakeWheelskP = Calibration("Intake Wheels Motor KP", default=0.0001, units="Volts/RadPerSec")

        # Apply PIDs
        self._updateAllPIDs()

        # Intake Wrist Logs
        addLog("Intake Wrist Desired Angle",
               lambda: self.curPosCmdDeg, "deg")
        addLog("Intake Wrist Actual Angle",
               lambda: rad2Deg(self._getAngleRad()), "deg")
        addLog("Intake Wrist Volt Command",
               lambda: (self.curPosCmdDeg - self.actualPos)*self.kP.get() + self._int_err * self.kI.get() + self.kG.get()*cos(self.actualPos) + self.upHelpV.get(), "V")

        # Intake Wheels Logs
        addLog("Intake Wheels Desired Speed",
               lambda: self.intakeWheelsMotorSpd.get(), "RPM")
        addLog("Intake Wheels Actual Speed",
               lambda: radPerSec2RPM(self.intakeWheelsMotor.getMotorVelocityRadPerSec()), "RPM")

    def update(self):
        # Note: Wrist cals are used directly, so do not need to update
        if (self.intakeWheelskP.isChanged() or self.intakeWheelskFF.isChanged()):
            self._updateAllPIDs()

        # Update intake wheels
        if self.operatorIntakeReversedEnabled:
            self.intakeWheelsMotor.setVelCmd(RPM2RadPerSec(self.intakeWheelsMotorSpd.get()))

        elif self.operatorIntakeEnabled:
            self.intakeWheelsMotor.setVelCmd(RPM2RadPerSec(-self.intakeWheelsMotorSpd.get()))

        else:
            self.intakeWheelsMotor.setVoltage(0)

        # Update wrist motor
        if (self.intakeAbsEnc.isFaulted() or self.curWristState == intakeWristState.NONE):
            vCmd = 0.0 # faulted or no command, so stop
            # reset integral on fault or no command
            self._int_err = 0.0
        else:
            self.intakeAbsEnc.update()
            self.actualPos = rad2Deg(self._getAngleRad())

            # If in deadzone or nothing commanded, do nothing
            err = self.curPosCmdDeg - self.actualPos
            if (abs(err) <= self.deadzone.get() or
                self.curWristState == intakeWristState.NONE):
                vCmd = 0
                # reset integral while inactive
                self._int_err = 0.0
            # Error outside deadzone and command is given
            else:
                # Adjust error so that it's offset by the deadzone
                if (err > 0):
                    err = err - self.deadzone.get()
                else:
                    err = err + self.deadzone.get()

                # integrate error (time-based)
                now = time.monotonic()
                dt = now - self._last_update_time
                # clamp dt to reasonable range to avoid large jumps
                if dt <= 0 or dt > 0.5:
                    dt = 0.02
                self._last_update_time = now

                # accumulate integral
                self._int_err += err * dt

                # anti-windup: clamp integral so I-term can't exceed maxV
                kI_val = self.kI.get()
                if abs(kI_val) > 1e-12:
                    max_int = abs(self.maxV.get() / kI_val)
                    # small safety margin
                    if max_int > 0:
                        if self._int_err > max_int:
                            self._int_err = max_int
                        elif self._int_err < -max_int:
                            self._int_err = -max_int

                # compute command including I term
                vCmd = self.kP.get() * err + (kI_val * self._int_err) + self.kG.get() * cos(self.actualPos)
                if self.curWristState == intakeWristState.STOW:
                    vCmd += self.upHelpV.get()
                vCmd = min(self.maxV.get(), max(-self.maxV.get(), vCmd))
                self.intakeWristMotor.setVoltage(vCmd)

    # Helper functions for intake wheels
    def driverEnableIntakeWheels(self,cmd: bool) -> None:
        self.driverIntakeEnabled = cmd

    def operatorEnableIntakeWheels(self,cmd: bool) -> None:
        self.operatorIntakeEnabled = cmd
    
    def operatorEnableIntakeWheelsReverse(self,cmd: bool) -> None:
        self.operatorIntakeReversedEnabled = cmd

    def getDriverIntakeWheelsState(self) -> bool:
        return self.driverIntakeEnabled

    def getOperatorIntakeWheelsState(self) -> bool:
        return self.operatorIntakeEnabled

    def operatorIntakeReversed(self,cmd: bool) -> None:
        self.operatorIntakeReversedEnabled = cmd

    # Helper functions for intake wrist
    def extendIntake(self) -> None:
        self.curWristState = intakeWristState.GROUND
        self.curPosCmdDeg = self.groundPos.get()

    def stowIntake(self) -> None:
        self.curWristState = intakeWristState.STOW
        self.curPosCmdDeg = self.stowPos.get()

    # Disable everything on intake
    def disableIntake(self) -> None:
        self.curWristState = intakeWristState.NONE
        self.operatorIntakeEnabled = False
        self.driverIntakeEnabled = False

    def getIntakeWristState(self):
        return self.curWristState

    def _getAngleRad(self):
        return self.intakeAbsEnc.getAngleRad()

    def _updateAllPIDs(self):
        self.intakeWheelsMotor.setPIDF(
            self.intakeWheelskP.get(),
            0,
            0,
            self.intakeWheelskFF.get()
        )

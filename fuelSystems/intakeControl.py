from fuelSystems.fuelSystemConstants import INTAKE_ANGLE_ABS_POS_ENC_OFFSET, intakeWristState
from math import cos
from utils.calibration import Calibration
from utils.signalLogging import addLog
from utils.singleton import Singleton
from utils.constants import INTAKE_CONTROL_CANID, INTAKE_WHEELS_CANID,INTAKE_ENC_PORT
from utils.units import deg2Rad, rad2Deg, RPM2RadPerSec, radPerSec2RPM
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.wrapperedThroughBoreHexEncoder import WrapperedThroughBoreHexEncoder

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
        self.kPUp = Calibration(name="Intake Wrist Up kP", default=0.04, units="V/degErr")
        self.kPDown = Calibration(name="Intake Wrist Down kP", default=0.02, units="V/degErr")
        self.kG = Calibration(name="Intake Wrist kG", default=0.7, units="V/cos(deg)")
        self.maxV = Calibration(name="Intake Wrist maxV", default=9.0, units="V")
        self.upHelpV = Calibration(name="Intake Wrist Up Voltage", default=1.5, units="V")
        self.downForceV = Calibration(name="Intake Wrist Down Force", default=-1.5, units="V")
        self.deadzone = Calibration(name="Intake Wrist deadzone", default=4.0, units="deg")

        # Intake Wrist Position Calibrations
        self.groundPos = Calibration(name="Intake Wrist Intake Off Ground Position", default=0.0, units="deg")
        self.stowPos = Calibration(name="Intake Wrist Stow Position", default=80.0, units="deg")

        # Intake Wrist Position Variable
        self.actualPos = 0
        self.curPosCmdDeg = self.stowPos.get()

        # Starts in stow position but doesn't know that until first update
        self.curWristState = intakeWristState.NONE
        self.driverIntakeEnabled = False
        self.operatorIntakeEnabled = False
        self.operatorIntakeReversedEnabled = False

        # Intake Wheels Calibrations
        self.intakeWheelsMotorSpd = Calibration(name="Intake Wheels Motor Speed", default=5000, units="RPM")
        self.intakeWheelskFF = Calibration("Intake Wheels Motor KFF", default=0.00017)
        self.intakeWheelskP = Calibration("Intake Wheels Motor KP", default=0.0001, units="Volts/RadPerSec")

        # Apply PIDs
        self._updateAllPIDs()

        # Intake Wrist Logs
        addLog("Intake Wrist Desired Angle",
               lambda: self.curPosCmdDeg, "deg")
        addLog("Intake Wrist Actual Angle",
               lambda: rad2Deg(self._getAngleRad()), "deg")

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
        if self.intakeAbsEnc.isFaulted():
            vCmd = 0.0 # faulted or no command, so stop
        elif self.curWristState == intakeWristState.NONE:
            vCmd = 0.0
            self.intakeAbsEnc.update()
        # Control wrist to desired position
        else:
            self.intakeAbsEnc.update()
            self.actualPos = rad2Deg(self._getAngleRad())

            # If in deadzone or nothing commanded, do nothing
            err = self.curPosCmdDeg - self.actualPos
            if abs(err) <= self.deadzone.get():
                vCmd = 0
            # If in ground position and being commanded down, give some voltage to stay down
            elif abs(err) <= self.deadzone.get() and self.curWristState == intakeWristState.GROUND:
                vCmd = self.downForceV.get()
                self.intakeWristMotor.setVoltage(vCmd)
            # Error outside deadzone and command is given
            else:
                # Determine desired position
                if self.curWristState == intakeWristState.GROUND:
                    self.curPosCmdDeg = self.groundPos.get()
                elif self.curWristState == intakeWristState.STOW:
                    self.curPosCmdDeg = self.stowPos.get()

                # Adjust error so that it's offset by the deadzone
                if (err > 0):
                    err = err - self.deadzone.get()
                else:
                    err = err + self.deadzone.get()

                # Compute voltage command for up or down
                if self.curWristState == intakeWristState.STOW:
                    vCmd = self.kPUp.get()*err + self.upHelpV.get()
                else:
                    vCmd = self.kPDown.get()*err

                # Adding kG term
                vCmd += self.kG.get()*cos(self.actualPos)

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

    def getIntakeWheelsState(self) -> bool:
        return self.driverIntakeEnabled or self.operatorIntakeEnabled

    def operatorIntakeReversed(self,cmd: bool) -> None:
        self.operatorIntakeReversedEnabled = cmd

    # Helper functions for intake wrist
    def setIntakeWristState(self,cmd: intakeWristState) -> None:
        self.curWristState = cmd

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

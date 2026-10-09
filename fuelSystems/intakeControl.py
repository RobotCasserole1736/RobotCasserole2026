from fuelSystems.fuelSystemConstants import INTAKE_WRIST_ABS_ENC_OFFSET_RAD, intakeWristState
from math import cos
from utils.calibration import Calibration
from utils.signalLogging import addLog
from utils.singleton import Singleton
from utils.constants import INTAKE_CONTROL_CANID, INTAKE_WHEELS_CANID,INTAKE_ENC_PORT
from utils.units import rad2Deg, RPM2RadPerSec, radPerSec2RPM
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.wrapperedThroughBoreHexEncoder import WrapperedThroughBoreHexEncoder
from numpy import interp

class IntakeControl(metaclass=Singleton):
    def __init__(self):
        # Encoder and Wrist Motor
        # Encoder offset should make reading 90 degrees in stow position
        # and 0 degrees in ground position
        self.intakeAbsEnc = WrapperedThroughBoreHexEncoder(
            port=INTAKE_ENC_PORT, name="Intake_Wrist_enc",
            mountOffsetRad=INTAKE_WRIST_ABS_ENC_OFFSET_RAD,
            dirInverted=True)
        self.intakeWristMotor = WrapperedSparkMax(
            INTAKE_CONTROL_CANID, name="Intake Wrist Motor", brakeMode=True, currentLimitA = 35.0)

        # Intake Wrist Control Calibrations
        self.kS = Calibration(name="Intake Wrist kS",default=0.4,units="V")
        self.kG = Calibration(name="Intake Wrist kG", default=0.7, units="V/cos(deg)")
        self.maxV = Calibration(name="Intake Wrist maxV", default=9.0, units="V")
        # kP going up will be reduced as it goes up to avoid slamming
        # Idea is that the slack will be removed at first and static friction overcome,
        # then more fine control can be used
        self.kPUp1 = Calibration(name="Intake Wrist Up kP 1", default=0.01, units="V/degErr")
        self.kPUp2 = Calibration(name="Intake Wrist Up kP 2", default=0.03, units="V/degErr")
        self.kPUp3 = Calibration(name="Intake Wrist Up kP 3", default=0.06, units="V/degErr")
        self.kPUp4 = Calibration(name="Intake Wrist Up kP 4", default=0.08, units="V/degErr")
        self.kPUpArr = [self.kPUp1.get(), self.kPUp2.get(), self.kPUp3.get(), self.kPUp4.get()]
        # Need to map the kP values to error
        self.kPUpErr1 = Calibration(name="Intake Wrist Up Err 1", default=0, units="deg")
        self.kPUpErr2 = Calibration(name="Intake Wrist Up Err 2", default=27, units="deg")
        self.kPUpErr3 = Calibration(name="Intake Wrist Up Err 3", default=54, units="deg")
        self.kPUpErr4 = Calibration(name="Intake Wrist Up Err 4", default=80, units="deg")
        self.errUpArr = [self.kPUpErr1.get(), self.kPUpErr2.get(), self.kPUpErr3.get(), self.kPUpErr4.get()]
        # self.upHelpV = Calibration(name="Intake Wrist Up Voltage", default=1.0, units="V")
        # Control parameters for lowering wrist
        self.kPDown = Calibration(name="Intake Wrist Down kP", default=0.01, units="V/degErr")
        self.downForceV = Calibration(name="Intake Wrist Down Force", default=-6.0, units="V")
        self.deadzone = Calibration(name="Intake Wrist deadzone", default=4.0, units="deg")

        # Intake Wrist Position Calibrations
        self.groundPos = Calibration(name="Intake Wrist Ground Position", default=0.0, units="deg")
        self.stowPos = Calibration(name="Intake Wrist Stow Position", default=80.0, units="deg")

        # Intake Wrist Position Variable
        self.actualPosDeg = 0
        self.curPosCmdDeg = self.stowPos.get()

        # Start with commanded movement
        self.curWristState = intakeWristState.NONE
        self.bWristStatePersist = False
        self.driverIntakeEnabled = False
        self.operatorIntakeEnabled = False
        self.operatorIntakeReversedEnabled = False

        # Intake Wheels Motor
        self.intakeWheelsMotor = WrapperedSparkMax(INTAKE_WHEELS_CANID, "Intake Wheels Motor")
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
        if (self.kPUp1.isChanged() or self.kPUp2.isChanged() or
            self.kPUp3.isChanged() or self.kPUp4.isChanged() or
            self.kPUpErr1.isChanged() or self.kPUpErr2.isChanged() or
            self.kPUpErr3.isChanged() or self.kPUpErr4.isChanged()):
            self._updatekPUp()

        # Update intake wheels
        if self.operatorIntakeReversedEnabled:
            self.intakeWheelsMotor.setVelCmd(RPM2RadPerSec(self.intakeWheelsMotorSpd.get()))
        elif self.operatorIntakeEnabled:
            self.intakeWheelsMotor.setVelCmd(RPM2RadPerSec(-self.intakeWheelsMotorSpd.get()))
        else:
            self.intakeWheelsMotor.setVoltage(0)

        # Wrist Motor is faulted, command no movement
        if self.intakeAbsEnc.isFaulted():
            vCmd = 0.0
        elif self.curWristState == intakeWristState.NONE:
            vCmd = 0.0
            self.intakeAbsEnc.update()
        # Control wrist to desired position
        else:
            self.intakeAbsEnc.update()
            self.actualPosDeg = rad2Deg(self._getAngleRad())
            errDeg = self.curPosCmdDeg - self.actualPosDeg

            # If in ground position and being commanded down, give some voltage to stay down
            if self.actualPosDeg <= 2 and self.curWristState == intakeWristState.GROUND:
                vCmd = self.downForceV.get()
            # Otherwise if in deadzone, no command
            elif abs(errDeg) <= self.deadzone.get():
                vCmd = 0
            # Changing position so do stuff
            else:
                # Determine direction
                # if self.curWristState == intakeWristState.GROUND:
                if errDeg < 0:
                    vCmd = -self.kS.get() + self.kPDown.get()*errDeg
                else:
                    # Interpolate kP based on error
                    kPUp = interp(errDeg,self.errUpArr,self.kPUpArr)
                    vCmd = self.kS.get() + kPUp*errDeg #+ self.upHelpV.get()

                # Adding kG term regardless of direction
                vCmd += self.kG.get()*cos(self.actualPosDeg)
                # Saturate voltage
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
    def setIntakeWrist(self,cmdSt: intakeWristState) -> None:
        self.curWristState = cmdSt
        if self.curWristState == intakeWristState.GROUND:
            self.curPosCmdDeg = self.groundPos.get()
        elif self.curWristState == intakeWristState.STOW:
            self.curPosCmdDeg = self.stowPos.get()
        else:
            self.curWristState = intakeWristState.NONE

        if (not self.bWristStatePersist) and (self.curWristState == intakeWristState.NONE):
            self.curPosCmdDeg = self.actualPosDeg

    def setIntakeWristStatePersist(self):
        self.bWristStatePersist = True

    def resetIntakeWristStatePersist(self):
        self.bWristStatePersist = False

    # Disable everything on intake
    def disableIntake(self) -> None:
        self.curWristState = intakeWristState.NONE
        self.operatorIntakeEnabled = False
        self.driverIntakeEnabled = False

    def getIntakeWristState(self) -> intakeWristState:
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

    def _updatekPUp(self):
        self.kPUpArr = [self.kPUp1.get(), self.kPUp2.get(), self.kPUp3.get(), self.kPUp4.get()]
        self.errUpArr = [self.kPUpErr1.get(), self.kPUpErr2.get(), self.kPUpErr3.get(), self.kPUpErr4.get()]

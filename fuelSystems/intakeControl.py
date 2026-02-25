from fuelSystems.fuelSystemConstants import INTAKE_ANGLE_ABS_POS_ENC_OFFSET, IntakeWristState
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
            name = "intake_Motor",
            brakeMode = True,
            currentLimitA = 20.0)
        self.intakeWheelsMotor = WrapperedSparkMax(
            INTAKE_WHEELS_CANID,
            "intake_Wheels_Motor"
            )

        # PID Calibrations
        self.kP = Calibration(
            name="Intake Wrist kP",
            default = 0.6,
            units="V/degErr")
        self.maxV = Calibration(
            name="Intake Wrist maxV",
            default = 6.0,
            units="V")
        self.deadzone = Calibration(
            name="Intake Wrist deadzone",
            default=4.0,
            units="deg")

        # position calibrations
        # an angle in degrees. Assumingt 0 is horizontal, - is down, etc.
        self.groundPos = Calibration(
            name = "Intake Wrist Intake Off Ground Position",
            default = -20,
            units="deg")
        self.stowPos = Calibration(
            name="Intake Wrist Stow Position",
            default = 95,
            units="deg")

        #positions
        self.actualPos = 0
        self.curPosCmdDeg = self.stowPos.get()
        self.curWristState = IntakeWristState.NOTHING

        addLog("Intake Wrist Desired Angle",
               lambda: self.curPosCmdDeg,
               "deg")
        addLog("Intake Wrist Actual Angle",
               lambda: rad2Deg(self._getAngleRad()),
                "deg")

    def update(self):
        # Update intake wheels
        if self.intakeEnabled:
            self.intakeWheelsMotor.setVoltage(8)
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
                self.curWristState == IntakeWristState.NOTHING):
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
    def enableIntakeWheels(self):
        self.intakeEnabled = True

    def disableIntakeWheels(self):
        self.intakeEnabled = False

    def getIntakeWheelsState(self):
        return self.intakeEnabled

    # Helper functions for intake wrist
    def extendIntake(self):
        self._setDesPos(IntakeWristState.GROUND)

    def stowIntake(self):
        self._setDesPos(IntakeWristState.STOW)

    def getIntakeWristState(self):
        return self.curWristState

    # maybe does the same thing as setPosCmd?
    # this is called in teleop periodic or autonomous to set the desired pos of intake wrist
    def _setDesPos(self, desState: IntakeWristState):
        if (desState == IntakeWristState.GROUND):
            self.curPosCmdDeg = self.groundPos.get()
        elif (desState == IntakeWristState.STOW):
            self.curPosCmdDeg = self.groundPos.get()

    def _getAngleRad(self):
        return deg2Rad(self.groundPos.get())

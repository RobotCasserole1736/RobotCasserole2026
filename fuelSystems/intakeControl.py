from fuelSystems.fuelSystemConstants import ALGAE_ANGLE_ABS_POS_ENC_OFFSET, IntakeTrayState
from utils.calibration import Calibration
from utils.signalLogging import addLog
from utils.singleton import Singleton
from utils.constants import INTAKE_CONTROL_CANID, INTAKE_WHEELS_CANID,INTAKE_ENC_PORT
from utils.units import deg2Rad, rad2Deg
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.wrapperedThroughBoreHexEncoder import WrapperedThroughBoreHexEncoder

class IntakeControl(metaclass=Singleton):

    def __init__(self):
          #motor and encoder
        self.intakeMotor = WrapperedSparkMax("intake_Motor",INTAKE_CONTROL_CANID, brakeMode = True, currentLimitA = 20.0)
        self.intakeAbsEnc = WrapperedThroughBoreHexEncoder(port=INTAKE_ENC_PORT, name="Intake_Tray_enc", mountOffsetRad=deg2Rad(ALGAE_ANGLE_ABS_POS_ENC_OFFSET), dirInverted=True)

        #PID stuff calibrations
        self.kP = Calibration(name="Intake Tray kP", default=.6, units="V/degErr")
        self.maxV = Calibration(name="Intake Tray maxV", default=6.0, units="V")
        self.deadzone = Calibration(name="Intake Tray deadzone", default=4.0, units="deg")

        #position calibrations... an angle in degrees. Assumingt 0 is horizontal, - is down, etc.  
        self.intakeOffGroundPos = Calibration(name="Intake Tray Intake Off Ground Position", default = -20, units="deg")
        self.stowPos = Calibration(name="Intake Tray Stow Position", default = 95, units="deg")
       
        #positions
        self.actualPos = 0
        self.curPosCmdDeg = self.stowPos.get()
        self.pos = IntakeTrayState.NOTHING
      

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
            elif self.pos == IntakeTrayState.NOTHING:
                # No command, so keep voltage at zero
                vCmd = 0
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
        self.intakeWheelsMotor.setVoltage(8)

    def lowerIntake(self):
        self.setDesPos(IntakeTrayState.INTAKEOFFGROUND)
        self.intakeLowered = True

    def disableIntakeWheels(self):
        self.intakeEnabled = False
        
    def raiseIntake(self):
        self.setDesPos(IntakeTrayState.STOW)
        self.intakeLowered = False

    def getIntakeLowered(self):
        return self.intakeLowered
    
    def getIntakeState(self):
        return self.intakeEnabled 

    def setDesPos(self, desState : IntakeTrayState): # maybe does the same thing as setPosCmd?
        #this is called in teleop periodic or autonomous to set the desired pos of intake wrist
        self.curPosCmdDeg = self._posToDegrees(desState)

    def getAngleRad(self):
        return deg2Rad(self.intakeOffGroundPos.get())

    # Might optimize to accept 1 enum parameter for new position
    def _posToDegrees(self,pos:IntakeTrayState) -> float:
        self.pos = pos
        if (pos == IntakeTrayState.INTAKEOFFGROUND):
            return self.intakeOffGroundPos.get()
        else:
            return self.stowPos.get()

   
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
          #motor and encoder
        self.intakeMotor = WrapperedSparkMax("intake_Motor",INTAKE_CONTROL_CANID, brakeMode = True, currentLimitA = 20.0)
        self.intakeAbsEnc = WrapperedThroughBoreHexEncoder(port=INTAKE_ENC_PORT, name="Intake_Wrist_enc", mountOffsetRad=deg2Rad(INTAKE_ANGLE_ABS_POS_ENC_OFFSET), dirInverted=True)

        #PID stuff calibrations
        self.kP = Calibration(name="Intake Wrist kP", default=.6, units="V/degErr")
        self.maxV = Calibration(name="Intake Wrist maxV", default=6.0, units="V")
        self.deadzone = Calibration(name="Intake Wrist deadzone", default=4.0, units="deg")

        #position calibrations... an angle in degrees. Assumingt 0 is horizontal, - is down, etc.  
        self.intakeOffGroundPos = Calibration(name="Intake Wrist Intake Off Ground Position", default = -20, units="deg")
        self.stowPos = Calibration(name="Intake Wrist Stow Position", default = 95, units="deg")
       
        #positions
        self.actualPos = 0
        self.curPosCmdDeg = self.stowPos.get()
        self.pos = IntakeWristState.NOTHING
      

        addLog("Intake Wrist Desired Angle",lambda: self.curPosCmdDeg, "deg")
        addLog("Intake Wrist Actual Angle", lambda: rad2Deg(self.getAngleRad()), "deg")

        self.intakeEnabled = False
        self.intakeWheelsMotor = WrapperedSparkMax("intake_Wheels_Motor",INTAKE_WHEELS_CANID)
        self.intakeLowered = False

        

    def update(self):
        if self.intakeEnabled:
            self.intakeWheelsMotor.setVoltage(8) 
            self.intakeAbsEnc.update()
        self.actualPos = rad2Deg(self.getAngleRad())

        if(self.intakeAbsEnc.isFaulted()):
            vCmd = 0.0 # faulted, so stop
        else:
            # Limited-output P control with deadzone
            err = self.curPosCmdDeg - self.actualPos
            if(abs(err) <= self.deadzone.get()):
                # in deadzone, no command
                vCmd = 0
            elif self.pos == IntakeWristState.NOTHING:
                # No command, so keep voltage at zero
                vCmd = 0
            else:
                # Command and outside deadzone
                # P control with limit
                
                # Adjust error so that it's offset by the deadzone
                if(err>0):
                    err = err - self.deadzone.get()
                else:
                    err = err + self.deadzone.get()

                vCmd = self.kP.get() * err
                vCmd = min(self.maxV.get(), max(-self.maxV.get(), vCmd))

        self.intakeMotor.setVoltage(vCmd)

    def enableIntake(self): # spins wheels.
        self.intakeEnabled = True
        self.intakeWheelsMotor.setVoltage(8)

    def lowerIntake(self):
        self.setDesPos(IntakeWristState.INTAKEOFFGROUND)
        self.intakeLowered = True

    def disableIntake(self): #stops wheels
        self.intakeWheelsMotor.setVoltage(0)
        self.intakeEnabled = False
        
    def raiseIntake(self):
        self.setDesPos(IntakeWristState.STOW)
        self.intakeLowered = False

    def getIntakeLowered(self):
        return self.intakeLowered
    
    def getIntakeState(self):
        return self.intakeEnabled 

    def setDesPos(self, desState : IntakeWristState): # maybe does the same thing as setPosCmd?
        #this is called in teleop periodic or autonomous to set the desired pos of intake wrist
        self.curPosCmdDeg = self._posToDegrees(desState)

    def getAngleRad(self):
        return deg2Rad(self.intakeOffGroundPos.get())

    # Might optimize to accept 1 enum parameter for new position
    def _posToDegrees(self,pos:IntakeWristState) -> float:
        self.pos = pos
        if (pos == IntakeWristState.INTAKEOFFGROUND):
            return self.intakeOffGroundPos.get()
        else:
            return self.stowPos.get()

   
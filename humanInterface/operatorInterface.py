from utils.faults import Fault
from utils.signalLogging import addLog
from wpilib import DriverStation, XboxController
from climber.Climber import Climber
from fuelSystems.intakeControl import IntakeControl
from fuelSystems.shooterControl import ShooterController
class OperatorInterface:
    """Class to gather input from the driver of the robot"""

    def __init__(self) -> None:
        # controller
        ctrlIdx = 0
        self.ctrl = XboxController(ctrlIdx)
        self.connectedFault = Fault(f"Operator XBox controller ({ctrlIdx}) unplugged")
        addLog("Xbox A Button", lambda: self.ctrl.getAButton())
        
        # Navigation commands
        self.autoSteerEnable = False

    def update(self) -> None:
        # value of controller buttons

        if self.ctrl.isConnected():
            # Convert from  joystic sign/axis conventions to robot velocity conventions
            self.connectedFault.setNoFault()

            #self.autoSteerToHubProcessor = self.ctrl.getXButton()
            #self.autoSteerDownfield = self.ctrl.getYButton()

            if(self.ctrl.getBackButton()):
                self.autoSteerEnable = False
            elif(self.ctrl.getStartButton()):
                self.autoSteerEnable = True
            else:
                pass

        else:
            # If the joystick is unplugged, pick safe-state commands and raise a fault
            if(DriverStation.isFMSAttached()):
                self.connectedFault.setFaulted()
            self.autoSteerEnable = False
        if self.ctrl.getAButtonPressed():
            self.climber = True

        if self.ctrl.getLeftBumperPressed():
            IntakeControl().enableIntakeWheels() 

       # if self.ctrl.getBButtonPressed():
        #    self.shooterControl.setTargetCmd(True)
        #if self.ctrl.getBButtonReleased():
         #   self.shooterControl.setTargetCmd(False)
     # This is here if we want operator to have shooting instead of driver.
        if self.ctrl.getPOV() == 0:
           self.pitchMotor += 1
        elif self.ctrl.getPOV() == 180:
            self.pitchMotor -= 1
#################################################################################################
## can add more controls if needed.

    def getAutoSteerEnable(self) -> bool:
        return self.autoSteerEnable
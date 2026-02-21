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
        # create a climber instance so we can query its state
        self.climber = Climber()
        self.intakeControl = IntakeControl()
        self.shooterControl = ShooterController()
        

    def update(self) -> None:
        # value of controller buttons

        if self.ctrl.isConnected():
            # Convert from  joystic sign/axis conventions to robot velocity conventions
            self.connectedFault.setNoFault()

        else:
            # If the joystick is unplugged, pick safe-state commands and raise a fault
            if(DriverStation.isFMSAttached()):
                self.connectedFault.setFaulted()
        if self.ctrl.getAButtonPressed():
            self.climber = True #TODO: add button combo to reverse the climber.
        elif self.ctrl.getAButtonPressed() and self.ctrl.getLeftBumperPressed():
            self.climber = False
            
        if self.ctrl.getLeftBumper():
            self.intakeControl.intakeEnabled = True
        if self.ctrl.getRightBumper():
            self.intakeControl.intakeEnabled = False

        if self.ctrl.getBButtonPressed():
            self.shooterControl.setShooting(True)
        if self.ctrl.getBButtonReleased():
            self.shooterControl.setShooting(False)
     # THIS DOES NOT WORK YET. NEEDS TO BE MERGED WITH SHOOTER AND TURRET BRANCH.
        if self.ctrl.getPOV() == 0:
           self.pitchMotor += 1
        elif self.ctrl.getPOV() == 180:
            self.pitchMotor -= 1
#################################################################################################
## can add more controls if needed.
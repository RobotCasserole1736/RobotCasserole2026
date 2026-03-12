from utils.faults import Fault
from utils.signalLogging import addLog
from wpilib import DriverStation, XboxController
from climber.Climber import Climber
from fuelSystems.intakeControl import IntakeControl
from fuelSystems.shooterControl import ShooterController
from fuelSystems.indexerControl import IndexerController
class OperatorInterface:
    """Class to gather input from the driver of the robot"""

    def __init__(self) -> None:
        # controller
        ctrlIdx = 1
        self.ctrl = XboxController(ctrlIdx)
        self.connectedFault = Fault(f"Operator XBox controller ({ctrlIdx}) unplugged")

    def update(self) -> None:
        # value of controller buttons

        if self.ctrl.isConnected():
            # Convert from  joystic sign/axis conventions to robot velocity conventions
            self.connectedFault.setNoFault()
            if self.ctrl.getAButtonPressed():
                self.climber = True

            if self.ctrl.getLeftBumper():
                IntakeControl().operatorEnableIntakeWheels()
            else:
                IntakeControl().operatorDisableIntakeWheels()

            self.shootCmd = self.ctrl.getBButton()
            if self.shootCmd:
                ShooterController().enableShooting()
            else:
                ShooterController().disableShooting()

            self.targetCmd = self.ctrl.getYButton()
            if self.targetCmd:
                ShooterController().enableTargeting()
            else:
                ShooterController().disableTargeting()
            
            self.indexerEjectCmd = self.ctrl.getXButton()
            IndexerController().setIndexerEject(self.indexerEjectCmd)


        else:
            # If the joystick is unplugged, pick safe-state commands and raise a fault
            
            self.shootCmd = False
            self.targetCmd = False
            ShooterController().disableShooting()
            ShooterController().disableTargeting()
            IntakeControl().operatorDisableIntakeWheels()
            if(DriverStation.isFMSAttached()):
                self.connectedFault.setFaulted()
        
#################################################################################################
## can add more controls if needed.

from fuelSystems.fuelSystemConstants import shooterDistance
from utils.faults import Fault
from utils.signalLogging import addLog
from wpilib import DriverStation, XboxController
from fuelSystems.intakeControl import IntakeControl
from fuelSystems.shooterControl import ShooterControl
from fuelSystems.indexerControl import IndexerControl

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

            if self.ctrl.getAButton():
                ShooterControl().enableShooting(shooterDistance.LONG)
            elif self.ctrl.getBButton():
                ShooterControl().enableShooting(shooterDistance.SHORT)
            else:
                ShooterControl().disableShooting()

            # if self.ctrl.getYButton():
            #     ShooterControl().enableTargeting()
            # else:
            #     ShooterControl().disableTargeting()

            IndexerControl().setIndexerEject(self.ctrl.getXButton())

        else:
            # If the joystick is unplugged, pick safe-state commands and raise a fault

            self.shootCmd = False
            self.targetCmd = False
            ShooterControl().disableShooting()
            ShooterControl().disableTargeting()
            IntakeControl().operatorDisableIntakeWheels()
            if(DriverStation.isFMSAttached()):
                self.connectedFault.setFaulted()

#################################################################################################
## can add more controls if needed.

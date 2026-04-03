from fuelSystems.fuelSystemConstants import shooterDistance, intakeWristState
from fuelSystems.indexerControl import IndexerControl
from fuelSystems.intakeControl import IntakeControl
from fuelSystems.shooterControl import ShooterControl
from utils.faults import Fault
from wpilib import DriverStation, XboxController

class OperatorInterface:
    """Class to gather input from the driver of the robot"""

    def __init__(self) -> None:
        # Controller
        ctrlIdx = 1
        self.ctrl = XboxController(ctrlIdx)
        self.connectedFault = Fault(f"Operator XBox controller ({ctrlIdx}) unplugged")

    def update(self) -> None:
        if self.ctrl.isConnected():
            # Convert from  joystic sign/axis conventions to robot velocity conventions
            self.connectedFault.setNoFault()

            # Enable intake
            if self.ctrl.getLeftTriggerAxis() > 0.5:
                IntakeControl().operatorEnableIntakeWheels(True)

            else:
                IntakeControl().operatorEnableIntakeWheels(False)

            IntakeControl().operatorEnableIntakeWheelsReverse(self.ctrl.getXButton())

            # Joystick down = intake in ground position
            # Hold down to keep applying force downward
            if self.ctrl.getLeftY() < -0.50:
                IntakeControl().setIntakeWristState(intakeWristState.GROUND)
            # Joystick up = Stow intake
            elif self.ctrl.getLeftY() > 0.50:
                IntakeControl().setIntakeWristState(intakeWristState.STOW)
            else:
                IntakeControl().setIntakeWristState(intakeWristState.NONE)

            # Set indexer to intake or eject
            IndexerControl().setIndexerIntake(self.ctrl.getAButton())
            IndexerControl().setIndexerEject(self.ctrl.getBButton())

            # Shoot fuel long or short distance
            if self.ctrl.getRightTriggerAxis() > 0.5:
                ShooterControl().enableShooting(shooterDistance.LONG)
            elif self.ctrl.getRightBumper():
                ShooterControl().enableShooting(shooterDistance.SHORT)
            else:
                ShooterControl().disableShooting()

            # if self.ctrl.getYButton():
            #     ShooterControl().enableTargeting()
            # else:
            #     ShooterControl().disableTargeting()

        # If the joystick is unplugged, pick safe-state commands and raise a fault
        else:
            IntakeControl().disableIntake()
            IndexerControl().disableIndexer()
            ShooterControl().disableShooting()
            ShooterControl().disableTargeting()
            if(DriverStation.isFMSAttached()):
                self.connectedFault.setFaulted()
#################################################################################################

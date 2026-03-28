from fuelSystems.fuelSystemConstants import shooterDistance
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

            if self.ctrl.getXButton():
                IntakeControl().operatorIntakeReversed()
            else:
                IntakeControl().operatorIntakeReversedDisabled()

            if self.ctrl.getLeftBumper():
                IntakeControl().operatorEnableIntakeWheels(False)
            elif self.ctrl.getLeftTriggerAxis() > 0.5:
                IntakeControl().operatorEnableIntakeWheels(True)
            else:
                IntakeControl().operatorDisableIntakeWheels()

            # Dpad down = extend intake
            if 135 < self.ctrl.getPOV() < 225:
                IntakeControl().extendIntake()
            # Dpad up = Stow intake
            elif 315 < self.ctrl.getPOV() < 360 or 0 <= self.ctrl.getPOV() < 45:
                IntakeControl().stowIntake()

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

            IndexerControl().setIndexerIntake(self.ctrl.getAButton())
            IndexerControl().setIndexerEject(self.ctrl.getBButton())

        # If the joystick is unplugged, pick safe-state commands and raise a fault
        else:
            self.shootCmd = False
            self.targetCmd = False
            ShooterControl().disableShooting()
            ShooterControl().disableTargeting()
            IntakeControl().operatorDisableIntakeWheels()
            if(DriverStation.isFMSAttached()):
                self.connectedFault.setFaulted()
#################################################################################################
    def getShootCmd(self):
        return self.shootCmd

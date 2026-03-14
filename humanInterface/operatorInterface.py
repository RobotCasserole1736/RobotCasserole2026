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
        addLog("Xbox A Button", lambda: self.ctrl.getAButton())
        
        # Navigation commands
        self.autoSteerEnable = False

    def update(self) -> None:
        if self.ctrl.isConnected():
            # Convert from  joystic sign/axis conventions to robot velocity conventions
            self.connectedFault.setNoFault()

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

            self.shootCmd = False
            self.targetCmd = False
            ShooterControl().disableShooting()
            ShooterControl().disableTargeting()
            IntakeControl().operatorDisableIntakeWheels()
            if(DriverStation.isFMSAttached()):
                self.connectedFault.setFaulted()
            self.autoSteerEnable = False 

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
    
    def getShootCmd(self):
        return self.shootCmd
    
    def getTargetCmd(self):
        return self.targetCmd

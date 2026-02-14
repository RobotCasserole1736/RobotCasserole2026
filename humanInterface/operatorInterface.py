from utils.faults import Fault
from utils.signalLogging import addLog
from wpilib import DriverStation, XboxController
from utils.mapLookup2d import MapLookup2D

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

        else:
            # If the joystick is unplugged, pick safe-state commands and raise a fault
            if(DriverStation.isFMSAttached()):
                self.connectedFault.setFaulted()

    def getPos1(self):
        return self.ctrl.getAButtonPressed()
    def getPos2(self):
        return self.ctrl.getBButtonPressed()
    def getPos3(self):
        return self.ctrl.getYButtonPressed()
    def getPos4(self):
        return self.ctrl.getXButtonPressed()
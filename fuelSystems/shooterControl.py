from wpilib import DigitalInput
from utils.singleton import Singleton
from fuelSystems.fuelSystemConstants import shooterTargetCmd
from utils.signalLogging import addLog
from utils.constants import FUEL_GAME_PIECE_PORT

class ShooterController(metaclass=Singleton):

    def __init__(self):
        
        # motor declerations here
        self.gamepieceSensor = DigitalInput(FUEL_GAME_PIECE_PORT)
        self.currentTargetCommand = shooterTargetCmd.CORNERONE

        pass

    def update(self):

        

        pass

    def setTargetCmd(self, targetCommand):
        pass

    def setShooting(self, shooterCommand):
        pass

    def getTurretAngle(self): #Return the current angle of the turret
        pass 

    def getShooterWheelVelocity(self):
        #Cannonically called "Bill" and "Jerry"
        pass

    def getIdealTrajectoryPitch(self):
        #return information
        pass
    
    def getIdealTrajectoryYaw(self):
        #return information
        pass

    def getIdealTopWheelSpeed(self):
        #return information
        pass
    
    def getIdealBottomWheelSpeed(self):
        #return information
        pass

    def getGamePieceStaged(self):
        return self.gamepieceSensor
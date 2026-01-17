from utils.singleton import Singleton
from fuelSystems.fuelSystemConstants import shooterTargetCmd
from utils.signalLogging import addLog

class ShooterController(metaclass=Singleton):

    def __init__(self):
        
        # motor declerations here

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
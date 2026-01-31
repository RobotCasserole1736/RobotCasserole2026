from utils.singleton import Singleton
import hal
from .driverstation import DriverStation
from wpilib import DriverStation
from wpilib import Timer
class hubLightColor(metaclass=Singleton):

    def __init__(self):
      ourTurn(self)=True
      DriverStation.getAlliance()
      #feeds back blue or red
      data = wpilib.DriverStation.getGameSpecificMessage()
      pass


    def update(self):
     matchTime=Timer.getMatchTime()
     currentTime=matchTime
     
     ally = DriverStation.getAlliance()
     if ally is not None:
       if ally == DriverStation.Alliance.kRed:
       #RED ACTION
        if data:
         match data:
           case "R":
             if currentTime > 15 and < 25::
              ourTurn=True

             else currentTime > 50 and < 75:
             ourTurn=True

             elif currentTime > 100:
             ourTurn=True
             
             elif:
             ourTurn=False
            ...
           case "B":
             if currentTime > 15 and < 50:
              ourTurn=True

             else currentTime > 75 and < 100:
             ourTurn=True

             elif currentTime > 125:
             ourTurn=True

             elif:
             ourTurn=False
       elif ally == DriverStation.Alliance.kBlue:
       # <BLUE ACTION>
        if data:
         match data:
           case "B":
             if currentTime > 15 and < 25::
              ourTurn=True

             else currentTime > 50 and < 75:
             ourTurn=True

             elif currentTime > 100:
             ourTurn=True
             
             elif:
             ourTurn=False
            ...
           case "R":
             if currentTime > 15 and < 50:
              ourTurn=True

             else currentTime > 75 and < 100:
             ourTurn=True

             elif currentTime > 125:
             ourTurn=True

             elif:
             ourTurn=False
     else:
     #<NO COLOR YET ACTION>
      
        pass
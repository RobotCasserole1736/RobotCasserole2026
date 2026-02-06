from utils.singleton import Singleton
from wpilib import DriverStation, Timer

class gameStateTracker(metaclass=Singleton):
  """
  Keeps track of hub active state
  """
  def __init__(self) -> None:
    self.hubActive = True
    self.data = DriverStation.getGameSpecificMessage()

  def update(self) -> None:
    matchTime=Timer.getMatchTime()
    alliance = DriverStation.getAlliance()
    if self.data is None:
      self.data = DriverStation.getGameSpecificMessage()

    # When on BLUE alliance
    if alliance == DriverStation.Alliance.kBlue:
      match self.data:
        # Red alliance hub is INACTIVE first
        case "R":
          if ((matchTime >= 20 and matchTime <= 55) or
              (matchTime >= 80 and matchTime <= 105) or
              (matchTime >= 130)):
            self.hubActive = True
          else:
            self.hubActive = False
        # Blue alliance hub is INACTIVE first
        case "B":
          if ((matchTime >= 20 and matchTime <= 30) or
              (matchTime >= 55 and matchTime <= 80) or
              (matchTime >= 105)):
            self.hubActive = True
          else:
            self.hubActive = False

  # Return hub active boolean
  def getHubActive(self) -> bool:
    return self.hubActive

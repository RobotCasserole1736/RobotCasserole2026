from utils.singleton import Singleton
from wpilib import DriverStation, Timer

class GameStateTracker(metaclass=Singleton):
  """
  Keeps track of hub active state
  """ 
  def __init__(self) -> None:
    self.hubActive = True
    self.data = DriverStation.getGameSpecificMessage()
    self.alliance = DriverStation.getAlliance()

  def update(self) -> None:
    matchTime = Timer.getMatchTime()
    
    # When data is received and alliance is known
    if self.data and self.alliance is not None:
      # Data and alliance color match
      if ((self.alliance == DriverStation.Alliance.kBlue and
          self.data == "B") or
         (self.alliance == DriverStation.Alliance.kRed and
          self.data == "R")):
        # Hub is active during shifts 2 and 4
        if ((matchTime >= 20 and matchTime <= 30) or
            (matchTime >= 55 and matchTime <= 80) or
            (matchTime >= 105)):
          self.hubActive = True
        else:
          self.hubActive = False
      # Data and alliance color do NOT match
      else:
        # Hub is active during shifts 1 and 3
        if ((matchTime >= 20 and matchTime <= 55) or
            (matchTime >= 80 and matchTime <= 105) or
            (matchTime >= 130)):
          self.hubActive = True
        else:
          self.hubActive = False
    # Data/alliance is not received or corrupt; assume hub is active
    else:
      self.hubActive = True
      self.data = DriverStation.getGameSpecificMessage()
      self.alliance = DriverStation.getAlliance()

  # Return hub active boolean
  def getHubActive(self) -> bool:
    return self.hubActive

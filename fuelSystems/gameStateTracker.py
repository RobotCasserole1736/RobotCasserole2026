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

    # When data is received
    if self.data:
      # Data and alliance color match
      if ((alliance == DriverStation.Alliance.kBlue and
          self.data == "B") or
         (alliance == DriverStation.Alliance.kRed and
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
    # Data is not received or corrupt; assume hub is active
    else:
      self.hubActive = True
      self.data = DriverStation.getGameSpecificMessage()

  # Return hub active boolean
  def getHubActive(self) -> bool:
    return self.hubActive

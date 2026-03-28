# from utils.calibration import Calibration
# from utils.constants import HOPPER_CANID
# from utils.units import RPM2RadPerSec
# from utils.singleton import Singleton
# from wrappers.wrapperedSparkMax import WrapperedSparkMax
# class HopperControl(metaclass=Singleton):
#     def __init__(self):
#         self.intakeCommand = False
#         self.ejectCommand = False
#         self.hopperMotor = WrapperedSparkMax(
#             HOPPER_CANID, "HopperMotor", brakeMode=True, currentLimitA=30)
#         self.hopperMotorkP = Calibration(name="Hopper kP", default= 0.0)
#         self.hopperMotorkV = Calibration(name="Hopper kV", default= 0.005)
#         self.motorVelCal = Calibration(name="Hopper Velocity", default=RPM2RadPerSec(115), units="RPM")

#     def update(self) -> None:
#         if self.hopperMotorkP.isChanged() or self.hopperMotorkV.isChanged():
#             self.hopperMotor.setPIDF(
#             kP=self.hopperMotorkP.get(),
#             kI=0,
#             kD=0,
#             kFF=self.hopperMotorkV.get())
#         if self.intakeCommand:
#             self.hopperMotor.setVelCmd(self.motorVelCal.get())
#         elif self.ejectCommand:
#             self.hopperMotor.setVelCmd(-self.motorVelCal.get())
#         else:
#             self.hopperMotor.setVoltage(0)

#     def enableHopperIntake(self) -> None:
#         self.intakeCommand = True

#     def enableHopperEject(self) -> None:
#         self.ejectCommand = True

#     def disableHopper(self) -> None:
#         self.intakeCommand = False
#         self.ejectCommand = False

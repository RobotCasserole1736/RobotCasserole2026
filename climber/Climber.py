# Use the local WrapperedSparkMax defined below in this file to avoid import/type conflicts
import math
from rev import SparkMax, SparkMaxConfig, SparkBaseConfig
from wrappers.wrapperedSparkMax import WrapperedSparkMax
class Climber:
    def Init(self):
        """Runs once when the robot starts."""
        self.arm_motor = WrapperedSparkMax(1, "ArmMotor", brakeMode=True)
        self.pivot_motor = WrapperedSparkMax(2, "PivotMotor", brakeMode=True)
        self.timer = 0
        self.is_moving = False
        self.move_duration = 0
        self.speed = 0
    def setSpeed(self, newSpeed):
        self.speed = newSpeed

    def getSpeed(self):
        return self.speed

    def startClimb(self,distance):
        if self.arm_motor.getMotorPositionRad() < distance:
            self.pivot_motor.setPosCmd(6)
                 #TODO: retract intake.
            self.arm_motor.setVelCmd(6)
            self.arm_motor.getMotorPositionRad()
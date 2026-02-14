from drivetrain.drivetrainControl import DrivetrainControl
from fuelSystems.fuelSystemConstants import shooterTargetCmd
from utils.signalLogging import addLog
from utils.calibration import Calibration
from utils.constants import TURRET_PITCH_CANID,  TURRET_FEED_CANID, MAIN_SHOOTER_CANID, HOOD_SHOOTER_CANID, blueHubLocation, redHubLocation#, #TURRET_YAW_CANID
from utils.units import deg2Rad
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.wrapperedKraken import WrapperedKraken
import math
from fuelSystems.fuelSystemConstants import SHOOTER_MAIN_WHEEL_RADIUS, PITCH_MOTOR_BELT_RATIO, HOOD_MOTOR_BELT_RATIO, MAIN_MOTOR_BELT_RATIO, ROBOT_CYCLE_TIME, GRAVITY, SHOOTER_HOOD_WHEEL_RADIUS, HOOD_ANGLE_OFFSET, SHOOTER_OFFSET, TURRET_MAX_YAW, TURRET_MIN_YAW, SHOOTER_ACTIVATOR_TARGET_PERCENT
from utils.allianceTransformUtils import onRed, transform
from wpilib import Field2d, SmartDashboard
from wpimath import geometry
# import wpilib
from wpilib import Mechanism2d, MechanismObject2d, MechanismLigament2d, Color8Bit
from utils.singleton import Singleton

class ShooterController(metaclass=Singleton):

    def __init__(self):
        #TODO -- ADD A CHECK TO PREVENT US FROM TRYING TO GO PAST OUR MAXIMUM ANGLES. from 5 to 67 degrees.
        self.shooterMainMotorkP = Calibration("shooterMain motor KP", default=0.6, units="Volts/RadPerSec")
        self.shooterMainMotorkI = Calibration("shooterMain motor KI", default=0.15)
        # self.shooterMainMotorkD = Calibration("shooterMain motor KD", default=0)

        self.shooterHoodMotorkP = Calibration("shooterHood motor KP", default=0.1, units="Volts/RadPerSec")
        self.shooterHoodMotorkI = Calibration("shooterHood motor KI", default=0)
        # self.shooterHoodMotorkD = Calibration("shooterHood motor KD", default=0)

        self.pitchMotorkP = Calibration("pitch motor KP", default=0.03)
        self.pitchMotorkI = Calibration("pitch motor KI", default=0)
        self.pitchMotorkS = Calibration("pitch motor KS", default=0.22) #good kS for this specific setup
        # self.pitchMotorkD = Calibration("pitch motor KD", default=0)

        self.yawMotorkP = Calibration("yaw motor KP", default=0)
        self.yawMotorkI = Calibration("yaw motor KI", default=0)
        # self.yawMotorkD = Calibration("yaw motor KD", default=0)

        # 2 krakens for the shooter wheels
        self.shooterMainMotor = WrapperedKraken(MAIN_SHOOTER_CANID, "ShooterMotorMain", brakeMode=False)
        self.shooterMainMotor.setInverted(True)
        self.shooterHoodMotor = WrapperedKraken(HOOD_SHOOTER_CANID, "ShooterMotorHood", brakeMode=False)
        self.shooterHoodMotor.setInverted(True)

        # 2 neo 550s (Controlled by SparkMaxes) for pitch/yaw
        self.pitchMotor = WrapperedSparkMax(TURRET_PITCH_CANID, "TurretMotorPitch", brakeMode=True)
        # self.pitchMotor.setInverted(True)
        # self.yawMotor = WrapperedSparkMax(TURRET_YAW_CANID, "TurretMotorYaw", brakeMode=True)
        self.feedMotor = WrapperedSparkMax(TURRET_FEED_CANID, "TurretMotorFeed", brakeMode=True)

        # Set PID parameters
        self._updateAllPIDs()

        self.toldToShoot = False
        self.toldToTarget = False

        self.currentTargetCommand = shooterTargetCmd.CORNERONE

        # Currently meters? rounded after converting the 7 ft example in elliots equations
        self.hubTrajectoryMaxHeight = 2.25
        self.hubTrajectoryVertexOffset = 0.304 # Also in meters

        # self.robotPosEst = DrivetrainControl().getModulePositions()
        self.curPos = DrivetrainControl().getCurEstPose()

        # Field for sim purposes
        self.simField = Field2d()
        SmartDashboard.putData("DT Pose 2D", self.simField)
        self.simField.getObject("turret")
        self.simField.getObject("blueHub")
        self.simField.getObject("blueHub").setPose(
            geometry.Pose2d(blueHubLocation,geometry.Rotation2d(0)))

        self.robotPos = self.simField.getRobotPose()
        self.simField.getObject("turret").setPose(
            geometry.Pose2d(geometry.Translation2d(
                self.robotPos.X(),
                self.robotPos.Y()),
                geometry.Rotation2d(self.robotPos.rotation().radians())))

        self.neededTurretYaw = 0

        # Create a "Window" in sim to see what the needed pitch (what the hood will be pointing at) is.
        self.hoodMechanismView = Mechanism2d(30, 30, Color8Bit())
        self.hoodRoot = Mechanism2d.getRoot(self.hoodMechanismView, "iAmRoot", 0, 0)
        self.hoodLigament = self.hoodRoot.appendLigament(
            "hoodLigament",10,0,4,Color8Bit(red=255, blue=20, green=20))
        SmartDashboard.putData("Mech2d", self.hoodMechanismView)

        self.neededTurretPitch = 0
        self.neededBallVel = 0

        # Set up logs
        addLog("Desired Pitch Angle",
               lambda: self.neededTurretPitch / PITCH_MOTOR_BELT_RATIO, units="rad")
        addLog("Actual Pitch Angle",
               lambda: self.pitchMotor.getMotorPositionRad() / PITCH_MOTOR_BELT_RATIO, units="rad")

        # Divided by 2*pi because converting to revolutions and
        # dividing by 2 or 4 as well cause of the ratio of the belt things. Logs the desired rot. Vel. of the wheels, **NOT** motors
        addLog("Desired Main Shooter Speed",
                lambda: (60 * (self.neededBallVel / SHOOTER_MAIN_WHEEL_RADIUS)) / (MAIN_MOTOR_BELT_RATIO*2*math.pi))
        addLog("Desired Hood Shooter Speed",
                lambda: (60 * (self.neededBallVel / SHOOTER_HOOD_WHEEL_RADIUS)) / (HOOD_MOTOR_BELT_RATIO*2*math.pi))
        # dividing by 2 pi to get rotations per second, Multiplying by 60 to make it rpm. Logs rot. Vel. of wheels.
        addLog("Actual Main Shooter Speed",
                lambda: 60 * self.shooterMainMotor.getMotorVelocityRadPerSec() / (MAIN_MOTOR_BELT_RATIO*2*math.pi))
        addLog("Actual Hood Shooter Speed",
                lambda: 60 * self.shooterHoodMotor.getMotorVelocityRadPerSec() / (HOOD_MOTOR_BELT_RATIO*2*math.pi))

    def update(self):
        # Update PIDs if calibrations have changed
        if (self.pitchMotorkP.isChanged() or self.pitchMotorkI.isChanged() or
            self.shooterHoodMotorkP.isChanged() or self.shooterHoodMotorkI.isChanged() or
            self.shooterMainMotorkP.isChanged() or self.shooterMainMotorkI.isChanged()):
            self._updateAllPIDs()

        # Right now software is assuming that we will only move the turret when the shoot button is held down
        if self.toldToTarget or self.toldToShoot:
            # Calculate the ideal ball Velocity Magnitude and Direction so it will make it to our target
            # This is "Traejctory Relative," X axis is the line from the base of the robot at the center
            # of the turret to the base of the hub at the center

            self.oldPos = self.curPos
            self.curPos = DrivetrainControl().getCurEstPose()
            if onRed():
                self.curTargetPos = transform(blueHubLocation)
            else:
                # THIS DOESN'T CHECK WHICH ALLIANCE WE ARE I NEED TO IMPLEMENT THAT LATER
                self.curTargetPos = blueHubLocation

            # The distance to max height offset from target ppos:
            self.targetMaxHeightOffsetHub = 1

            # Currently assuming super strong ideal motors that have no gearboxes
            # We all love ideal software land

            """self.robotRelTurretYaw = self.yawMotor.getMotorPositionRad() + self.curPos.rotation().radians()
            self.robotRelTurretPitch = self.pitchMotor.getMotorPositionRad() #I think this should work for
            #finding our current turret orientation relative to the field? """

            # Oh here calculate the turret's position relative to the field (for if the turret isn't in the center of our robot).
            # Right now ignoring this to have simpler starting code.
            # Relative to the feild
            self.turretPosX = self.curPos.translation().X() + \
                math.cos(self.robotPos.rotation().radians()) * SHOOTER_OFFSET
            self.turretPosY = self.curPos.translation().Y() + \
                math.sin(self.robotPos.rotation().radians()) * SHOOTER_OFFSET

            # Get distance to target
            self.targetTurretDiffX = self.curTargetPos.X() - self.turretPosX
            self.targetTurretDiffY = self.curTargetPos.Y() - self.turretPosY

            self.distToTarget = math.sqrt((self.targetTurretDiffX) ** 2 + (self.targetTurretDiffY) ** 2)

            # lookup target height -- later this will be done through the target class which is why it has its own line
            self.targetTrajectoryMaxHeight = self.hubTrajectoryMaxHeight

            # Find distance to that max height:
            self.distToMaxHeight = self.distToTarget - self.targetMaxHeightOffsetHub

            # Use distance to hub to calculate desired Velocity and angle --
            self.desTrajVel = math.sqrt((2*abs(GRAVITY)*self.targetTrajectoryMaxHeight)/(math.sin(GRAVITY)**2))
            # Right now I assume this is radians.
            self.desTrajPitch = math.atan((2*self.targetTrajectoryMaxHeight)/(self.distToMaxHeight))

            # Get robot's Velocity by measuring distance traveled since last cycle and
            # dividing it by time.

            self.robotFieldXVel = (self.curPos.translation().X() - self.oldPos.translation().X()) / ROBOT_CYCLE_TIME
            self.robotFieldYVel = (self.curPos.translation().Y() - self.oldPos.translation().Y()) / ROBOT_CYCLE_TIME

            # Get rotational Velocity of robot? Using this to compensate for
            # tangential Velocity the robot applies to the turret.
            self.robotRotVel = (self.curPos.rotation().radians() - self.oldPos.rotation().radians()) / ROBOT_CYCLE_TIME

            # Calculate the magnitude of the tangential Velocity:
            self.turretTanVel = self.robotRotVel * SHOOTER_OFFSET

            # And figure out the componenets of that rotational Velocity (Robot's axis relative)
            # the tangential Velocity is a right angle to the direction of the robot, hence adding 2 pi.
            # self.turretTanVelY = self.turretTanVel * math.sin(self.curPos.rotation().radians() + math.pi/2)
            # self.turretTanVelX = self.turretTanVel * math.cos(self.curPos.rotation().radians() + math.pi/2)

            # Convert the robot's Velocity to be relative to our trajectory-freindly axis from its own relative one.
            # The angle difference between the field axis and the trajectory one.
            # Also adding in the tangential Velocity components

            if self.targetTurretDiffX < 0:
                self.robotToTrajAxisAngleDiff = math.pi + math.atan(self.targetTurretDiffY / (-self.targetTurretDiffX)) # get rid of ugly solution -- Task for my future self -- Check if this
            elif self.targetTurretDiffX > 0: #If our X is greater than the target's
                self.robotToTrajAxisAngleDiff =  - math.atan(self.targetTurretDiffY / (self.targetTurretDiffX))
            else:
                self.robotToTrajAxisAngleDiff = math.atan(self.targetTurretDiffY / (0.00000001))

            # if (self.robotXVel + self.turretTanVelX) != 0:
            #    self.robotTrajRelVelX = 1 / (math.sin(self.robotToTrajAxisAngleDiff) * (self.robotXVel + self.turretTanVelX)) #code accidentally flips
            #    self.robotTrajRelVelY = 1 / (math.cos(self.robotToTrajAxisAngleDiff) * (self.robotYVel + self.turretTanVelY)) #X and Y axis
            # else:
            #    self.robotTrajRelVelX = 0
            #    self.robotTrajRelVelY = 0

            # All of the components of the vector for the needed ball Velocity to score
            self.neededBallXVel = math.cos(self.desTrajPitch) * self.desTrajVel# - self.robotTrajRelVelX
            self.neededBallZVel = math.sin(self.desTrajPitch) * self.desTrajVel
            # -1 * self.robotTrajRelVelY
            self.neededBallYVel = 0

            # Convert the components of the needed ball Velocity vector to a magnitude, yaw and pitch.
            # Each of these still relative to ideal launch axis.
            self.neededBallVel = math.sqrt(
                (self.neededBallXVel) ** 2 +
                (self.neededBallYVel) ** 2 +
                (self.neededBallZVel) ** 2)
            self.neededBallYaw = math.atan((0) / (self.neededBallXVel))
            self.neededBallPitch = math.atan((self.neededBallZVel) / (self.neededBallXVel))

            # Now we correct the yaw so it is relative to robot's current direction instead of our ideal trajectory axis
            self.neededSimTurretYaw = (self.neededBallYaw - self.robotToTrajAxisAngleDiff) # + self.robotPosEst.getCurEstPose().rotation().radians()
            self.neededTurretYaw = self.neededSimTurretYaw + self.curPos.rotation().radians()
            self.neededTurretPitch = -deg2Rad(self.neededBallPitch * PITCH_MOTOR_BELT_RATIO)
            # self.neededTurretPitch = HOOD_ANGLE_OFFSET - (self.neededTurretPitch * PITCH_MOTOR_BELT_RATIO)

            # So by this point hopefully all we need to do is point turret to self.neededTurretYaw and self.neededTurretPitch
            # And set the rotational Velocity of the motors to self.neededShooterRotVel (After compensating for gear of course)
            # And we'll be golden.

            # Only shoot if we are close enough angle to the hub:
            # if abs(self.yawMotor.getMotorPositionRad() - self.neededTurretYaw) / self.neededTurretPitch <= SHOOTER_ACTIVATOR_TARGET_PERCENT:
            # if abs(self.pitchMotor.getMotorPositionRad() - self.neededTurretPitch) / self.neededTurretPitch <= SHOOTER_ACTIVATOR_TARGET_PERCENT:

            # I need to check if we are turning past our limit.
            """if self.neededTurretYaw < TURRET_MIN_YAW < self.yawMotor.getMotorPositionRad():
                #wrap around
                self.neededTurretYaw += 2 * math.pi
            elif self.yawMotor.getMotorPositionRad() < TURRET_MAX_YAW < self.neededTurretYaw:
                #wrap around
                self.neededTurretYaw -= 2 * math.pi
            """

            # Now all thats left is figure out the rotational Velocity of the wheels and pass those to motors
            # The needed ball Velocity is divided by the radius of those wheels (refer to tangential Velocity equations)
            # and divided by the belt to motor ratio (technically should multiply by .25 or .5 but whatever its the same cause its 1/4 or 1/2.)
            if self.toldToShoot:
                self.shooterMainMotor.setVelCmd(
                    ((self.neededBallVel / SHOOTER_MAIN_WHEEL_RADIUS)) / MAIN_MOTOR_BELT_RATIO)
                self.shooterHoodMotor.setVelCmd(
                    (self.neededBallVel / SHOOTER_HOOD_WHEEL_RADIUS) / HOOD_MOTOR_BELT_RATIO)
                self.feedMotor.setVoltage(9)
            else:
                self.shooterMainMotor.setVoltage(0)
                self.shooterHoodMotor.setVoltage(0)
                self.feedMotor.setVoltage(0)
                self.neededBallVel = 0

            if self.toldToTarget:
                # Again not currently compensating for gearing?
                self.pitchMotor.setPosCmd(self.neededTurretPitch, self.pitchMotorkS.get())
            else:
                self.pitchMotor.setVoltage(0)

            # self.yawMotor.setPosCmd(self.neededTurretYaw)

            # Something to investigate Thursday(01/29) is if I can have a node thingy in sim that rotates with the robot
            # that i controll in here as a sim version of a turret to make sure parts of this works, currently have no
            # way of testing this code and that's bound to go swell.

            # Update sim stuff
            self.hoodLigament.setAngle((self.neededTurretPitch / math.pi) * 180)
            SmartDashboard.putData("Mech2d", self.hoodMechanismView)

            self.robotPos = self.simField.getRobotPose()
            self.simField.getObject("turret").setPose(geometry.Pose2d(geometry.Translation2d(self.turretPosX, self.turretPosY), geometry.Rotation2d(self.neededSimTurretYaw)))
        else:
            self.feedMotor.setVoltage(0)
            self.shooterHoodMotor.setVoltage(0)
            self.shooterMainMotor.setVoltage(0)
            self.pitchMotor.setVoltage(0)
            self.neededBallVel = 0

    def setTargetCmd(self, targetCommand):
        pass

    def enableShooting(self):
        self.toldToShoot = True

    def disableShooting(self):
        self.toldToShoot = False

    def enableTargeting(self):
        self.toldToTarget = True

    def disableTargeting(self):
        self.toldToTarget = False

    def getIdealTrajectoryPitch(self):
        # return information
        # Currently not using these, might use them in the future to clean up the current update() call and
        # make it more readable
        pass

    def getIdealTrajectoryYaw(self):
        #return information
        pass

    def getIdealMainWheelSpeed(self):
        #return information
        pass

    def getIdealHoodWheelSpeed(self):
        #return information
        pass

    def setPitch(self,angle):
        self.neededTurretPitch = -deg2Rad(angle * PITCH_MOTOR_BELT_RATIO)

    def _updateAllPIDs(self):
        self.shooterMainMotor.setPID(
            self.shooterMainMotorkP.get(),
            self.shooterMainMotorkI.get(),
            0.0)
        self.shooterHoodMotor.setPID(
            self.shooterHoodMotorkP.get(),
            self.shooterHoodMotorkI.get(),
            0.0)
        self.pitchMotor.setPID(
            self.pitchMotorkP.get(),
            0.0,
            0.0)
        # self.yawMotor.setPID(
        #     self.yawMotorkP.get(),
        #     o.o,
        #     0.0)

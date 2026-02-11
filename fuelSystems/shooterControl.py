from drivetrain.poseEstimation.drivetrainPoseEstimator import DrivetrainPoseEstimator
from drivetrain.drivetrainControl import DrivetrainControl
from fuelSystems.fuelSystemConstants import shooterTargetCmd
from utils.signalLogging import addLog
from utils.calibration import Calibration, CalibrationWrangler
from utils.constants import TURRET_PITCH_CANID,  TURRET_FEED_CANID, MAIN_SHOOTER_CANID, HOOD_SHOOTER_CANID, blueHubLocation, redHubLocation#, #TURRET_YAW_CANID
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.wrapperedKraken import WrapperedKraken
import math
from fuelSystems.fuelSystemConstants import SHOOTER_MAIN_WHEEL_RADIUS, HOOD_MOTOR_BELT_RATIO, MAIN_MOTOR_BELT_RATIO, ROBOT_CYCLE_TIME, GRAVITY, SHOOTER_HOOD_WHEEL_RADIUS, HOOD_ANGLE_OFFSET, SHOOTER_OFFSET, TURRET_MAX_YAW, TURRET_MIN_YAW, SHOOTER_ACTIVATOR_TARGET_PERCENT
from utils.allianceTransformUtils import onRed, transform
from wpilib import Field2d, SmartDashboard
from wpimath import geometry
# import wpilib
from wpilib import Mechanism2d, MechanismObject2d, MechanismLigament2d, Color8Bit
from utils.singleton import Singleton

class ShooterController(metaclass=Singleton):

    def __init__(self):
        #TODO -- ADD A CHECK TO PREVENT US FROM TRYING TO GO PAST OUR MAXIMUM ANGLES. from 5 to 67 degrees.
        self.shooterMainMotorkP = Calibration("shooterMain motor KP", default=0.4, units="Volts/RadPerSec")
        self.shooterMainMotorkI = Calibration("shooterMain motor KI", default=0)
        # self.shooterMainMotorkD = Calibration("shooterMain motor KD", default=0)

        self.shooterHoodMotorkP = Calibration("shooterHood motor KP", default=0.4, units="Volts/RadPerSec")
        self.shooterHoodMotorkI = Calibration("shooterHood motor KI", default=0)
        # self.shooterHoodMotorkD = Calibration("shooterHood motor KD", default=0)

        self.pitchMotorkP = Calibration("pitch motor KP", default=0.001)
        self.pitchMotorkI = Calibration("pitch motor KI", default=0)
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
        self.pitchMotor.setInverted(True)
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

        # Set up logs
        # addLog("Actual Main velocity shooter actual",
        #        lambda: self.shooterMainMotor.getMotorVelocityRadPerSec() / (2*math.pi))
        # addLog("Hood velocity shooter actual",
        #        lambda: self.shooterHoodMotor.getMotorVelocityRadPerSec() / (2* math.pi))
        # addLog("Desired Main Shooter Speed", lambda: 0)
        # addLog("Desired Hood shooter Speed", lambda: 0)
        # addLog("Desired Pitch Motor Angle", lambda:0, units="rad")
        # addLog("Actual Pitch Motor Angle", lambda: 0, units="rad")

        # divide by 2 pi to get rotations per second, Multiply by 60 to make it rpm,
        addLog("Actual Main Shooter Speed",
                lambda: 60 * self.shooterMainMotor.getMotorVelocityRadPerSec() / (2*math.pi))
        addLog("Actual Hood Shooter Speed",
                lambda: 60 * self.shooterHoodMotor.getMotorVelocityRadPerSec() / (2* math.pi))

        # Divided by 2*pi because converting to revolutions and
        # dividing by 2 or 4 as well cause of the ratio of the belt things.
        addLog("Desired Main Shooter Speed",
                lambda: (60 * (self.neededBallVelo / SHOOTER_MAIN_WHEEL_RADIUS)) / (2*2*math.pi))
        addLog("Desired Hood Shooter Speed",
                lambda: (60 * (self.neededBallVelo / SHOOTER_HOOD_WHEEL_RADIUS)) / (4*2*math.pi))

        # Pitch logs
        addLog("Desired Pitchmotor angle", lambda: HOOD_ANGLE_OFFSET - self.neededTurretPitch, units="rad")
        addLog("Actual Pitchmotor angle", self.pitchMotor.getMotorPositionRad, units="rad")

    def update(self):
        # Update PIDs if calibrations have changed
        if (self.pitchMotorkP.isChanged() or self.pitchMotorkI.isChanged() or
            self.shooterHoodMotorkP.isChanged() or self.shooterHoodMotorkI.isChanged() or
            self.shooterMainMotorkP.isChanged() or self.shooterMainMotorkI.isChanged()):
            self._updateAllPIDs()

        # Right now software is assuming that we will only move the turret when the shoot button is held down
        if self.toldToTarget or self.toldToShoot:
            # Calculate the ideal ball velocity Magnitude and Direction so it will make it to our target
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

            # Oh here calculate the turret's position relative to the feild (for if the turret isn't in the center of our robot).
            # Right now ignoring this to have simpler starting code.
            # Relative to the feild
            self.turretPosX =  self.curPos.translation().X() + math.cos(self.robotPos.rotation().radians()) * SHOOTER_OFFSET
            self.turretPosY =  self.curPos.translation().Y() + math.sin(self.robotPos.rotation().radians()) * SHOOTER_OFFSET

            # Get distance to target
            self.targetTurretDiffX = self.curTargetPos.X() - self.turretPosX
            self.targetTurretDiffY = self.curTargetPos.Y() - self.turretPosY

            self.distToTarget = math.sqrt((self.targetTurretDiffX) ** 2 + (self.targetTurretDiffY) ** 2)

            # lookup target height -- later this will be done through the target class which is why it has its own line
            self.targetTrajectoryMaxHeight = self.hubTrajectoryMaxHeight

            # Find distance to that max height:
            self.distToMaxHeight = self.distToTarget - self.targetMaxHeightOffsetHub

            # Use distance to hub to calculate desired velocity and angle --
            self.desTrajVelo = math.sqrt((2*abs(GRAVITY)*self.targetTrajectoryMaxHeight)/(math.sin(GRAVITY)**2))
            self.desTrajPitch = math.atan((2*self.targetTrajectoryMaxHeight)/(self.distToMaxHeight)) #Right now I assume this is radians.

            # Get robots velocity by measuring distance traveled since last cycle and
            # dividing it by time.

            self.robotFieldXVelo = (self.curPos.translation().X() - self.oldPos.translation().X()) / ROBOT_CYCLE_TIME
            self.robotFieldYVelo = (self.curPos.translation().Y() - self.oldPos.translation().Y()) / ROBOT_CYCLE_TIME

            # Get rotational velocity of robot? Using this to compensate for tangential velocity the robot applies to the turret.
            self.robotRotVelo = (self.curPos.rotation().radians() - self.oldPos.rotation().radians()) / ROBOT_CYCLE_TIME

            # Calculate the magnitude of the tangential velocity:
            self.turretTanVelo = self.robotRotVelo * SHOOTER_OFFSET

            # And figure out the componenets of that rotational velocity (Robot's axis relative)
            # the tangential velocity is a right angle to the direction of the robot, hence adding 2 pi.
            # self.turretTanVeloY = self.turretTanVelo * math.sin(self.curPos.rotation().radians() + math.pi/2)
            # self.turretTanVeloX = self.turretTanVelo * math.cos(self.curPos.rotation().radians() + math.pi/2)

            # Convert the robot's velocity to be relative to our trajectory-freindly axis from its own relative one.
            # The angle difference between the field axis and the trajectory one.
            # Also adding in the tangential velocity components

            if self.targetTurretDiffX < 0:
                self.robotToTrajAxisAngleDiff = math.pi + math.atan(self.targetTurretDiffY / (-self.targetTurretDiffX)) # get rid of ugly solution -- Task for my future self -- Check if this
            elif self.targetTurretDiffX > 0: #If our X is greater than the target's
                self.robotToTrajAxisAngleDiff =  - math.atan(self.targetTurretDiffY / (self.targetTurretDiffX))
            else:
                self.robotToTrajAxisAngleDiff = math.atan(self.targetTurretDiffY / (0.00000001))

            # if (self.robotXVelo + self.turretTanVeloX) != 0:
            #    self.robotTrajRelVeloX = 1 / (math.sin(self.robotToTrajAxisAngleDiff) * (self.robotXVelo + self.turretTanVeloX)) #code accidentally flips
            #    self.robotTrajRelVeloY = 1 / (math.cos(self.robotToTrajAxisAngleDiff) * (self.robotYVelo + self.turretTanVeloY)) #X and Y axis
            # else:
            #    self.robotTrajRelVeloX = 0
            #    self.robotTrajRelVeloY = 0

            #All of the components of the vector for the needed ball velocity to score
            self.neededBallXVelo = math.cos(self.desTrajPitch) * self.desTrajVelo# - self.robotTrajRelVeloX
            self.neededBallZVelo = math.sin(self.desTrajPitch) * self.desTrajVelo
            self.neededBallYVelo = 0#-1 * self.robotTrajRelVeloY

            # Convert the components of the needed ball velocity vector to a magnitude, yaw and pitch.
            # Each of these still relative to ideal launch axis.
            self.neededBallVelo = math.sqrt(
                (self.neededBallXVelo) ** 2 +
                (self.neededBallYVelo) ** 2 +
                (self.neededBallZVelo) ** 2)
            self.neededBallYaw = math.atan((0) / (self.neededBallXVelo))
            self.neededBallPitch = math.atan((self.neededBallZVelo) / (self.neededBallXVelo))

            # Now we correct the yaw so it is relative to robot's current direction instead of our ideal trajectory axis
            self.neededSimTurretYaw = (self.neededBallYaw - self.robotToTrajAxisAngleDiff) # + self.robotPosEst.getCurEstPose().rotation().radians()
            self.neededTurretYaw = self.neededSimTurretYaw + self.curPos.rotation().radians()
            self.neededTurretPitch = self.neededBallPitch

            # So by this point hopefully all we need to do is point turret to self.neededTurretYaw and self.neededTurretPitch
            # And set the rotational velocity of the motors to self.neededShooterRotVelo (After compensating for gear of course)
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

            # Now all thats left is figure out the rotational velocity of the wheels and pass those to motors
            # The needed ball velocity is divided by the radius of those wheels (refer to tangential velocity equations)
            # and divided by the belt to motor ratio (technically should multiply by .25 or .5 but whatever its the same cause its 1/4 or 1/2.)
            if self.toldToShoot:
                # dividing by two and for compensates for motor gearing/belt ratios
                self.shooterMainMotor.setVelCmd(
                    ((self.neededBallVelo / SHOOTER_MAIN_WHEEL_RADIUS)) / MAIN_MOTOR_BELT_RATIO)
                self.shooterHoodMotor.setVelCmd(
                    (self.neededBallVelo / SHOOTER_HOOD_WHEEL_RADIUS) / HOOD_MOTOR_BELT_RATIO) # do proper
                self.feedMotor.setVoltage(9)
            else:
                self.shooterMainMotor.setVelCmd(0)
                self.shooterHoodMotor.setVelCmd(0)
                self.feedMotor.setVoltage(0)

            if self.toldToTarget:
                # Again not currently compensating for gearing?
                self.pitchMotor.setPosCmd(HOOD_ANGLE_OFFSET - (self.neededTurretPitch * 4))
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
            self.shooterHoodMotor.setVelCmd(0)
            self.shooterMainMotor.setVelCmd(0)
            self.pitchMotor.setVoltage(0)

    def setTargetCmd(self, targetCommand):
        pass

    def enableShooting(self):
        self.toldToShoot = True
        pass

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
            self.pitchMotorkI.get(),
            0.0)
        # self.yawMotor.setPID(
        #     self.yawMotorkP.get(),
        #     self.yawMotorkI.get(),
        #     self.yawMotorkD.get())

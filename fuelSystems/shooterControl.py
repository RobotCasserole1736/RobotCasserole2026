from drivetrain.drivetrainControl import DrivetrainControl
from fuelSystems.fuelSystemConstants import (shooterTargetCmd, shooterDistance, VERTEXOFFSETARRAY, YAW_MOTOR_RATIO,POSITIONARRAY, HEIGHTARRAY,
    SHOOTER_HEIGHT, SHOOTER_MAIN_WHEEL_RADIUS, PITCH_MOTOR_BELT_RATIO, HOOD_MOTOR_BELT_RATIO, MAIN_MOTOR_BELT_RATIO,
    ROBOT_CYCLE_TIME, GRAVITY, SHOOTER_HOOD_WHEEL_RADIUS, SHOOTER_OFFSET, TURRET_MAX_YAW, TURRET_MIN_YAW,
    SHOOTER_ACTIVATOR_TARGET_PERCENT, HOOD_ANGLE_OFFSET, PITCH_ENCODER_RATIO, SHOOTERSTATICPITCH)
from math import atan, tan, cos, sin, sqrt, pi
from utils.calibration import Calibration
from utils.constants import (TURRET_PITCH_CANID, PITCH_ENC_PORT, TURRET_FEED_CANID, MAIN_SHOOTER_CANID,
    HOOD_SHOOTER_CANID, blueHubLocation , redHubLocation, TURRET_YAW_CANID)
from utils.signalLogging import addLog
from utils.units import deg2Rad, RPM2RadPerSec
from utils.allianceTransformUtils import onRed, transform
from utils.singleton import Singleton
from wpilib import Field2d, SmartDashboard, Mechanism2d, Color8Bit
from wpimath import geometry
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.wrapperedKraken import WrapperedKraken
from wrappers.wrapperedThroughBoreHexEncoder import WrapperedThroughBoreHexEncoder
from fuelSystems.indexerControl import IndexerControl

class ShooterControl(metaclass=Singleton):

    def __init__(self):
        #TODO -- ADD A CHECK TO PREVENT US FROM TRYING TO GO PAST OUR MAXIMUM ANGLES. from 5 to 67 degrees.
        self.shooterMainMotorkP = Calibration("shooterMain motor KP", default=0.12, units="Volts/RadPerSec")
        self.shooterMainMotorkI = Calibration("shooterMain motor KI", default=0.001)
        self.shooterMainMotorKD = Calibration("shooterMain motor KD", default=0)
        self.shooterMainMotorKS = Calibration("shooterMain motor KS", default=0)
        self.shooterMainMotorKV = Calibration("shooterMain motor KV", default=0.14)
        self.shooterMainMotorKA = Calibration("shooterMain motor KA", default=3.8)
        self.shooterMainShortVelocity = Calibration("shooterMain Short Velocity", default=10, units="RPM")
        self.shooterMainLongVelocity = Calibration("shooterMain Long Velocity", default=40, units="RPM")
        self.shooterMainRadSTolerance = Calibration("shooterMain Shot Velocity Tolerance", default=600, units="Rad/S")
        """
        Motors we currently don't have:
        self.shooterHoodMotorkP = Calibration("shooterHood motor KP", default=0.1, units="Volts/RadPerSec")
        self.shooterHoodMotorkI = Calibration("shooterHood motor KI", default=0.0)

        self.pitchMotorkP = Calibration("pitch motor KP", default=0.03)
        self.pitchMotorkS = Calibration("pitch motor KS", default=0.22)

        self.yawMotorkP = Calibration("yaw motor KP", default=0.15)
        self.yawMotorkS = Calibration("yaw motor KS", default=0.12)

        self.hoodTestVelCmd = Calibration("Hood Wheel Test Command", default=10)
        self.yawTestCmd = Calibration("Yaw Test Command", default=10)
        self.pitchTestCmd = Calibration("Pitch Test Command", default=0)"""

        self.feedMotorVoltage = Calibration("Feeder Motor Voltage", default=3.0, units="Volts")

        # 2 krakens for the shooter wheels
        self.shooterMainMotor = WrapperedKraken(MAIN_SHOOTER_CANID, "ShooterMotorMain", brakeMode=False)
        self.shooterMainMotor.setInverted(True)
        self.shooterMainShotType = shooterDistance.NONE
        #Currently don't have:
        #self.shooterHoodMotor = WrapperedKraken(HOOD_SHOOTER_CANID, "ShooterMotorHood", brakeMode=False)
        #self.shooterHoodMotor.setInverted(True)

        # 2 neo 550s (Controlled by SparkMaxes) for pitch/yaw
        #self.pitchMotor = WrapperedSparkMax(TURRET_PITCH_CANID, "TurretMotorPitch", brakeMode=True)
        #self.yawMotor = WrapperedSparkMax(TURRET_YAW_CANID, "TurretMotorYaw", brakeMode=True)

        self.feedMotor = WrapperedSparkMax(TURRET_FEED_CANID, "TurretMotorFeed", brakeMode=True)

        # Set PID parameters
        self._updateAllPIDs()

        self.toldToShoot = False
        #Don't need targeting command; can't target right now, but not worth effort of removing and re-adding.
        self.toldToTarget = False

        self.setTargetCmd(shooterTargetCmd.HUB)

        #self.pitchAbsEnc = WrapperedThroughBoreHexEncoder(port=PITCH_ENC_PORT, name="Shooter Pitch Enc", mountOffsetRad=deg2Rad(HOOD_ANGLE_OFFSET), dirInverted=True)

        # Currently meters? rounded after converting the 7 ft example in elliots equations
        self.hubTrajectoryVertexOffset = self._getTargetVertexOffset(self.currentTargetCommand)#0.304 # Also in meters

        # self.robotPosEst = DrivetrainControl().getModulePositions()
        self.curPos = DrivetrainControl().getCurEstPose()

        # Field for sim purposes
        """self.simField = Field2d()
        SmartDashboard.putData("DT Pose 2D", self.simField)
        self.simField.getObject("turret")
        self.simField.getObject("blueHub")
        self.simField.getObject("blueHub").setPose(
            geometry.Pose2d(blueHubLocation,geometry.Rotation2d(0)))

        robotPosSim = self.simField.getRobotPose()
        self.simField.getObject("turret").setPose(
            geometry.Pose2d(geometry.Translation2d(
                robotPosSim.X(),
                robotPosSim.Y()),
                geometry.Rotation2d(robotPosSim.rotation().radians())))"""

        # Create a "Window" in sim to see what the needed pitch (what the hood will be pointing at) is.
        '''self.hoodMechanismView = Mechanism2d(30, 30, Color8Bit())
        self.hoodRoot = Mechanism2d.getRoot(self.hoodMechanismView, "iAmRoot", 0, 0)
        self.hoodLigament = self.hoodRoot.appendLigament(
            "hoodLigament",10,0,4,Color8Bit(red=255, blue=20, green=20))
        SmartDashboard.putData("Mech2d", self.hoodMechanismView)'''

        #Declerations for variables used in the addLog() stuff
        self.neededTurretYaw = 0
        self.neededTurretPitch = 0
        self.neededFuelVel = 0

        self.launchVelocity = 1

        addLog("shooterMain Launch Velocity", lambda: self.launchVelocity)

        IndexerControl().setIndexerEject(False)
        IndexerControl().setIndexerIntake(False)

        #self.targetMaxHeightOffsetHub = 1

        # Set up logs
        """addLog("Desired Pitch Angle",
               lambda: self.neededTurretPitch / PITCH_MOTOR_BELT_RATIO, units="rad")
        addLog("Actual Pitch Angle",
               lambda: self.pitchAbsEnc.getAngleRad() / PITCH_ENCODER_RATIO, units="rad")
        addLog("Desired Yaw Angle",
               lambda: self.neededTurretYaw / YAW_MOTOR_RATIO, units="rad")
        addLog("Actual Yaw Angle",
                lambda: self.yawMotor.getMotorPositionRad() / YAW_MOTOR_RATIO, units="rad")"""

        # Divided by 2*pi because converting to revolutions and
        # dividing by 2 or 4 as well cause of the ratio of the belt things. Logs the desired rot. Vel. of the wheels, **NOT** motors
        addLog("Desired Main Shooter Speed",
                lambda: (60 * (self.neededFuelVel / SHOOTER_MAIN_WHEEL_RADIUS)) / (MAIN_MOTOR_BELT_RATIO*2*pi))
        """addLog("Desired Hood Shooter Speed",
                lambda: (60 * (self.neededFuelVel / SHOOTER_HOOD_WHEEL_RADIUS)) / (HOOD_MOTOR_BELT_RATIO*2*pi))"""
        # dividing by 2 pi to get rotations per second, Multiplying by 60 to make it rpm. Logs rot. Vel. of wheels.
        addLog("Actual Main Shooter Speed",
                lambda: 60 * self.shooterMainMotor.getMotorVelocityRadPerSec() / (MAIN_MOTOR_BELT_RATIO*2*pi))
        """addLog("Actual Hood Shooter Speed",
                lambda: 60 * self.shooterHoodMotor.getMotorVelocityRadPerSec() / (HOOD_MOTOR_BELT_RATIO*2*pi))"""

    def update(self):# drivetrainCommand):
        # Update PIDs if calibrations have changed
        """if (self.pitchMotorkP.isChanged() or self.shooterHoodMotorkP.isChanged() or
            self.shooterHoodMotorkI.isChanged() or self.shooterMainMotorkP.isChanged() or
            self.shooterMainMotorkI.isChanged() or self.yawMotorkP.isChanged()):
            self._updateAllPIDs()"""
        if (self.shooterMainMotorkP.isChanged() or self.shooterMainMotorkI.isChanged()
             or self.shooterMainMotorKD.isChanged() or self.shooterMainMotorKS.isChanged()
             or self.shooterMainMotorKV.isChanged() or self.shooterMainMotorKA.isChanged()):
            self._updateAllPIDs()

        #self.pitchAbsEnc.update()

        # Right now software is assuming that we will only move the turret when the shoot button is held down
        #if self.toldToTarget or self.toldToShoot:
        if self.toldToShoot or self.toldToTarget: #Delete this one if we switch back.

            # Calculate the ideal Fuel Velocity Magnitude and Direction so it will make it to our target
            # This is "Traejctory Relative," X axis is the line from the base of the robot at the center
            # of the turret to the base of the hub at the center


            oldPos = self.curPos
            self.curPos = DrivetrainControl().getCurEstPose()

            #rawTurretYaw = self.yawMotor.getMotorPositionRad() / YAW_MOTOR_RATIO
            """robotRelTurretYaw = robotTurretYaw + curPos.rotation().radians()
            robotRelTurretPitch = pitchMotor.getMotorPositionRad() * """

            # Oh here calculate the turret's position relative to the field (for if the turret isn't in the center of our robot).
            # Right now ignoring this to have simpler starting code.
            # Relative to the feild
            turretPosX = self.curPos.translation().X() + \
                cos(self.curPos.rotation().radians()) * SHOOTER_OFFSET
            turretPosY = self.curPos.translation().Y() + \
                sin(self.curPos.rotation().radians()) * SHOOTER_OFFSET

            # Get distance to target, relative to Turret's position, not the robot's position
            self.curTargetPos = self._getTargetPos(self.currentTargetCommand)

            distanceToTargetX = self.curTargetPos.X() - turretPosX
            distanceToTargetY = self.curTargetPos.Y() - turretPosY

            distToTarget = sqrt((distanceToTargetX) ** 2 + (distanceToTargetY) ** 2)

            # get desired maximum height -- later this will be done through the target class which is why it has its own line
            targetTrajectoryMaxHeight = self.targetVertexHeight - SHOOTER_HEIGHT
            '''
            # Find distance until the Fuel will reach that max height:
            distToVertex = distToTarget - self.targetVertexOffset
            '''
            # Use distance to hub to calculate desired Velocity and angle --
            #desTrajVel = sqrt((2*abs(GRAVITY)*targetTrajectoryMaxHeight)/(sin(GRAVITY)**2))
            #We need to make sure that this value is not negative before doing the following value isn't negative before taking the square root.
            desTrajVel = (-GRAVITY*distToTarget**2)/((targetTrajectoryMaxHeight - distToTarget * tan(SHOOTERSTATICPITCH))*(2 * cos(SHOOTERSTATICPITCH) ** 2 ))
            if desTrajVel >= 0:
                self.neededFuelVel = sqrt(desTrajVel)
            else:
                self.neededFuelVel = 0


            # Right now I assume this is radians.

            desTrajPitch = SHOOTERSTATICPITCH #atan((2*targetTrajectoryMaxHeight)/(distToVertex))

            # Get robot's Velocity by measuring distance traveled since last cycle and
            # dividing it by time.

            robotFieldXVel = (self.curPos.translation().X() - oldPos.translation().X()) / ROBOT_CYCLE_TIME
            robotFieldYVel = (self.curPos.translation().Y() - oldPos.translation().Y()) / ROBOT_CYCLE_TIME

            # Get angular Velocity of robot? Using this to compensate for
            # tangential Velocity the robot applies to the turret.
            robotAngularVel = (self.curPos.rotation().radians() - oldPos.rotation().radians()) / ROBOT_CYCLE_TIME

            # Calculate the magnitude of the tangential Velocity:
            turretTanVel = robotAngularVel * SHOOTER_OFFSET

            # And figure out the componenets of that angular Velocity (Robot's axis relative)
            # the tangential Velocity is a right angle to the direction of the robot, hence adding pi/2.
            # turretTanVelY = turretTanVel * sin(curPos.rotation().radians() + pi/2)
            # turretTanVelX = turretTanVel * cos(curPos.rotation().radians() + pi/2)

            # Convert the robot's Velocity to be relative to our trajectory-friendly axis from its own relative one:
            # First we need the angle difference between the field axis and the trajectory one.
            # This also serves as the angle we need to aim to point at the hub
            robotToTrajAxisAngleDiff = self._getFieldToRobAxisDiff(distanceToTargetX, distanceToTargetY)

            #For converting the x and y of the robot relative velocities to the
            # trajectory-freindly axis from its own relative one:
            # Also adding in the tangential Velocity components
            if (robotFieldXVel ) != 0: #If we ever need to comensate for robot's tangential velo again, just add it everywhere where
                #robotFieldXVel is
                robotTrajRelVelX = 1 / (sin(robotToTrajAxisAngleDiff) * (robotFieldXVel)) #code accidentally flips
                robotTrajRelVelY = 1 / (cos(robotToTrajAxisAngleDiff) * (robotFieldYVel)) #X and Y axis
            else:
                robotTrajRelVelX = 0
                robotTrajRelVelY = 0

            # All of the components of the vector for the needed Fuel Velocity to score
            #neededFuelXVel = cos(desTrajPitch) * desTrajVel - robotTrajRelVelX
            #neededFuelZVel = sin(desTrajPitch) * desTrajVel
            #neededFuelYVel = 0 #-1 * robotTrajRelVelY #OK so our current implementation is for if we aren't turning to compensate for robot translation in this axis.

            # Convert the components of the needed Fuel Velocity vector to a magnitude, yaw and pitch.
            # Each of these still relative to ideal launch axis.

            #These aren't needed for the one-wheel only shooter design, but ill keep the other stuff just in case for sim debugging.
            """neededFuelVel = sqrt(
                (neededFuelXVel) ** 2 +
                (neededFuelYVel) ** 2 +
                (neededFuelZVel) ** 2)"""
            #self.neededFuelYaw = atan((0) / (neededFuelXVel))
            #self.neededFuelPitch = atan((neededFuelZVel) / (neededFuelXVel))

            # Now we correct the yaw so it is relative to robot's current direction instead of our ideal trajectory axis
            #neededSimTurretYaw = (self.neededFuelYaw - robotToTrajAxisAngleDiff) # + self.curPos.rotation().radians()'''
            #Actual Calculation ones:
            #self.neededTurretPitch = HOOD_ANGLE_OFFSET - (self.neededFuelPitch * PITCH_MOTOR_BELT_RATIO)
            #self.neededTurretYaw = (neededSimTurretYaw + self.curPos.rotation().radians()) * YAW_MOTOR_RATIO
            #/\ we need "self." here so that the logging works

            # So by this point hopefully all we need to do is point turret to self.neededTurretYaw and self.neededTurretPitch
            # And set the angular Velocity of the motors to self.neededShooterRotVel (After compensating for gear of course)

            # Only shoot if we are pointing towards the hub within a certain margin of error:
            # if abs(self.yawMotor.getMotorPositionRad() - self.neededTurretYaw) / self.neededTurretPitch <= SHOOTER_ACTIVATOR_TARGET_PERCENT:
            # if abs(self.pitchMotor.getMotorPositionRad() - self.neededTurretPitch) / self.neededTurretPitch <= SHOOTER_ACTIVATOR_TARGET_PERCENT:

            # If we need to check if we are turning past our limit.
            """if self.neededTurretYaw < TURRET_MIN_YAW < self.yawMotor.getMotorPositionRad():
                #wrap around
                self.neededTurretYaw += 2 * pi
            elif self.yawMotor.getMotorPositionRad() < TURRET_MAX_YAW < self.neededTurretYaw:
                #wrap around
                self.neededTurretYaw -= 2 * pi
            """

            # Now all thats left is figure out the angular Velocity of the wheels and pass those to motors
            # The needed Fuel Velocity is divided by the radius of those wheels (refer to tangential Velocity equations)
            # and divided by the belt to motor ratio (technically should multiply by .25 or .5 but whatever its the same cause its 1/4 or 1/2.)

            self.launchVelocity = 1

            if self.shooterMainShotType == shooterDistance.SHORT:
               self. launchVelocity = self.shooterMainShortVelocity.get()
            elif self.shooterMainShotType == shooterDistance.LONG:
                self.launchVelocity = self.shooterMainLongVelocity.get()


            if self.toldToShoot:

                #IndexerControl().setIndexerIntake(True)
                self.shooterMainMotor.setVelCmd(((self.launchVelocity / SHOOTER_MAIN_WHEEL_RADIUS)) / MAIN_MOTOR_BELT_RATIO, self.shooterMainMotorKS.get())
                #self.neededFuelVel = self.hoodTestVelCmd.get() #delete this when not testing
                #self.shooterHoodMotor.setVelCmd((self.neededFuelVel / SHOOTER_HOOD_WHEEL_RADIUS) / HOOD_MOTOR_BELT_RATIO)
                self.feedMotor.setVoltage(self.feedMotorVoltage.get())

                #if abs(self.shooterMainMotor.actVel - self.shooterMainMotor.desVel) <= self.shooterMainRadSTolerance.get():
                    #IndexerControl().setIndexerIntake(True)
                #else:
                    #IndexerControl().setIndexerIntake(False)
            else:
                self.shooterMainMotor.setVoltage(0)
                #self.shooterHoodMotor.setVoltage(0)
                self.feedMotor.setVoltage(0)
                self.neededFuelVel = 0
                #IndexerControl().setIndexerIntake(False)

            """if self.toldToTarget:
                # self.pitchMotor.setPosCmd(self.neededTurretPitch, self.pitchMotorkS.get())
                # self.yawMotor.setPosCmd(self.neededTurretYaw, self.yawMotorkS.get())
                self.pitchMotor.setPosCmd(deg2Rad(self.yawTestCmd.get()) * YAW_MOTOR_RATIO)
                self.yawMotor.setPosCmd(-deg2Rad(self.pitchTestCmd.get() * PITCH_MOTOR_BELT_RATIO))
            else:
                self.pitchMotor.setVoltage(0)
                self.yawMotor.setVoltage(0)"""

            # Something to investigate Thursday(01/29) is if I can have a node thingy
            # in sim that rotates with the robot that I control in here as a sim version
            # of a turret to make sure parts of this works, currently have no way of
            # testing this code and that's bound to go swell.

            # Update sim stuff
            """self.hoodLigament.setAngle((self.neededTurretPitch / pi) * 180)
            SmartDashboard.putData("Mech2d", self.hoodMechanismView)
            self.simField.getObject("turret").setPose(geometry.Pose2d(geometry.Translation2d(turretPosX, turretPosY), geometry.Rotation2d(neededSimTurretYaw)))"""
        else:
            #REMEMBER TO UNCOMMENT THESE THEY ARE SUPER IMPORTANTE
            #Kill all motors
            self.feedMotor.setVoltage(0)
            #self.shooterHoodMotor.setVoltage(0)
            self.shooterMainMotor.setVoltage(0)
            #self.pitchMotor.setVoltage(0)
            #self.yawMotor.setVoltage(0)
            self.neededFuelVel = 0
            #IndexerControl().setIndexerIntake(False)

    def _getFieldToRobAxisDiff(self, distToTargetX, distToTargetY):
        if distToTargetX < 0:
            return pi + atan( distToTargetY / (-distToTargetX))
        if distToTargetX > 0: #If our X is greater than the target's
            return - atan( distToTargetY / (distToTargetX))

        #Inelegant solution for if the distToTargetX is 0 to avoid divide by zero errors:
        return atan( distToTargetY / (0.00000001)) #Super small number but not zero so it doesn't crash

    def setTargetCmd(self, targetCommand):
        self.currentTargetCommand = targetCommand
        self.targetVertexHeight = self._getTargetHeight(self.currentTargetCommand)
        self.targetVertexOffset =  self._getTargetVertexOffset(self.currentTargetCommand)
        pass

    def enableShooting(self, launchLocation : shooterDistance):
        self.toldToShoot = True
        self.shooterMainShotType = launchLocation

    def disableShooting(self):
        self.toldToShoot = False

    def enableTargeting(self):
        self.toldToTarget = True

    def disableTargeting(self):
        self.toldToTarget = False

    # def _getPitchRad(self) -> float:
    #     return self.pitchMotor.getMotorPositionRad() / PITCH_MOTOR_BELT_RATIO

    def _updateAllPIDs(self):
        self.shooterMainMotor.setPID(
            self.shooterMainMotorkP.get(),
            self.shooterMainMotorkI.get(),
            self.shooterMainMotorKD.get(),
            self.shooterMainMotorKV.get(),
            self.shooterMainMotorKA.get())
        '''self.shooterHoodMotor.setPID(
            self.shooterHoodMotorkP.get(),
            self.shooterHoodMotorkI.get(),
            0.0)
        self.pitchMotor.setPID(
            self.pitchMotorkP.get(),
            0.0,
            0.0)
        self.yawMotor.setPID(
            self.yawMotorkP.get(),
            0.0,
            0.0)'''

    def _getTargetPos(self, target):
        #It should choose one based on our position
        if onRed():
            return transform(POSITIONARRAY[self.cmdToInt(target)])
        else:
            return POSITIONARRAY[self.cmdToInt(target)]

    def _getTargetHeight(self, target: shooterTargetCmd) -> float:
        # Index array from fuelSystemConstants using the enum as the index
        return HEIGHTARRAY[self.cmdToInt(target)]

    def _getTargetVertexOffset(self, target) -> float:
        return VERTEXOFFSETARRAY[self.cmdToInt(target)]

    def cmdToInt(self,target: shooterTargetCmd) -> int:

        if target != shooterTargetCmd.AUTOTARGET: #If the command is to not auto calculate, don't calculate what cmd we should have
            return target.value

        if onRed() == False and self.curPos.translation().X() <= 3.963924 or onRed() == True and self.curPos.translation().X() >= 16.51305:
            return shooterTargetCmd.HUB.value
        #This means we in mid or other team's zone
        if self.curPos.translation().Y() >= 8.042656:
            return shooterTargetCmd.CORNERONE.value #CHANGE THIS TO TOP CORNER
        else:
            return shooterTargetCmd.CORNERTWO.value #CHANGE THIS TO BOTTOM CORNER

    def _applySpin(self, distanceToHub, hood):
        #Call this in the motor thing and it returns the value for that motor
        #THIS MATH IS MADE UPP AND WONT WORK

        if self.cmdToInt(self.currentTargetCommand) != shooterTargetCmd.HUB.value:
            if hood == True:
                #is hood motor and doesn't need spin
                return (((self.neededFuelVel / SHOOTER_HOOD_WHEEL_RADIUS)) / HOOD_MOTOR_BELT_RATIO)

            #is main motor and doesn't need spin
            return (((self.neededFuelVel / SHOOTER_MAIN_WHEEL_RADIUS)) / MAIN_MOTOR_BELT_RATIO)

        if (hood == True):
            #is hood motor and needs spin
            return (((self.neededFuelVel / SHOOTER_HOOD_WHEEL_RADIUS)) / HOOD_MOTOR_BELT_RATIO) * (distanceToHub ** 2 * (1/10))

        #is main motor and needs spin
        return (((self.neededFuelVel / SHOOTER_MAIN_WHEEL_RADIUS)) / MAIN_MOTOR_BELT_RATIO) / distanceToHub ** 2 * (1/10)
'''    def driveAim(self, drivetrainCommand):

        return self.curTargetPos'''

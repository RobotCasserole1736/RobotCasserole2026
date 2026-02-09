from utils.singleton import Singleton
from fuelSystems.fuelSystemConstants import shooterTargetCmd
from utils.signalLogging import addLog
from utils.calibration import Calibration, CalibrationWrangler
from utils.constants import TURRET_PITCH_CANID,  TURRET_FEED_CANID, MAIN_SHOOTER_CANID, HOOD_SHOOTER_CANID, blueHubLocation, redHubLocation#, #TURRET_YAW_CANID
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.wrapperedKraken import WrapperedKraken
from drivetrain.poseEstimation.drivetrainPoseEstimator import DrivetrainPoseEstimator
from drivetrain.drivetrainControl import DrivetrainControl
import math 
from fuelSystems.fuelSystemConstants import GRAVITY, SHOOTER_HOOD_WHEEL_RADIUS, HOOD_ANGLE_OFFSET, SHOOTER_OFFSET, TURRET_MAX_YAW, TURRET_MIN_YAW, SHOOTER_ACTIVATOR_TARGET_PERCENT
from utils.allianceTransformUtils import onRed, transform
from wpilib import Field2d
from wpimath import geometry 
import wpilib
from wpilib import Mechanism2d, MechanismObject2d, MechanismLigament2d, Color8Bit

class ShooterController(metaclass=Singleton):

    def __init__(self): 
        #TODO -- ADD A CHECK TO PREVENT US FROM TRYING TO GO PAST OUR MAXIMUM ANGLES. from 5 to 67 degrees. 
        
        # motor declerations here
        #2 krakens for the shooter wheels
        self.shooterMainMotor = WrapperedKraken(MAIN_SHOOTER_CANID, "ShooterMotorMain", brakeMode=False)
        self.shooterHoodMotor = WrapperedKraken(HOOD_SHOOTER_CANID, "ShooterMotorHood", brakeMode=False)

        self.shooterMainMotorkP = Calibration("shooterMain motor KP", default=0.2)
        self.shooterMainMotorkI = Calibration("shooterMain motor KI", default=0)
        self.shooterMainMotorkD = Calibration("shooterMain motor KD", default=0)

        self.shooterHoodMotorkP = Calibration("shooterHood motor KP", default=0.2)
        self.shooterHoodMotorkI = Calibration("shooterHood motor KI", default=0)
        self.shooterHoodMotorkD = Calibration("shooterHood motor KD", default=0)

        self.shooterMainMotor.setPID(self.shooterMainMotorkP.get(), self.shooterMainMotorkI.get(), self.shooterMainMotorkD.get())
        self.shooterHoodMotor.setPID(self.shooterHoodMotorkP.get(), self.shooterHoodMotorkI.get(), self.shooterHoodMotorkD.get())

        # 2 neo 550s (Controlled by SparkMaxes) for pitch/yaw
        self.pitchMotor = WrapperedSparkMax(TURRET_PITCH_CANID, "TurretMotorPitch", brakeMode=True)
        #self.yawMotor = WrapperedSparkMax(TURRET_YAW_CANID, "TurretMotorYaw", brakeMode=True)
        self.feedMotor = WrapperedSparkMax(TURRET_FEED_CANID, "TurretMotorFeed", brakeMode=True)

        self.pitchMotorkP = Calibration("pitch motor KP", default=0)
        self.pitchMotorkI = Calibration("pitch motor KI", default=0)
        self.pitchMotorkD = Calibration("pitch motor KD", default=0)

        self.yawMotorkP = Calibration("yaw motor KP", default=0)
        self.yawMotorkI = Calibration("yaw motor KI", default=0)
        self.yawMotorkD = Calibration("yaw motor KD", default=0)

        self.pitchMotor.setPID(self.pitchMotorkP.get(), self.pitchMotorkI.get(), self.pitchMotorkD.get())
        #self.yawMotor.setPID(self.yawMotorkP.get(), self.yawMotorkI.get(), self.yawMotorkD.get())

        self.toldToShoot = False
        self.toldtoTarget = False

        self.currentTargetCommand = shooterTargetCmd.CORNERONE

        self.hubTrajectoryMaxHeight = 2.25 #Currently meters? rounded after converting the 7 ft example in elliots equations
        self.hubTrajectoryVertexOffset = .304 #Also in meters

        # self.driveTrainCtrl = DrivetrainControl()
        # self.robotPosEst = DrivetrainControl().getModulePositions()

        self.curPos = DrivetrainControl().poseEst.getCurEstPose()

        #Field for sim purposes
        self.simField = Field2d()
        wpilib.SmartDashboard.putData("DT Pose 2D", self.simField)
        self.simField.getObject("turret")
        self.simField.getObject("blueHub")
        self.simField.getObject("blueHub").setPose(geometry.Pose2d(blueHubLocation,geometry.Rotation2d(0)))
        self.robotPos = self.simField.getRobotPose()
        self.simField.getObject("turret").setPose(geometry.Pose2d(geometry.Translation2d(self.robotPos.X(), self.robotPos.Y()), geometry.Rotation2d(self.robotPos.rotation().radians())))
        self.neededTurretYaw = 0

        #Create a "Window" in sim to see what the needed pitch (what the hood will be pointing at) is.
        self.hoodMechanismView = Mechanism2d(30, 30, Color8Bit())
        self.hoodRoot = Mechanism2d.getRoot(self.hoodMechanismView, "iAmRoot", 0, 0)
        self.hoodLigament = self.hoodRoot.appendLigament("hoodLigament", 10,0,4,Color8Bit(red=255, blue=20, green=20))
        wpilib.SmartDashboard.putData("Mech2d", self.hoodMechanismView)

        #Set up logs
        addLog("Main velocity shooter actual", lambda: self.shooterMainMotor.getMotorVelocityRadPerSec() / (2*math.pi))
        addLog("Hood velocity shooter actual", lambda: self.shooterHoodMotor.getMotorVelocityRadPerSec() / (2* math.pi))
            
        addLog("Main velocity shooter desired", lambda: 0)
        addLog("Hood velocity shooter desired", lambda: 0)
        
        addLog("Desired Pitchmotor angle", lambda:0 )
        addLog("current measured pitchmotor angle (radians)", lambda: 0)

        pass

    def update(self):

        #Right now software is assuming that we will only move the turret when the shoot button is held down
        if self.toldToTarget or self.toldToShoot: 

            #Calculate the ideal ball velocity Magnitude and Direction so it will make it to our target
            #This is "Traejctory Relative," X axis is the line from the base of the robot at the center of the turret to
            #the base of the hub at the center  

            self.oldPos = self.curPos 
            self.curPos = DrivetrainControl().getCurEstPose()
            if onRed():
                self.curTargetPos = transform(blueHubLocation) 
            else: 
                self.curTargetPos = blueHubLocation #THIS DOESN'T CHECK WHICH ALLIANCE WE ARE I NEED TO IMPLEMENT THAT LATER

            #The distance to max height offset from target ppos:
            self.targetMaxHeightOffsetHub = 1

            #currently assuming super strong ideal motors that have no gearboxes 
            #We all love ideal software land

            """self.robotRelTurretYaw = self.yawMotor.getMotorPositionRad() + self.curPos.rotation().radians()
            self.robotRelTurretPitch = self.pitchMotor.getMotorPositionRad() #I think this should work for
            #finding our current turret orientation relative to the field? """

            #Oh here calculate the turret's position relative to the feild (for if the turret isn't in the center of our robot). 
            #Right now ignoring this to have simpler starting code.
            #Relative to the feild
            self.turretPosX =  self.curPos.translation().X() + math.cos(self.robotPos.rotation().radians()) * SHOOTER_OFFSET
            self.turretPosY =  self.curPos.translation().Y() + math.sin(self.robotPos.rotation().radians()) * SHOOTER_OFFSET

            #Get distance to target
            self.targetTurretDiffX = self.curTargetPos.X() - self.turretPosX
            self.targetTurretDiffY = self.curTargetPos.Y() - self.turretPosY

            self.distToTarget = math.sqrt((self.targetTurretDiffX) ** 2 + (self.targetTurretDiffY) ** 2)

            #lookup target height 
            self.targetTrajectoryMaxHeight = self.hubTrajectoryMaxHeight

            #Find distance to that max height:
            self.distToMaxHeight = self.distToTarget - self.targetMaxHeightOffsetHub

            #Use distance to hub to calculate desired velocity and angle -- 
            self.desTrajVelo = math.sqrt((2*abs(GRAVITY)*self.targetTrajectoryMaxHeight)/(math.sin(GRAVITY)**2))
            self.desTrajPitch = math.atan((2*self.targetTrajectoryMaxHeight)/(self.distToMaxHeight)) #Right now I assume this is radians.

            # Get robots velocity by measuring distance traveled since last cycle and
            # dividing it by time.

            self.robotCycleTime = 0.02

            self.robotFieldXVelo = (self.curPos.translation().X() - self.oldPos.translation().X()) / self.robotCycleTime
            self.robotFieldYVelo = (self.curPos.translation().Y() - self.oldPos.translation().Y()) / self.robotCycleTime

            #Get rotational velocity of robot? Using this to compensate for tangential velocity the robot applies to the turret.
            self.robotRotVelo = (self.curPos.rotation().radians() - self.oldPos.rotation().radians()) / self.robotCycleTime

            #Calculate the magnitude of the tangential velocity: 
            self.turretTanVelo = self.robotRotVelo * SHOOTER_OFFSET

            #And figure out the componenets of that rotational velocity (Robot's axis relative)
            #the tangential velocity is a right angle to the direction of the robot, hence adding 2 pi. 
            #self.turretTanVeloY = self.turretTanVelo * math.sin(self.curPos.rotation().radians() + math.pi/2)
            #self.turretTanVeloX = self.turretTanVelo * math.cos(self.curPos.rotation().radians() + math.pi/2)

            #Convert the robot's velocity to be relative to our trajectory-freindly axis from its own relative one.
            #The angle difference between the field axis and the trajectory one.
            #Also adding in the tangential velocity components 

            if self.targetTurretDiffX < 0:
                self.robotToTrajAxisAngleDiff = math.pi + math.atan(self.targetTurretDiffY / (-self.targetTurretDiffX)) # get rid of ugly solution -- Task for my future self -- Check if this
            elif self.targetTurretDiffX > 0: #If our X is greater than the target's
                self.robotToTrajAxisAngleDiff =  - math.atan(self.targetTurretDiffY / (self.targetTurretDiffX))
            else:
                self.robotToTrajAxisAngleDiff = math.atan(self.targetTurretDiffY / (0.00000001))

            #if (self.robotXVelo + self.turretTanVeloX) != 0:
            #    self.robotTrajRelVeloX = 1 / (math.sin(self.robotToTrajAxisAngleDiff) * (self.robotXVelo + self.turretTanVeloX)) #code accidentally flips
            #    self.robotTrajRelVeloY = 1 / (math.cos(self.robotToTrajAxisAngleDiff) * (self.robotYVelo + self.turretTanVeloY)) #X and Y axis
            #else:
            #    self.robotTrajRelVeloX = 0
            #    self.robotTrajRelVeloY = 0
            
            #All of the components of the vector for the needed ball velocity to score
            self.neededBallXVelo = math.cos(self.desTrajPitch) * self.desTrajVelo# - self.robotTrajRelVeloX
            self.neededBallZVelo = math.sin(self.desTrajPitch) * self.desTrajVelo 
            self.neededBallYVelo = 0#-1 * self.robotTrajRelVeloY

            #Convert the components of the needed ball velocity vector to a magnitude, yaw and pitch. 
            #Each of these still relative to ideal launch axis.
            self.neededBallVelo = math.sqrt((self.neededBallXVelo) ** 2 + (self.neededBallYVelo) ** 2 + (self.neededBallZVelo) ** 2)
            self.neededBallYaw = math.atan((0) / (self.neededBallXVelo)) 
            self.neededBallPitch = math.atan((self.neededBallZVelo) / (self.neededBallXVelo)) 

            #Now we correct the yaw so it is relative to robot's current direction instead of our ideal trajectory axis
            self.neededSimTurretYaw = (self.neededBallYaw - self.robotToTrajAxisAngleDiff) # + self.robotPosEst.getCurEstPose().rotation().radians()
            self.neededTurretYaw = self.neededSimTurretYaw + self.curPos.rotation().radians()
            self.neededTurretPitch = self.neededBallPitch #

            #Now all thats left is figure out the rotational velocity of the wheels:
            self.neededShooterRotVelo = self.neededBallVelo #/ SHOOTER_WHEEL_RADIUS 
            
            #so by this point hopefully all we need to do is point turret to self.neededTurretYaw and self.neededTurretPitch
            #and set the rotational velocity of the motors to self.neededShooterRotVelo (After compensating for gear of course)
            #and we'll be golden.

            #only shoot if we are close enough angle to the hub:
            #if abs(self.yawMotor.getMotorPositionRad() - self.neededTurretYaw) / self.neededTurretPitch <= SHOOTER_ACTIVATOR_TARGET_PERCENT:
            #if abs(self.pitchMotor.getMotorPositionRad() - self.neededTurretPitch) / self.neededTurretPitch <= SHOOTER_ACTIVATOR_TARGET_PERCENT:
            
            #I need to check if we are turning past our limit.
            """if self.neededTurretYaw < TURRET_MIN_YAW < self.yawMotor.getMotorPositionRad():
                #wrap around
                self.neededTurretYaw += 2 * math.pi
            elif self.yawMotor.getMotorPositionRad() < TURRET_MAX_YAW < self.neededTurretYaw:
                #wrap around
                self.neededTurretYaw -= 2 * math.pi
            """         

            if self.toldToShoot:
                self.shooterMainMotor.setVelCmd((self.neededShooterRotVelo / SHOOTER_HOOD_WHEEL_RADIUS)) 
                self.shooterHoodMotor.setVelCmd((self.neededShooterRotVelo / (SHOOTER_HOOD_WHEEL_RADIUS * 2)))
                self.feedMotor.setVoltage(3)
            else:
                self.shooterMainMotor.setVelCmd(0)
                self.shooterHoodMotor.setVelCmd(0)
                self.feedMotor.setVoltage(0)            

            addLog("Main velocity shooter actual", lambda: self.shooterMainMotor.getMotorVelocityRadPerSec() / (2*math.pi))
            addLog("Hood velocity shooter actual", lambda: self.shooterHoodMotor.getMotorVelocityRadPerSec() / (2* math.pi))
            
            addLog("Main velocity shooter desired", lambda: (self.neededShooterRotVelo / SHOOTER_HOOD_WHEEL_RADIUS) / (2*math.pi))
            addLog("Hood velocity shooter desired", lambda: (self.neededShooterRotVelo / (SHOOTER_HOOD_WHEEL_RADIUS * 2)) / (2*math.pi))

            if self.toldtoTarget == True:
                self.pitchMotor.setPosCmd(HOOD_ANGLE_OFFSET-(self.neededTurretPitch * 4)) #Again not currently compensating for gearing?
            else:
                self.pitchMotor.setVelCmd(0)

            #self.yawMotor.setPosCmd(self.neededTurretYaw)

            #Something to investigate Thursday(01/29) is if I can have a node thingy in sim that rotates with the robot
            #that i controll in here as a sim version of a turret to make sure parts of this works, currently have no
            #way of testing this code and that's bound to go swell.

            addLog("Desired Pitchmotor angle", lambda: HOOD_ANGLE_OFFSET - self.neededTurretPitch)
            addLog("current measured pitchmotor angle (radians)", self.pitchMotor.getMotorPositionRad)
        
            self.hoodLigament.setAngle((self.neededTurretPitch / math.pi) * 180)
            wpilib.SmartDashboard.putData("Mech2d", self.hoodMechanismView)

            self.robotPos = self.simField.getRobotPose()
            self.simField.getObject("turret").setPose(geometry.Pose2d(geometry.Translation2d(self.turretPosX, self.turretPosY), geometry.Rotation2d(self.neededSimTurretYaw)))
        else:
            self.feedMotor.setVoltage(0)
            self.shooterHoodMotor.setVelCmd(0)
            self.shooterMainMotor.setVelCmd(0)
            self.pitchMotor.setVelCmd(0)

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
        #return information
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
from AutoSequencerV2.command import Command
from drivetrain.drivetrainCommand import DrivetrainCommand
from fuelSystems.fuelSystemConstants import intakeWristState
from fuelSystems.intakeControl import IntakeControl
from utils.units import rad2Deg
from wpilib import Timer

from drivetrain.drivetrainControl import DrivetrainControl

#this is just a mechanical drive forward command, not using a Choreo path

class DriveBackwardSlowCommand(Command):
    def __init__(self,duration=0.0,speed=0.0):
        self.returnDriveTrainCommand = DrivetrainCommand()
        self.drivetrainControl = DrivetrainControl()
        #set the velocity only in the X direction (aka downfield) and initialize the timer
        #This happens at the beginning, it's an init not an initalize
        # self.returnDriveTrainCommand.velX = -0.25
        self.returnDriveTrainCommand.velX = -1 * speed
        self.returnDriveTrainCommand.velY = 0.0
        self.returnDriveTrainCommand.velT = 0.0
        self.startTime = Timer.getFPGATimestamp()
        self.started = False
        self.dur = duration

    def initialize(self):
        #So, when we initialize (which is right before the command runs), we want to record the time
        self.startTime = Timer.getFPGATimestamp()
        self.started = True

    def hasStarted(self):
        #returns a True or False value based on whether the program has started and is not completed
        return self.started and not self.isDone()

    def execute(self):
        IntakeControl().setIntakeWristState(intakeWristState.GROUND)
        if rad2Deg(IntakeControl()._getAngleRad()) < 45:
            #this returns the total drive train command, a combination of the x, y, and z vectors
            self.drivetrainControl.setManualCmd(self.returnDriveTrainCommand, False)

    def isDone(self):
        #when time we've spent running is greater than or equal to 3, we're done, so it returns true
        return Timer.getFPGATimestamp() - self.startTime >= self.dur

    def end(self,interrupt):
        self.drivetrainControl.setManualCmd(DrivetrainCommand(0,0,0), False)

from utils.singleton import Singleton

class IntakeController(metaclass=Singleton):

    def __init__(self):

        self.intakeEnabled = False

        pass

    def update(self):
        pass 

    def enableIntake(self):
        pass 

    def disableIntake(self):
        pass 

    def getIntakeState(self):
        return self.intakeEnabled 
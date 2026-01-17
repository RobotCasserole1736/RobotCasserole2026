from utils.singleton import Singleton

class IndexerController(metaclass=Singleton):

    def __init__(self):
        
        #Define motors

        self.indexerEnabled = False

        pass

    def update(self):
        
        #If we are told to spin
        #   Spin
        
        pass

    def setIndexerEnabled(self):
        self.indexerEnabled = True
        pass

    def setIndexerDisabled(self):
        self.indexerEnabled = False 
        pass 

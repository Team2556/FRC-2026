from commands2 import Command
from wpilib import Timer, DriverStation

class AutoIntermediate(Command):
    '''Command that just waits for a specified amount of time, used for delaying auto paths'''
    auto_time = 20
    
    def __init__(self, delay_time : float, cancel_if_too_late = False):
        super().__init__()
        self.delay_time = delay_time
        self.cancel_if_too_late = cancel_if_too_late
        
        self.cancelled = False
    
    def initialize(self):
        time_remaining = DriverStation.getMatchTime()
        if self.cancel_if_too_late and time_remaining < (AutoIntermediate.auto_time - self.delay_time):
            self.cancelled = True
    
    def isFinished(self):
        
        if not DriverStation.isAutonomous():
            return True
        
        if self.cancelled: return False
            
        time_remaining = DriverStation.getMatchTime()
        return time_remaining < (AutoIntermediate.auto_time - self.delay_time)
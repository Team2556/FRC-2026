from commands2 import Command
from wpilib import Timer, DriverStation
from constants.path.key_poses import kPath
from subsystems.drivetrain.drivetrain import SwerveDriveTrain

class AutoCheckpoint(Command):
    '''Command that just waits for a specified amount of time, used for delaying auto paths'''
    auto_time = 20
    
    def __init__(self, checkpoint_id : int, cancel_if_too_late = False):
        super().__init__()
        
        match checkpoint_id:
            case 0:
                self.delay_time = kPath.START_DELAY
            case 1:
                self.delay_time = kPath.CHECKPOINT_1
            case 2:
                self.delay_time = kPath.CHECKPOINT_2
        
        self.cancel_if_too_late = cancel_if_too_late
        
        self.cancelled = False
    
    def initialize(self):
        time_remaining = DriverStation.getMatchTime()
        if self.cancel_if_too_late and time_remaining < (AutoCheckpoint.auto_time - self.delay_time):
            self.cancelled = True
    
    def isFinished(self):
        
        if self.delay_time == 0:
            return True
        
        if not DriverStation.isAutonomous():
            return True
        
        if self.cancelled: 
            return False
            
        time_remaining = DriverStation.getMatchTime()
        return time_remaining < (AutoCheckpoint.auto_time - self.delay_time)

class DriveBlank(Command):
    '''Makes the drivetrain drive from blank values so any align_rotation'''
    
    def __init__(self, drivetrain : SwerveDriveTrain):
        self.drivetrain = drivetrain
        
    def execute(self):
        self.drivetrain.drive_blank()
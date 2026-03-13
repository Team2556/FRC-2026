from wpilib import DriverStation, SmartDashboard
from wpilib.interfaces import GenericHID
from commands2 import Subsystem
from util.custom_controller import XboxController

class TimeManager(Subsystem):
    
    intervals = [30, 55, 80, 105, 130]
    
    def __init__(self, controller_1 : XboxController, controller_2 : XboxController):
        self.intervals.append(999)
        self.intervals.insert(0, 0)
        self.controller_1 = controller_1
        self.controller_2 = controller_2
    
    def periodic(self):
        time_remaining = DriverStation.getMatchTime()
        
        SmartDashboard.putNumber("Time/Time Remaining", time_remaining)
        for i in range(len(self.intervals) - 1):
            if self.intervals[i + 1] > time_remaining:
                time_until_next_phase = time_remaining - self.intervals[i]
                SmartDashboard.putNumber("Time/Time Until Next Phase", time_until_next_phase)
                break
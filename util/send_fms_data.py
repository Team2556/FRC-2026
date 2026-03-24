from wpilib import DriverStation

from commands2 import Subsystem

from util.nt_util import NTTable


class SendFMSData(Subsystem):

    intervals = [30, 55, 80, 105, 130]

    def __init__(self):
        self.intervals.append(999)
        self.intervals.insert(0, 0)
        
        # Increasing value that is used for "Hub Active" light in this case
        # Synchronizes with the teleop phases "1, 2, 3, 4" so it's easier to use
        self.current_interval = 0

        self.nt = NTTable("Extra FMS Data")
        self.nt.float("Time Remaining")
        self.nt.float("Time Until Next Phase")
        self.nt.bool("Hub Active")

    def periodic(self):
        time_remaining = DriverStation.getMatchTime()

        self.nt.set("Time Remaining", time_remaining)
        for i in range(len(self.intervals) - 1):
            if self.intervals[i + 1] > time_remaining:
                time_until_next_phase = time_remaining - self.intervals[i]
                self.current_interval = len(self.intervals) - 2 - i
                self.nt.set("Time Until Next Phase", time_until_next_phase)
                break
        
        self.nt.set("Hub Active", self.get_hub_active())
    
    def get_hub_active(self):
        
        auto_winner = DriverStation.getGameSpecificMessage()
        if not auto_winner == "R" and not auto_winner == "B":
            return True
        
        if self.current_interval < 1 or self.current_interval > 4:
            return True
        
        is_hub_active = self.current_interval == 2 or self.current_interval == 4
        
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            is_hub_active = not is_hub_active
        
        if auto_winner == "R":
            is_hub_active = not is_hub_active
        
        return is_hub_active

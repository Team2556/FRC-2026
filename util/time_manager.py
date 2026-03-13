from wpilib import DriverStation
from commands2 import Subsystem

from util.nt_util import NTTable


class TimeManager(Subsystem):

    intervals = [30, 55, 80, 105, 130]

    def __init__(self):
        self.intervals.append(999)
        self.intervals.insert(0, 0)

        self.nt = NTTable("Time")
        self.nt.float("Time Remaining")
        self.nt.float("Time Until Next Phase")

    def periodic(self):
        time_remaining = DriverStation.getMatchTime()

        self.nt.set("Time Remaining", time_remaining)
        for i in range(len(self.intervals) - 1):
            if self.intervals[i + 1] > time_remaining:
                time_until_next_phase = time_remaining - self.intervals[i]
                self.nt.set("Time Until Next Phase", time_until_next_phase)
                break

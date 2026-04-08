from ntcore import NetworkTableInstance
from wpilib import SmartDashboard


class RpiVision:
    def __init__(self):
        self._inst = NetworkTableInstance.getDefault()

        self._state_table = self._inst.getTable("RpiVision")
        # self._ = self._state_table.getIntegerTopic(0).publish()
        
        self._min_hue = self._state_table.getIntegerTopic("Min Hue").publish()
        self._max_hue = self._state_table.getIntegerTopic("Max Hue").publish()
        self._min_sat = self._state_table.getIntegerTopic("Min Sat").publish()
        self._max_sat = self._state_table.getIntegerTopic("Max Sat").publish()
        self._min_val = self._state_table.getIntegerTopic("Min Val").publish()
        self._max_val = self._state_table.getIntegerTopic("Max Val").publish()

        # values = {self._min_hue, self._max_hue, self._min_sat, self._max_sat, self._min_val, self._max_val}

        self._min_hue.set(18)
        self._max_hue.set(45)
        self._min_sat.set(100)
        self._max_sat.set(255)
        self._min_val.set(140)
        self._max_val.set(255)
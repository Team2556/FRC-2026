from util.custom_controller import XboxController
from commands2 import Subsystem

from constants.shooter import kShooterMotor
from constants.drive import kAutoAlign

class TuneShooterSpeed(Subsystem):
    
    deadband = 0.5
    amount_per_second = 150
    
    def __init__(self, controller : XboxController):
        self.controller = controller
    
    def periodic(self):
        controller_value = self.controller.getLeftY()
        if controller_value >= self.deadband:
            kShooterMotor.TUNED_RPM += (self.amount_per_second / 25)
        elif controller_value <= -self.deadband:
            kShooterMotor.TUNED_RPM -= (self.amount_per_second / 25)
    
    def reset(self):
        kShooterMotor.TUNED_RPM = 0

class TuneAlignAngle(Subsystem):
    
    deadband = 0.5
    amount_per_second = 10
    
    def __init__(self, controller : XboxController):
        self.controller = controller
    
    def periodic(self):
        controller_value = self.controller.getRightX() * -1
        if controller_value >= self.deadband:
            kAutoAlign.ALIGN_TUNER_OFFSET += (self.amount_per_second / 25)
        elif controller_value <= -self.deadband:
            kAutoAlign.ALIGN_TUNER_OFFSET -= (self.amount_per_second / 25)
    
    def reset(self):
        kAutoAlign.ALIGN_TUNER_OFFSET = 0
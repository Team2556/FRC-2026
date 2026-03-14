from commands2 import Command
from subsystems.shooter.shooter_hood import ShooterHood
from constants.shooter import kHoodMotor
from wpilib import SmartDashboard

class ResetShooterHood(Command):
    def __init__(self, shooter_hood : ShooterHood):
        
        self.shooter_hood = shooter_hood
        
        self.addRequirements(self.shooter_hood)
        SmartDashboard.putBoolean("Hood Motor Resetting", False)
        
    def initialize(self):
        self.shooter_hood.set_speed(kHoodMotor.RESET_HOME_SPEED)
        SmartDashboard.putBoolean("Hood Motor Resetting", True)
    
    def isFinished(self):
        return self.shooter_hood.is_hard_stopped()
    
    def end(self, interrupted):
        self.shooter_hood.set_speed(0)
        self.shooter_hood.set_position(0)
        SmartDashboard.putBoolean("Hood Motor Resetting", False)
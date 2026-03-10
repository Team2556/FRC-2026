
import commands2
import phoenix6
from phoenix6 import controls, configs, signals
from phoenix6.controls import Follower
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from wpilib import SmartDashboard, DigitalInput
from constants.climb import kClimb 

class ClimbSubsystem(commands2.Subsystem):

    def __init__(self):
        self.climb_motor = TalonFX(kClimb.CAN_ID, "rio")
        self.climb_motor.configurator.apply(kClimb._CONFIG)
        self.climb_motor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)
        self.climb_position_voltage = controls.PositionVoltage(position=0, slot=0)
        self.state : str = "down"
        
        SmartDashboard.putNumber("Climb Position Up", kClimb.POSITION_UP)
        SmartDashboard.putNumber("Climb Position Down", kClimb.POSITION_DOWN)
        
    def raise_climb(self):
        self.climb_motor.set_control(
            self.climb_position_voltage
            .with_position(kClimb.POSITION_UP)
            .with_slot(0)
        )
        
        self.state = "up"
        
    def lower_climb(self):
        self.climb_motor.set_control(
            self.climb_position_voltage
            .with_position(kClimb.POSITION_DOWN)
            .with_slot(0)
        )
        
        self.state = "down"

        
    def periodic(self):
        kClimb.POSITION_UP = SmartDashboard.getNumber("Climb Position Up", kClimb.POSITION_UP)
        kClimb.POSITION_DOWN = SmartDashboard.getNumber("Climb Position Down", kClimb.POSITION_DOWN)
        SmartDashboard.putString("Climb State", self.state)
        

    

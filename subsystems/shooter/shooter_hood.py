from commands2 import Subsystem
from constants.shooter import kHoodMotor
from phoenix6.hardware import TalonFX
from phoenix6.controls import PositionVoltage
from wpilib import SmartDashboard
from util.editable_pid import EditablePID

# Currently not tested yet. Also there is no "shooter hood" command as all the controlling for
# this subsystem will be in the hub align command (eventually)
class ShooterHood(Subsystem):
    def __init__(self):
        self.shooter_motor = TalonFX(kHoodMotor.CAN_ID, "rio")
        self.shooter_motor.configurator.apply(kHoodMotor._CONFIG)
        
        self.position_voltage = PositionVoltage(0)
    
    def increment(self, mult):
        self.shooter_motor.set_control(self.shooter_motor.get() + (kHoodMotor.INCREMENT_AMOUNT * mult))
    
    def set_position(self, position):
        self.shooter_motor.set_control(self.position_voltage.with_position(position))
    
    def reset(self):
        self.set_position(0)
        
    def periodic(self):
        SmartDashboard.putNumber("Hood Angle", self.shooter_motor.get())
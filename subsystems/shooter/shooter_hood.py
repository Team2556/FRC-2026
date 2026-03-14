from commands2 import Subsystem
from constants.shooter import kHoodMotor
from phoenix6.hardware import TalonFX
from phoenix6.controls import PositionVoltage, DutyCycleOut
from phoenix6.signals import NeutralModeValue
from util.editable_pid import EditablePID
from util.nt_util import NTTable

# Currently not tested yet. Also there is no "shooter hood" command as all the controlling for
# this subsystem will be in the hub align command (eventually)
class ShooterHood(Subsystem):
    def __init__(self):
        self.hood_motor = TalonFX(kHoodMotor.CAN_ID, "rio")
        self.hood_motor.configurator.apply(kHoodMotor._CONFIG)
        self.hood_motor.setNeutralMode(NeutralModeValue.BRAKE)
        
        self.position_voltage = PositionVoltage(0)
        self.home_voltage = DutyCycleOut(0)
        
        self.nt = NTTable("Hood")
        self.nt.float("Hood Angle", 0.0)
        self.editable_pid = EditablePID("Hood", self.hood_motor, kHoodMotor._CONFIG)
    
    def set_speed(self, speed):
        self.hood_motor.set_control(self.home_voltage.with_output(speed))
    
    def increment(self, mult):
        self.hood_motor.set_control(self.hood_motor.get() + ((kHoodMotor.INCREMENT_AMOUNT / 20) * mult))
    
    def set_position(self, position):
        self.hood_motor.set_control(self.position_voltage.with_position(position))
    
    def is_hard_stopped(self):
        # AI made this maybe test it
        velocity = self.hood_motor.get_velocity().value
        current = self.hood_motor.get_stator_current().value
    
        return abs(velocity) < 0.1 and current > 40
    
    def reset(self):
        self.set_position(0)
        
    def periodic(self):
        self.nt.set("Hood Angle", self.hood_motor.get())
        self.editable_pid.periodic()
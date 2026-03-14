from commands2 import Subsystem
import phoenix6
from phoenix6.controls import VelocityVoltage
from phoenix6.hardware import TalonFX
from constants.transfer import kSpindexer, kTransfer
from constants.canbus import kCANId
from util.editable_pid import EditablePID
from util.nt_util import NTTable

class TransferSubsystem(Subsystem):
    def __init__(self):
        self.spindex_motor = TalonFX(kCANId.hopper.SPINDEXER, "rio")
        self.up_transfer_motor = TalonFX(kCANId.hopper.TRASNFER, "rio")
        
        self.spindex_motor.configurator.apply(kSpindexer._CONFIG)
        self.up_transfer_motor.configurator.apply(kTransfer._CONFIG)
        
        self.spindex_motor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)
        self.up_transfer_motor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)
        
        self.spindex_velocity_voltage = VelocityVoltage(0)
        self.up_transfer_velocity_voltage = VelocityVoltage(0)
        
        # Network Tables stuff
        self.nt = NTTable("Transfer")
        self.nt.bool("Active", False)
        
        self.spindexer_nt = NTTable("Transfer").get_subtable("Spindexer")
        self.spindexer_nt.float("RPM", 0.0)
        self.spindexer_nt.float("Target RPM", kSpindexer.TARGET_RPM)
        self.spindexer_nt.float("Ideal RPM", 0.0)
        
        self.up_transfer_nt = NTTable("Transfer").get_subtable("Up Transfer")
        self.up_transfer_nt.float("RPM", 0.0)
        self.up_transfer_nt.float("Target RPM", kTransfer.TARGET_RPM)
        self.up_transfer_nt.float("Ideal RPM", 0.0)
        
        self.spindex_editable_pid = EditablePID("Transfer/Spindexer", self.spindex_motor, kSpindexer._CONFIG)
        self.up_transfer_editable_pid = EditablePID("Transfer/Up Transfer", self.up_transfer_motor, kTransfer._CONFIG)
    
    def activate(self):
        self.spindex_motor.set_control(self.spindex_velocity_voltage.with_velocity(kSpindexer.TARGET_RPM / 60))
        self.up_transfer_motor.set_control(self.up_transfer_velocity_voltage.with_velocity(kTransfer.TARGET_RPM / 60))
        self.spindexer_nt.float("Ideal RPM", kSpindexer.TARGET_RPM / 60)
        self.up_transfer_nt.float("Ideal RPM", kTransfer.TARGET_RPM / 60)
        self.nt.set("Active", True)

    def stop(self):
        self.spindex_motor.set_control(self.spindex_velocity_voltage.with_velocity(0))
        self.up_transfer_motor.set_control(self.up_transfer_velocity_voltage.with_velocity(0))
        self.spindexer_nt.float("Ideal RPM", 0)
        self.up_transfer_nt.float("Ideal RPM", 0)
        self.nt.set("Active", False)
    
    def periodic(self):    
        self.spindexer_nt.set("RPM", self.spindex_motor.get_velocity().value)
        kSpindexer.TARGET_RPM = self.spindexer_nt.get("Target RPM")
        
        self.up_transfer_nt.set("RPM", self.up_transfer_motor.get_velocity().value)
        kTransfer.TARGET_RPM = self.up_transfer_nt.get("Target RPM")
        
        self.spindex_editable_pid.periodic()
        self.up_transfer_editable_pid.periodic()

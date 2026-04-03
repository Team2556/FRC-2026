import wpilib
from commands2 import Subsystem
import phoenix6
from phoenix6.controls import VelocityVoltage, NeutralOut
from phoenix6.hardware import TalonFX

from util.config_util import apply_config
from util.editable_pid import EditablePID
from util.nt_util import NTTable

from constants.transfer import kSpindexer, kTransfer
from constants.canbus import kCANId


class TransferSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self.spindex_motor = TalonFX(kCANId.hopper.SPINDEXER, "rio")
        self.up_transfer_motor = TalonFX(kCANId.hopper.TRASNFER, "rio")

        apply_config(self.spindex_motor,      kSpindexer._CONFIG, "Spindexer")
        apply_config(self.up_transfer_motor,  kTransfer._CONFIG,  "Up Transfer")

        self.spindex_motor.setNeutralMode(phoenix6.signals.NeutralModeValue.COAST)
        self.up_transfer_motor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)

        self.spindex_velocity_voltage = VelocityVoltage(0)
        self.up_transfer_velocity_voltage = VelocityVoltage(0)
        self._neutral = NeutralOut()

        # Network Tables
        self.nt = NTTable("Transfer")
        self.nt.bool("Active", False)

        self.spindexer_nt = self.nt.get_subtable("Spindexer")
        self.spindexer_nt.float("RPM", 0.0)
        self.spindexer_nt.float("Target RPM", kSpindexer.TARGET_RPM)
        self.spindexer_nt.float("Ideal RPM", 0.0)

        self.up_transfer_nt = self.nt.get_subtable("Up Transfer")
        self.up_transfer_nt.float("RPM", 0.0)
        self.up_transfer_nt.float("Target RPM", kTransfer.TARGET_RPM)
        self.up_transfer_nt.float("Ideal RPM", 0.0)

        self.spindex_editable_pid = EditablePID("Transfer/Spindexer", self.spindex_motor, kSpindexer._CONFIG)
        self.up_transfer_editable_pid = EditablePID("Transfer/Up Transfer", self.up_transfer_motor, kTransfer._CONFIG)

    def activate(self):
        self.spindex_motor.set_control(
            self.spindex_velocity_voltage.with_velocity(kSpindexer.TARGET_RPM / 60)
        )
        self.up_transfer_motor.set_control(
            self.up_transfer_velocity_voltage.with_velocity(kTransfer.TARGET_RPM / 60)
        )
        self.spindexer_nt.set("Ideal RPM", kSpindexer.TARGET_RPM)
        self.up_transfer_nt.set("Ideal RPM", kTransfer.TARGET_RPM)
        self.nt.set("Active", True)

    def reverse(self):
        self.spindex_motor.set_control(
            self.spindex_velocity_voltage.with_velocity(-kSpindexer.TARGET_RPM / 60)
        )
        self.spindexer_nt.set("Ideal RPM", -kSpindexer.TARGET_RPM)
        self.nt.set("Active", True)

    def spindex_only(self):
        self.spindex_motor.set_control(
            self.spindex_velocity_voltage.with_velocity(kSpindexer.TARGET_RPM / 60)
        )
        self.spindexer_nt.set("Ideal RPM", kSpindexer.TARGET_RPM)
        self.nt.set("Active", True)

    def stop(self):
        self.spindex_motor.set_control(self._neutral)
        self.up_transfer_motor.set_control(self._neutral)
        self.spindexer_nt.set("Ideal RPM", 0.0)
        self.up_transfer_nt.set("Ideal RPM", 0.0)
        self.nt.set("Active", False)

    def periodic(self):
        self.spindex_editable_pid.periodic()
        self.up_transfer_editable_pid.periodic()

        # get_velocity() returns RPS — multiply by 60 to display RPM
        self.spindexer_nt.set("RPM", self.spindex_motor.get_velocity().value * 60)
        self.up_transfer_nt.set("RPM", self.up_transfer_motor.get_velocity().value * 60)

        # Pull tunable targets from NT
        kSpindexer.TARGET_RPM = self.spindexer_nt.get("Target RPM")
        kTransfer.TARGET_RPM = self.up_transfer_nt.get("Target RPM")

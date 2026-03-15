import commands2
import wpilib

import phoenix6
from phoenix6 import controls
from phoenix6.hardware import TalonFX

from util.editable_pid import EditablePID
from util.nt_util import NTTable

from constants.climb import kClimb
from constants.canbus import kCANId


class ClimbSubsystem(commands2.Subsystem):
    def __init__(self):
        self.climb_motor = TalonFX(kCANId.climb.MOTOR, "rio")
        # Skip configuration in simulation/tests to prevent hanging
        if not wpilib.RobotBase.isSimulation():
            self.climb_motor.configurator.apply(kClimb._CONFIG)
            self.climb_motor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)
        self.climb_position_voltage = controls.PositionVoltage(position=0, slot=0)
        self.state: str = "down"

        self.nt = NTTable("Climb")
        self.nt.float("Position Up", kClimb.POSITION_UP)
        self.nt.float("Position Down", kClimb.POSITION_DOWN)
        self.nt.float("Ideal Position", 0.0)
        self.nt.float("Position", 0.0)
        self.nt.string("State", self.state)

        self.editable_pid = EditablePID("Climb", self.climb_motor, kClimb._CONFIG)

    def raise_climb(self):
        self.climb_motor.set_control(
            self.climb_position_voltage.with_position(kClimb.POSITION_UP).with_slot(0)
        )
        self.state = "up"
        self.nt.set("Ideal Position", kClimb.POSITION_UP)

    def lower_climb(self):
        self.climb_motor.set_control(
            self.climb_position_voltage.with_position(kClimb.POSITION_DOWN).with_slot(0)
        )
        self.state = "down"
        self.nt.set("Ideal Position", kClimb.POSITION_DOWN)

    def periodic(self):
        kClimb.POSITION_UP = self.nt.get("Position Up")
        kClimb.POSITION_DOWN = self.nt.get("Position Down")
        self.nt.set("Position", self.climb_motor.get_position().value)
        self.nt.set("State", self.state)
        # EditablePID only applies config when values change and skips simulation
        self.editable_pid.periodic()

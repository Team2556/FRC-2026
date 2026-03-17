import wpilib
from enum import Enum

import commands2

from phoenix6.hardware import TalonFX
from phoenix6.controls import Follower, VelocityVoltage, NeutralOut
from phoenix6 import signals

from util.nt_util import NTTable
from util.editable_pid import EditablePID

from constants.canbus import kCANId
from constants.shooter import kShooterMotor


class ShooterState(Enum):
    IDLE = 0
    """Coasting ~1000 RPM"""
    ENABLED = 1
    """At target RPM (or increasing to)"""


class DualMotorShooter(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        self._bottom_motor = TalonFX(kCANId.shooter.BOTTOM_MOTOR, "rio")
        self._top_motor = TalonFX(kCANId.shooter.TOP_MOTOR, "rio")

        self.cfg = kShooterMotor._CONFIG
        self.cfg.motor_output.neutral_mode = signals.NeutralModeValue.COAST

        if not wpilib.RobotBase.isSimulation():
            self._top_motor.configurator.apply(self.cfg)
            self._bottom_motor.configurator.apply(self.cfg)

        self._bottom_motor.set_control(
            Follower(
                self._top_motor.device_id,
                motor_alignment=signals.spn_enums.MotorAlignmentValue.OPPOSED,
            )
        )

        self.idle_request = VelocityVoltage(
            velocity=kShooterMotor.IDLE_RPM / 60, slot=0
        )
        self.coast_request = NeutralOut()
        self.charge_request = VelocityVoltage(
            velocity=kShooterMotor.TARGET_RPM / 60, slot=1
        )
        self.shoot_request = VelocityVoltage(
            velocity=kShooterMotor.TARGET_RPM / 60, slot=2
        )

        self._state: ShooterState = ShooterState.IDLE
        self.is_charged = False

        self.nt = NTTable("Shooter")
        self.nt.string("State")
        self.nt.bool("Motor Charged")

        self.nt_sub = self.nt.get_subtable("Motor")
        self.nt_sub.float("RPM", 0.0)
        self.nt_sub.float("Target RPM", default=kShooterMotor.TARGET_RPM)
        self.nt_sub.float("Idle RPM", default=kShooterMotor.IDLE_RPM)
        self.nt_sub.float(
            "Reach Target Velocity Error",
            default=kShooterMotor.REACH_TARGET_VELOCITY_ERROR,
        )

        self.editable_PID = EditablePID(
            "Shooter/Motor",
            self._top_motor,
            self.cfg,
            use_slot0=True,
            use_slot1=True,
            use_slot2=True,
        )

    def disable(self) -> None:
        self._state = ShooterState.IDLE
        self.is_charged = False

    def enable(self) -> None:
        self._state = ShooterState.ENABLED
        self.is_charged = False

    def periodic(self):
        self.editable_PID.periodic()

        motor_velocity_rpm = self._top_motor.get_velocity().value * 60
        if self._state == ShooterState.IDLE:
            self._top_motor.set_control(
                self.idle_request.with_velocity(kShooterMotor.IDLE_RPM / 60)
            )

            self.nt.set("State", "IDLE")

        elif self._state == ShooterState.ENABLED and not self.is_charged:
            self._top_motor.set_control(
                self.charge_request.with_velocity(kShooterMotor.TARGET_RPM / 60)
            )
            self.is_charged = (
                abs(motor_velocity_rpm - kShooterMotor.TARGET_RPM)
                < kShooterMotor.REACH_TARGET_VELOCITY_ERROR
            )

            self.nt.set("State", "CHARGING")

        elif self._state == ShooterState.ENABLED and self.is_charged:
            self._top_motor.set_control(
                self.shoot_request.with_velocity(kShooterMotor.TARGET_RPM / 60)
            )

            self.nt.set("State", "CHARGED")

        self.nt.set("Motor Charged", self.is_charged)
        self.nt_sub.set("RPM", motor_velocity_rpm)

        kShooterMotor.IDLE_RPM = self.nt_sub.get("Idle RPM")
        kShooterMotor.TARGET_RPM = self.nt_sub.get("Target RPM")
        kShooterMotor.REACH_TARGET_VELOCITY_ERROR = self.nt_sub.get(
            "Reach Target Velocity Error"
        )

import commands2

from phoenix6.hardware import TalonFX
from phoenix6.controls import Follower, VelocityVoltage, CoastOut
from phoenix6 import signals

from enum import Enum

from constants.canbus import kCANId
from constants.shooter import kShooterMotor


class ShooterState(Enum):
    IDLE = 0
    """Coasting ~1000 RPM"""
    ENABLED = 1
    """At target RPM (or increaing to)"""


class DualMotorShooter(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        self._bottom_motor = TalonFX(kCANId.shooter.BOTTOM_MOTOR, "rio")
        self._top_motor = TalonFX(kCANId.shooter.TOP_MOTOR, "rio")

        self.cfg = kShooterMotor._CONFIG
        self.cfg.motor_output.neutral_mode = signals.NeutralModeValue.COAST

        self._bottom_motor.configurator.apply(self.cfg)
        self._top_motor.configurator.apply(self.cfg)

        self._bottom_motor.set_control(
            Follower(
                self._top_motor.device_id,
                motor_alignment=signals.spn_enums.MotorAlignmentValue.OPPOSED,
            )
        )

        self.idle_request = VelocityVoltage(velocity=kShooterMotor.IDLE_RPM, slot=0)
        self.coast_request = CoastOut()
        self.charge_request = VelocityVoltage(velocity=kShooterMotor.TARGET_RPM, slot=0)
        self.shoot_request = VelocityVoltage(velocity=kShooterMotor.TARGET_RPM, slot=1)

        self._state: ShooterState = ShooterState.IDLE
        self.is_charged = False

    def disable(self) -> None:
        self._state = ShooterState.IDLE
        self.is_charged = False

    def enable(self) -> None:
        self._state = ShooterState.ENABLED
        self.is_charged = False

    def periodic(self):
        motor_velocity = self._top_motor.get_velocity().value
        
        if self._state == ShooterState.IDLE:
            at_idle_RPM = (
                abs(motor_velocity - kShooterMotor.IDLE_RPM)
                < kShooterMotor.REACH_TARGET_VELOCITY_ERROR
            )
            self._top_motor.set_control(
                self.idle_request if at_idle_RPM else self.coast_request
            )

        elif self._state == ShooterState.ENABLED and not self.is_charged:
            self._top_motor.set_control(self.charge_request)

            self.is_charged = (
                abs(motor_velocity - kShooterMotor.TARGET_RPM)
                < kShooterMotor.REACH_TARGET_VELOCITY_ERROR
            )

        elif self._state == ShooterState.ENABLED and self.is_charged:
            self._top_motor.set_control(self.shoot_request)

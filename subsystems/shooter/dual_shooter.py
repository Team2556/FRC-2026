import numpy as np

import wpilib
import commands2
from phoenix6.hardware import TalonFX
from phoenix6.controls import Follower, VelocityVoltage, NeutralOut
from phoenix6 import signals

from util.nt_util import NTTable
from util.editable_pid import EditablePID

from constants.canbus import kCANId
from constants.shooter import kShooterMotor, kShooterData


class DualMotorShooter(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        self._top_motor = TalonFX(kCANId.shooter.TOP_MOTOR, "rio")
        self._bottom_motor = TalonFX(kCANId.shooter.BOTTOM_MOTOR, "rio")

        cfg = kShooterMotor._CONFIG
        cfg.motor_output.neutral_mode = signals.NeutralModeValue.COAST

        if not wpilib.RobotBase.isSimulation():
            self._top_motor.configurator.apply(cfg)
            self._bottom_motor.configurator.apply(cfg)

        self._bottom_motor.set_control(
            Follower(
                self._top_motor.device_id,
                motor_alignment=signals.spn_enums.MotorAlignmentValue.OPPOSED,
            )
        )

        self._coast = NeutralOut()
        self._vel_req = VelocityVoltage(velocity=0, slot=0)

        self._enabled = False
        self._target_rpm = kShooterMotor.TARGET_RPM
        self.is_charged = False

        self.nt = NTTable("Shooter")
        self.nt_sub = self.nt.get_subtable("Motor")
        self.nt_sub.float("RPM", 0.0)
        self.nt_sub.float("Target RPM", default=kShooterMotor.TARGET_RPM)
        self.nt_sub.float(
            "Reach Target Velocity Error",
            default=kShooterMotor.REACH_TARGET_VELOCITY_ERROR,
        )
        self.nt.bool("Motor Charged")

        self.editable_PID = EditablePID(
            "Shooter/Motor", self._top_motor, cfg, use_slot0=True
        )

    def enable(self) -> None:
        self._enabled = True

    def disable(self) -> None:
        self._enabled = False

    def set_target_rpm(self, rpm: float) -> None:
        self._target_rpm = rpm

    def is_at_rpm(self) -> None:
        error_threshold = self.nt_sub.get("Reach Target Velocity Error")
        motor_rpm = self._top_motor.get_velocity().value * 60
        return abs(motor_rpm - self._target_rpm) < error_threshold

    @staticmethod
    def get_rpm_by_distance(distance: float) -> float:
        distances, rpms = zip(*kShooterData.SHOT_RPMS)
        return float(np.interp(distance, distances, rpms))

    def periodic(self):
        self.editable_PID.periodic()

        motor_rpm = self._top_motor.get_velocity().value * 60

        if not self._enabled:
            self._top_motor.set_control(self._coast)
        else:
            self._top_motor.set_control(
                self._vel_req.with_velocity(self._target_rpm / 60)
            )

        self.nt_sub.set("RPM", motor_rpm)
        self.nt_sub.set("Target RPM", self._target_rpm)

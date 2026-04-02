from enum import Enum

import numpy as np

import wpilib
import commands2
from phoenix6.hardware import TalonFX
from phoenix6.controls import Follower, VelocityVoltage, NeutralOut
from phoenix6 import signals
from wpimath.geometry import Pose2d

from util.nt_util import NTTable
from util.editable_pid import EditablePID

from constants.canbus import kCANId
from constants.shooter import kShooterMotor, kShooterData


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

        self.coast_request = NeutralOut()
        # Single slot 0 request used for idle, charging, and shooting.
        # One PID set covers all RPM targets.
        self.velocity_request = VelocityVoltage(velocity=0, slot=0)

        self._state: ShooterState = ShooterState.IDLE
        self.is_charged = False
        self._auto_rpm: float | None = None  # set by distance interpolation; None = use NT

        self.nt = NTTable("Shooter")
        self.nt.string("State")
        self.nt.bool("Motor Charged")

        self.nt_sub = self.nt.get_subtable("Motor")
        self.nt_sub.float("RPM", 0.0)
        self.nt_sub.float("Target RPM", default=kShooterMotor.TARGET_RPM)
        self.nt_sub.float("Current Target RPM", default=kShooterMotor.CURRENT_TARGET_RPM)
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
        )
        
        self.shoot_far = False

    def disable(self) -> None:
        self._state = ShooterState.IDLE
        self.is_charged = False

    def enable(self) -> None:
        self._state = ShooterState.ENABLED
        self.is_charged = False
    
    def toggle_shoot_far(self, shoot_far):
        self.shoot_far = shoot_far

    # ------------------------------------------------------------------
    # Distance-based RPM interpolation
    # ------------------------------------------------------------------

    def add_auto_rpm_measurement(self, robot_pose: Pose2d, target_pose: Pose2d) -> None:
        """Interpolate and store the target RPM for the current shot distance.

        Call every execute() loop while auto-aiming.  Clears automatically
        when ``clear_auto_target()`` is called at command end.
        """
        distance = robot_pose.translation().distance(target_pose.translation())
        self._auto_rpm = self.get_rpm_by_distance(distance)

    def clear_auto_target(self) -> None:
        """Revert to NT-tunable target RPM (call from command end())."""
        self._auto_rpm = None

    @staticmethod
    def get_rpm_by_distance(distance: float) -> float:
        """Return the interpolated shooter RPM for a given distance (meters)."""
        distances, rpms = zip(*kShooterData.SHOT_RPMS)
        return float(np.interp(distance, distances, rpms))

    # ------------------------------------------------------------------

    def periodic(self):
        self.editable_PID.periodic()

        # Pull NT-tunable scalars every loop
        kShooterMotor.IDLE_RPM = self.nt_sub.get("Idle RPM")
        kShooterMotor.REACH_TARGET_VELOCITY_ERROR = self.nt_sub.get("Reach Target Velocity Error")

        # Active target: distance-interpolated auto RPM takes priority over NT
        if self._auto_rpm is not None:
            active_target = self._auto_rpm
        else:
            kShooterMotor.CURRENT_TARGET_RPM = self.nt_sub.get("Target RPM")
            active_target = kShooterMotor.CURRENT_TARGET_RPM

        motor_velocity_rpm = self._top_motor.get_velocity().value * 60

        if self._state == ShooterState.IDLE:
            self._top_motor.set_control(
                self.velocity_request.with_velocity(kShooterMotor.IDLE_RPM / 60)
            )
            self.nt.set("State", "IDLE")

        elif self._state == ShooterState.ENABLED and not self.is_charged:
            target_rps = (active_target + kShooterMotor.TUNED_RPM) / 60
            self._top_motor.set_control(self.velocity_request.with_velocity(target_rps))
            self.is_charged = (
                abs(motor_velocity_rpm - (active_target + kShooterMotor.TUNED_RPM))
                < kShooterMotor.REACH_TARGET_VELOCITY_ERROR
            )
            self.nt.set("State", "CHARGING")

        elif self._state == ShooterState.ENABLED and self.is_charged:
            target_rps = (active_target + kShooterMotor.TUNED_RPM) / 60
            self._top_motor.set_control(self.velocity_request.with_velocity(target_rps))
            self.nt.set("State", "CHARGED")

        self.nt.set("Motor Charged", self.is_charged)
        self.nt_sub.set("RPM", motor_velocity_rpm)
        self.nt_sub.set("Current Target RPM", active_target + kShooterMotor.TUNED_RPM)
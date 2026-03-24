import numpy as np

from enum import Enum

import wpilib

from commands2 import Subsystem

from wpimath.geometry import Pose2d

from phoenix6.hardware import TalonFXS
from phoenix6.controls import MotionMagicVoltage, DutyCycleOut
from phoenix6.signals import NeutralModeValue, ReverseLimitValue

from util.editable_pid import EditablePID
from util.nt_util import NTTable
from util.math_helpers import distanceFromPose2dtoPose2d, clamp

from subsystems.drivetrain.driveio import CustomSwerve

from constants.shooter import kHoodMotor, kShooterData
from constants.canbus import kCANId


class HoodStates(Enum):
    HIDE = 0
    AUTO = 1
    MAX = 2
    OUTER_RING = 3
    INNER_RING = 4
    RESETTING = 5
    NONE = 6


class ShooterHood(Subsystem):
    def __init__(self):
        super().__init__()
        self._state: HoodStates = HoodStates.HIDE
        self._motor = TalonFXS(kCANId.shooter.HOOD_CONTROL, "rio")

        if not wpilib.RobotBase.isSimulation():
            self._motor.configurator.apply(kHoodMotor._CONFIG)
            self._motor.setNeutralMode(NeutralModeValue.BRAKE)

        self.position_request = MotionMagicVoltage(position=0, enable_foc=False)
        self.home_request = DutyCycleOut(0, enable_foc=False)

        self.hard_stopped = False
        self.auto_hood_angle = 5.0

        self.editable_pid = EditablePID("Shooter/Hood", self._motor, kHoodMotor._CONFIG)

        self.nt = NTTable("Shooter").get_subtable("Hood")
        self.nt.float("Hood Position (deg)", 0.0)
        self.nt.float("Target Angle (deg)", 0.0)
        self.nt.string("State", "HIDE")

        self._target_position_revs = 0

    def set_state(self, state: HoodStates) -> None:
        self._state = state

    def set_speed(self, speed: float) -> None:
        self._motor.set_control(self.home_request.with_output(speed))

    def set_position(self, position_revs: float) -> None:
        min_revs = kHoodMotor.to_revs(kHoodMotor.HOME_ANGLE_DEG)
        max_revs = kHoodMotor.to_revs(kHoodMotor.MAX_ANGLE_DEG)

        self._target_position_revs = clamp(position_revs, min_revs, max_revs)
        self._motor.set_control(
            self.position_request.with_position(self._target_position_revs)
        )

    def is_at_angle(self) -> bool:
        current_deg = kHoodMotor.to_deg(self._motor.get_position().value)
        target_deg = kHoodMotor.to_deg(self._target_position_revs)

        return abs(current_deg - target_deg) <= kHoodMotor.REACH_TARGET_ANGLE_ERROR

    def is_hard_stopped(self) -> bool:
        return (
            self._motor.get_reverse_limit().value is ReverseLimitValue.CLOSED_TO_GROUND
        )

    def add_auto_hood_measurement(
        self, drivestate: CustomSwerve.DriveState, target_pose: Pose2d
    ) -> None:
        robot_pose = drivestate.pose
        distance = distanceFromPose2dtoPose2d(robot_pose, target_pose)

        interpolated_angle = self.get_angle_by_distance(distance=distance)
        self.auto_hood_angle = clamp(
            interpolated_angle, kHoodMotor.HOME_ANGLE_DEG, kHoodMotor.MAX_ANGLE_DEG
        )

    def _apply_angle(self) -> None:
        target_angle = kHoodMotor.HOME_ANGLE_DEG

        match self._state:
            case HoodStates.AUTO:
                target_angle = self.auto_hood_angle
            case HoodStates.MAX:
                target_angle = kHoodMotor.MAX_ANGLE_DEG
            case HoodStates.INNER_RING:
                target_angle = kHoodMotor.INNER_RING_ANGLE
            case HoodStates.OUTER_RING:
                target_angle = kHoodMotor.OUTER_RING_ANGLE
            case HoodStates.HIDE:
                target_angle = kHoodMotor.HOME_ANGLE_DEG

        target_revs = kHoodMotor.to_revs(target_angle + kHoodMotor.TUNER_OFFSET)
        self.set_position(target_revs)

    def periodic(self) -> None:
        if self.is_hard_stopped() and not self.hard_stopped:
            self._motor.set_position(kHoodMotor.to_revs(kHoodMotor.HOME_ANGLE_DEG))
        self.hard_stopped = self.is_hard_stopped()

        if self._state != HoodStates.RESETTING and self._state != HoodStates.NONE:
            self._apply_angle()

        self.nt.set(
            "Hood Position (deg)",
            round(kHoodMotor.to_deg(self._motor.get_position().value), 1),
        )
        self.nt.set("Target Angle (deg)", kHoodMotor.to_deg(self._target_position_revs))
        self.nt.set("State", self._state.name)

        self.editable_pid.periodic()

    @staticmethod
    def get_angle_by_distance(distance: float) -> float:
        interpolation_distance_data, interpolation_angle_data = zip(
            *kShooterData.SHOT_ANGLES
        )
        return np.interp(
            distance, interpolation_distance_data, interpolation_angle_data
        )

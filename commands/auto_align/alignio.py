import math

import commands2

import numpy as np

from wpimath.geometry import Pose2d
from wpimath.controller import PIDController
from wpimath.units import degrees, meters

from constants.drive import kAutoAlign
from constants.shooter import kShooterConfig, kShooterData

from util.flip_util import FlipUtil
from util import math_helpers

from subsystems.shooter.dual_shooter import DualMotorShooter
from subsystems.drivetrain.drivetrain import SwerveDriveTrain


class TurretTargetBase(commands2.Command):
    def __init__(self, drivetrain: SwerveDriveTrain, target: Pose2d):
        super().__init__()
        self._drivetrain = drivetrain
        self.target_pose_blue = target
        self.target = FlipUtil.fieldPose(self.target_pose_blue)

        self.shooter_offset = kShooterConfig.SHOOTER_OFFSET
        self.shooter_direction = kShooterConfig.SHOOTER_DIRECTION

        self.rotation_PID = PIDController(
            kAutoAlign.ROTATION_PID.p,
            kAutoAlign.ROTATION_PID.i,
            kAutoAlign.ROTATION_PID.d,
        )
        self.rotation_PID.enableContinuousInput(-180.0, 180.0)

    def initialize(self):
        self.target = FlipUtil.fieldPose(self.target_pose_blue)

    def get_target_yaw(self, robot_pose: Pose2d, target_pose: Pose2d) -> degrees:
        shooter_field_pos = robot_pose.translation() + self.shooter_offset.rotateBy(
            robot_pose.rotation()
        )
        vector_pointer = target_pose.translation() - shooter_field_pos
        target_robot_yaw = math.degrees(
            math.atan2(vector_pointer.Y(), vector_pointer.X())
        )
        return target_robot_yaw - self.shooter_direction

    def with_target(self, target):
        self.target = target
        return self


class TurretTargetWithVelocity(TurretTargetBase):
    def __init__(
        self, drivetrain: SwerveDriveTrain, shooter: DualMotorShooter, target: Pose2d
    ):
        super().__init__(drivetrain, target)
        self._shooter = shooter
        self.flight_time_scalar = kAutoAlign.FLIGHT_TIME_SCALAR

    def calculate_rotation(self) -> float:
        drive_state = self._drivetrain.get_state()

        hub_translation = self.target.translation()
        distance_to_hub = hub_translation.distance(drive_state.pose.translation())

        ball_flight_time = (
            self.estimate_flight_time(distance_to_hub) * self.flight_time_scalar
        )

        lead_ball_offset = drive_state.velocity * ball_flight_time * -1
        target_pose = self.target.transformBy(lead_ball_offset)

        target_yaw = self.get_target_yaw(drive_state.pose, target_pose)
        rotation_rate = self.rotation_PID.calculate(drive_state.heading, target_yaw)

        error = drive_state.heading - target_yaw
        self.current_accuracy = abs((error + 180) % 360 - 180)

        return math_helpers.clamp(rotation_rate, -1.0, 1.0)

    @staticmethod
    def estimate_flight_time(distance: meters) -> float:
        interpolation_distance_data, interpolation_time_data = zip(
            *kShooterData.SHOT_TIME
        )
        time = float(
            np.interp(distance, interpolation_distance_data, interpolation_time_data)
        )
        return math_helpers.clamp(time, 0, 5)

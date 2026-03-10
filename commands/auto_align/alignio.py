import math

import commands2

from wpimath.geometry import Pose2d
from wpimath.controller import PIDController
from wpimath.units import degrees, meters
from wpilib import SmartDashboard

from constants.drive import kAutoAlign
from constants.shooter import kShooterConfig
from constants.field import kHub
from constants.math import kMath

from util.flip_util import FlipUtil
from util import math_helpers

from subsystems.controlled_motor import ControlledTalonMotor
from subsystems.drivetrain import driveio
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

class TurretTargeWithVelocity(TurretTargetBase):
    def __init__(
        self, drivetrain: SwerveDriveTrain, shooter: ControlledTalonMotor, hood: None, target: Pose2d
    ):
        super().__init__(drivetrain, target)
        self._shooter = shooter
        self._hood = hood
        self.flight_time_scalar = kAutoAlign.FLIGHT_TIME_SCALAR

    def calculate_rotation(self) -> float:
        drive_state = self._drivetrain.get_state()
        shooter_rpm = self._shooter.get_rpm()
        hood_angle = 45

        hub_translation = self.target.translation()
        distance_to_hub = hub_translation.distance(drive_state.pose.translation())

        ball_flight_time = (
            self.estimate_flight_time(distance_to_hub, shooter_rpm, hood_angle)
            * self.flight_time_scalar
        )

        lead_ball_offset = drive_state.velocity * ball_flight_time
        target_pose = self.target.transformBy(lead_ball_offset)

        target_yaw = self.get_target_yaw(drive_state.pose, target_pose)
        rotation_rate = self.rotation_PID.calculate(drive_state.heading, target_yaw)
        return math_helpers.clamp(rotation_rate, -1.0, 1.0)

    @staticmethod
    def estimate_flight_time(
        distance: meters, shooter_rpm: float, shot_angle: degrees
    ) -> float:
        if shooter_rpm == 0:
            return 0
        shooter_rps = shooter_rpm / 60
        angular_velocity = shooter_rps * kMath.AngularVelocityPerRPS
        velocity = angular_velocity * kShooterConfig.WHEEL_RADIUS
        horizontal_velocity = velocity * math.cos(shot_angle * kMath.RadiansPerDegree)
        time = distance / horizontal_velocity
        return math_helpers.clamp(time, 0, 5)

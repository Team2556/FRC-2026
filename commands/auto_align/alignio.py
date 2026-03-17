import math

import commands2

import numpy as np

import wpilib
from wpimath.geometry import Pose2d, Translation2d
from wpimath.controller import PIDController
from wpimath.units import degrees, meters
from wpilib import SmartDashboard

from constants.drive import kAutoAlign
from constants.shooter import kShooterConfig, kShooterData

from util.flip_util import FlipUtil
from util import math_helpers
from util.nt_util import NTTable

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
        
        self.nt = NTTable("Auto Align")
        self.nt.float("k_p", kAutoAlign.ROTATION_PID.p)
        self.nt.float("k_i", kAutoAlign.ROTATION_PID.i)
        self.nt.float("k_d", kAutoAlign.ROTATION_PID.d)

    def initialize(self):
        self.target = FlipUtil.fieldPose(self.target_pose_blue)
    
    def periodic(self):
        self.rotation_PID.setP(self.nt.get("k_p"))
        self.rotation_PID.setI(self.nt.get("k_i"))
        self.rotation_PID.setD(self.nt.get("k_d"))
            
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
        
        self._estimated_target_pose = Pose2d()

    def calculate_rotation(self) -> float:
        drive_state = self._drivetrain.get_state()
        hub_translation = self.target.translation()
        robot_translation = drive_state.pose.translation()

        distance_to_hub = hub_translation.distance(robot_translation)
        ball_flight_time = (
            self.estimate_flight_time(distance_to_hub) * self.flight_time_scalar
        )

        robot_rotation = drive_state.pose.rotation()
        vel = drive_state.velocity.translation()
        field_vel = vel.rotateBy(robot_rotation)
        vel_offset = Translation2d(
            field_vel.x * ball_flight_time, field_vel.y * ball_flight_time
        )

        lead_translation = Translation2d(
            hub_translation.x - vel_offset.x, hub_translation.y - vel_offset.y
        )
        target_pose = Pose2d(lead_translation, self.target.rotation())
        self._estimated_target_pose = target_pose

        target_yaw = self.get_target_yaw(drive_state.pose, target_pose)
        rotation_rate = self.rotation_PID.calculate(drive_state.heading, target_yaw)
        
        error = drive_state.heading - target_yaw
        self.current_accuracy = abs((error + 180) % 360 - 180)
        if self.current_accuracy < kAutoAlign.REQUIRED_SHOOT_ACCURACY_DEGREES:
            return 0
        
        return math_helpers.clamp(rotation_rate, -1.0, 1.0)

    @staticmethod
    def estimate_flight_time(distance: meters) -> float:
        return 1
        interpolation_distance_data, interpolation_time_data = zip(
            *kShooterData.SHOT_TIME
        )
        time = float(
            np.interp(distance, interpolation_distance_data, interpolation_time_data)
        )
        return math_helpers.clamp(time, 0, 5)


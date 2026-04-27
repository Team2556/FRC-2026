"""Drive command factories.

Mirrors the flow in 10219's DriveCommands.java:
  1. Magnitude-based deadband + squaring (preserves direction)
  2. Scale to real m/s and rad/s
  3. Convert field-relative to robot-relative via ChassisSpeeds.fromFieldRelativeSpeeds
  4. Pass robot-relative ChassisSpeeds to drivetrain.run_velocity
"""

from __future__ import annotations

import math
from typing import Callable

import commands2
from commands2 import cmd

from wpilib import DriverStation

from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

from util.flip_util import FlipUtil

from subsystems.drivetrain.drivetrain import SwerveDriveTrain

from constants.field import kField
from constants.drive import kDriveConfig
from constants.path.key_poses import kPath

DEADBAND = 0.1


def _deadband(value: float, band: float) -> float:
    """Return 0 if inside the band, otherwise rescale so output is continuous."""
    if abs(value) < band:
        return 0.0
    return math.copysign((abs(value) - band) / (1.0 - band), value)


def _get_linear_velocity_from_joysticks(x: float, y: float) -> Translation2d:
    """Deadband the combined magnitude, square it, preserve direction.

    This is a direct port of 10219's DriveCommands.getLinearVelocityFromJoysticks.
    """
    mag = _deadband(math.hypot(x, y), DEADBAND)
    direction = Rotation2d(math.atan2(y, x))
    mag = mag * mag
    return (
        Pose2d(Translation2d(), direction)
        .transformBy(Transform2d(mag, 0.0, Rotation2d()))
        .translation()
    )


def joystick_drive(
    drivetrain: SwerveDriveTrain,
    x_supplier: Callable[[], float],
    y_supplier: Callable[[], float],
    omega_supplier: Callable[[], float],
) -> commands2.Command:
    """Factory that builds the default teleop drive command.

    Takes axis suppliers (typically lambdas reading raw controller sticks).
    Deadband, squaring, and field-to-robot conversion all happen here.
    """

    def _execute():
        linear = _get_linear_velocity_from_joysticks(
            x_supplier(), y_supplier()
        )

        omega = _deadband(omega_supplier(), DEADBAND)
        omega = math.copysign(omega * omega, omega)
        omega *= drivetrain.get_max_angular_speed() * drivetrain.modifiers.rotation

        # If align override is active, use that instead of joystick omega
        if drivetrain._align_rotation is not None:
            omega = drivetrain._align_rotation

        # Build field-relative speeds at full scale
        field_speeds = ChassisSpeeds(
            linear.X() * drivetrain.get_max_linear_speed(),
            linear.Y() * drivetrain.get_max_linear_speed(),
            omega,
        )

        # Convert field-relative to robot-relative, accounting for alliance
        is_flipped = DriverStation.getAlliance() == DriverStation.Alliance.kRed
        robot_rotation = drivetrain.get_rotation()
        if is_flipped:
            robot_rotation = robot_rotation + Rotation2d(math.pi)

        robot_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            field_speeds, robot_rotation
        )

        drivetrain.run_velocity(robot_speeds)

    return (
        cmd.run(_execute, drivetrain)
        .finallyDo(lambda _interrupted: drivetrain.stop())
    )


def initial_pose(drivetrain: SwerveDriveTrain, pose: Pose2d) -> commands2.Command:
    """Instant command that sets odometry to ``pose`` (flipped for alliance)."""
    return cmd.runOnce(
        lambda: drivetrain.reset_pose(FlipUtil.fieldPose(pose)),
        drivetrain,
    )


def reset_heading(drivetrain: SwerveDriveTrain, angle_offset=0) -> commands2.Command:
    """Instant command that zeros heading while keeping the current translation."""
    def _reset():
        p = drivetrain.get_pose()
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            drivetrain.reset_pose(Pose2d(p.X(), p.Y(), Rotation2d(angle_offset)))
        else:
            drivetrain.reset_pose(Pose2d(p.X(), p.Y(), Rotation2d(math.pi + angle_offset)))

    return cmd.runOnce(_reset, drivetrain)


def set_odometry_to_corner(drivetrain: SwerveDriveTrain, corner: Pose2d) -> commands2.Command:
    """Instant command that sets odometry to a known blue-alliance corner pose."""
    return cmd.runOnce(
        lambda: drivetrain.reset_pose(FlipUtil.fieldPose(corner)),
        drivetrain,
    )

# Pre-built corner poses (blue-alliance origin)
BACK_LEFT_CORNER  = Pose2d(0.35, kField.WIDTH - 0.35, Rotation2d())
BACK_RIGHT_CORNER = Pose2d(0.35, 0.35, Rotation2d())

import commands2

from wpilib import SmartDashboard
from wpimath.units import degrees, meters
from wpimath.geometry import Translation2d, Transform2d, Rotation2d, Pose2d
from wpimath.controller import PIDController

import math

from constants.field import kHub
from constants.drive import kAutoAlign
from constants.shooter import kShooterConfig
from constants.math import kMath

from util import custom_controller, math_helpers
from util.flip_util import FlipUtil

from subsystems.drivetrain import drivetrain, driveio
from subsystems.shooter.controlled_motor import ControlledMotor

from commands.auto_align import alignio


class TurretToPose(alignio.TurretTargetBase):
    def __init__(self, drivetrain, controller, target: Pose2d):
        super().__init__(drivetrain, target)
        self._controller = controller
        self.addRequirements(drivetrain)

    def execute(self):
        rotation_rate = self.calculate_rotation(self._controller)
        self._drivetrain.drive_with_controller(
            self._controller,
            rotation_rate=rotation_rate,
            velocity_mult=kAutoAlign.ROBOT_VELOCITY_MULT,
        )


class HubAlign(alignio.TurretTargeWithVelocity):
    def __init__(self, drivetrain, controller, shooter: ControlledMotor, hood):
        super().__init__(drivetrain, shooter, hood, kHub.POS)
        self._controller = controller

        SmartDashboard.putNumber(
            "Hub Align Flight Time Scalar", self.flight_time_scalar
        )

        self.addRequirements(drivetrain)

    def execute(self):
        self.flight_time_scalar = SmartDashboard.getNumber(
            "Hub Align Flight Time Scalar", self.flight_time_scalar
        )
        
        rotation_rate = self.calculate_rotation()
        self._drivetrain.drive_with_controller(
            self._controller,
            rotation_rate=rotation_rate,
            velocity_mult=kAutoAlign.ROBOT_VELOCITY_MULT,
        )

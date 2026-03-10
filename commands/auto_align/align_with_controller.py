import commands2

from wpilib import SmartDashboard
from wpimath.units import degrees, meters
from wpimath.geometry import Translation2d, Transform2d, Rotation2d, Pose2d
from wpimath.controller import PIDController

import math

from constants.field import kHub, kPassSpots
from constants.drive import kAutoAlign
from constants.shooter import kShooterConfig
from constants.math import kMath
from constants.drive import kDriveConfig

from util import custom_controller, math_helpers
from util.flip_util import FlipUtil

from subsystems.drivetrain import drivetrain, driveio
from subsystems.shooter.controlled_motor import ControlledMotor
from subsystems.controlled_motor import ControlledTalonMotor

from commands.auto_align import alignio


class TurretToPose(alignio.TurretTargeWithVelocity):
    def __init__(self, drivetrain, controller, target: Pose2d, shooter: ControlledTalonMotor, hood: None):
        super().__init__(drivetrain, shooter, hood, target)
        self._controller = controller
        self.addRequirements(drivetrain)

    def execute(self):
        rotation_rate = self.calculate_rotation()
        self._drivetrain.set_target_align_rotation_rate(
            rotation_rate * kAutoAlign.ROBOT_VELOCITY_MULT * kDriveConfig.MAX_ANGULAR_RATE
        )
    
    def end(self, interrupted):
        self._drivetrain.stop_target_align()

class HubAlign(alignio.TurretTargeWithVelocity):
    def __init__(self, drivetrain, controller : custom_controller.XboxController, shooter: ControlledMotor, hood):
        super().__init__(drivetrain, shooter, hood, kHub.POS)
        self._controller = controller

        SmartDashboard.putNumber(
            "Hub Align Flight Time Scalar", self.flight_time_scalar
        )

        # Doesn't need requirement because to only modifies the drivetrain's override_rotation
        # self.addRequirements(drivetrain)

    def execute(self):
        self.flight_time_scalar = SmartDashboard.getNumber(
            "Hub Align Flight Time Scalar", self.flight_time_scalar
        )
        
        rotation_rate = self.calculate_rotation()
        self._drivetrain.set_target_align_rotation_rate(
            rotation_rate * kAutoAlign.ROBOT_VELOCITY_MULT * kDriveConfig.MAX_ANGULAR_RATE
        )
    
    def end(self, interrupted):
        self._drivetrain.stop_target_align()

class ConditionalTargetAlign():
    def __init__(
        self, 
        drivetrain : drivetrain.SwerveDriveTrain,
        controller, 
        shooter: ControlledMotor, 
        hood
        ):
        
        self.drivetrain = drivetrain
        
        self.hub_align = HubAlign(self.drivetrain, controller, shooter, hood)
        self.pass_left = TurretToPose(self.drivetrain, controller, kPassSpots.PASS_SPOT_LEFT, shooter, hood)
        self.pass_right = TurretToPose(self.drivetrain, controller, kPassSpots.PASS_SPOT_RIGHT, shooter, hood)

import commands2
from wpilib import SmartDashboard
from wpimath.geometry import Translation2d, Transform2d, Rotation2d
from wpimath.controller import PIDController
import math

from constants.field import kHub
from constants.drive import kAutoAlign
from constants.shooter import kShooterConfig

from util import custom_controller, math_helpers
from subsystems.drivetrain import drivetrain


class HubAlign(commands2.Command):
    def __init__(
        self,
        drivetrain: drivetrain.SwerveDriveTrain,
        controller: custom_controller.XboxController,
    ):
        super.__init__()
        self._drivetrain = drivetrain
        self._controller = controller

        self.shooter_offset = kShooterConfig.SHOOTER_OFFSET
        SmartDashboard.putNumber("Shooter Offset X", self.shooter_offset.x)
        SmartDashboard.putNumber("Shooter Offset Y", self.shooter_offset.y)

        self.shooter_direction = kShooterConfig.SHOOTER_DIRECTION

        self.rotation_PID = PIDController(
            kAutoAlign.ROTATION_PID.p,
            kAutoAlign.ROTATION_PID.i,
            kAutoAlign.ROTATION_PID.d,
        )
        self.rotation_PID.enableContinuousInput(-180.0, 180.0)

        self.distance_mult = kAutoAlign.CORRECTION_MULT
        SmartDashboard.putNumber("Auto Align Correction Mult", self.distance_mult)

        self.addRequirements(drivetrain)

    def calculate_target_yaw(self, robot_pose):
        shooter_field_pos = robot_pose.translation() + self.shooter_offset.rotateBy(
            robot_pose.rotation()
        )
        to_hub = kHub.POS - shooter_field_pos

        return math.degrees(math.atan2(to_hub.Y(), to_hub.X()))

    def execute(self):
        self.shooter_offset = Translation2d(
            SmartDashboard.getNumber("Shooter Offset Y", self.shooter_offset.y),
            SmartDashboard.getNumber("Shooter Offset X", self.shooter_offset.x),
        )
        self.distance_mult = SmartDashboard.getNumber("Auto Align Distance Mult", 0)
        self.distance_exp = SmartDashboard.getNumber("Auto Align DIstance Exponent", 0)

        drive_state = self._drivetrain.get_state()
        distance_to_hub = kHub.POS.distance(drive_state.pose.translation())

        velocity_correction = self.distance_mult * (distance_to_hub**self.distance_exp)

        estimated_pos = drive_state.pose.transformBy(
            drive_state.velocity * velocity_correction
        )

        shooter_offset = self.shooter_offset.rotateBy(drive_state.rotation)
        shooter_pose = estimated_pos.transformBy(
            Transform2d(shooter_offset, Rotation2d())
        )

        target_heading = (
            self.calculate_target_yaw(shooter_pose) - self.shooter_direction
        )
        rotation_rate = self.rotation_PID.calculate(drive_state.heading, target_heading)
        rotation_rate = math_helpers.clamp(rotation_rate, -1.0, 1.0)

        self._drivetrain.drive_with_controller(
            self._controller,
            rotation_rate=rotation_rate,
            velocity_mult=kAutoAlign.ROBOT_VELOCITY_MULT,
        )

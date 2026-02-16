import commands2
from wpilib import SmartDashboard
from wpimath.geometry import Translation2d, Transform2d, Rotation2d
import math

from constants._configs import kShooter, kField
from constants.tuners import kAutoAlign

from util import custom_controller, math_helpers
from subsystems.drivetrain import drivetrain


class HubAlign(commands2.Command):
    def __init__(
        self,
        drivetrain: drivetrain.SwerveDriveTrain,
        controller: custom_controller.XboxController,
    ):
        self._drivetrain = drivetrain
        self._controller = controller

        self.shooter_offset = kShooter.SHOOTER_OFFSET
        SmartDashboard.putNumber("Shooter Offset X", self.shooter_offset.x)
        SmartDashboard.putNumber("Shooter Offset Y", self.shooter_offset.y)

        self.shooter_direction = kShooter.SHOOTER_DIRECTION

        self.hub_pos = kField.RED_HUB_POS  # Translation2d

        self.rotation_PID = kAutoAlign.ROTATIONAL_PID
        self.rotation_PID.enableContinuousInput(-180.0, 180.0)

        self.correction_mult = kAutoAlign.CORRECTION_MULT
        SmartDashboard.putNumber("Auto Align Correction Mult", self.correction_mult)

        self.addRequirements(drivetrain)

    def calculate_target_yaw(self, robot_pose):
        shooter_field_pos = robot_pose.translation() + self.shooter_offset.rotateBy(
            robot_pose.rotation()
        )
        to_hub = self.hub_pos - shooter_field_pos

        return math.degrees(math.atan2(to_hub.Y(), to_hub.X()))

    def execute(self):
        self.shooter_offset = Translation2d(
            SmartDashboard.getNumber("Shooter Offset Y", self.shooter_offset.y),
            SmartDashboard.getNumber("Shooter Offset X", self.shooter_offset.x),
        )
        self.correction_mult = SmartDashboard.getNumber("Auto Align Correction Mult", 0)

        robot_state = self._drivetrain._drivetrain.get_state()
        robot_pose = robot_state.pose
        robot_velocity = robot_state.speeds

        distance_to_hub = self.hub_pos.distance(robot_pose.translation())
        distance_mult = distance_to_hub * self.correction_mult
        distance_mult = math_helpers.clamp(distance_mult, 0, 0.25)

        estimate_pos_change = (
            Translation2d(robot_velocity.vx, robot_velocity.vy) * distance_mult
        )
        robot_pose += estimate_pos_change

        robot_rotation = robot_pose.rotation()
        robot_heading = robot_rotation.degrees()
        shooter_offset = self.shooter_offset.rotateBy(robot_rotation)

        shooter_transform = Transform2d(shooter_offset, Rotation2d())
        robot_pose += shooter_transform

        target_heading = self.calculate_target_yaw(robot_pose) - self.shooter_direction

        rotation_rate = self.rotation_PID.calculate(robot_heading, target_heading)
        rotation_rate = math_helpers.clamp(rotation_rate, -1.0, 1.0)

        self._drivetrain.drive_with_controller(
            self._controller,
            rotation_rate=rotation_rate,
            velocity_mult=kAutoAlign.ROBOT_VELOCITY_MULT,
        )

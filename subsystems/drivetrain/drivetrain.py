import commands2
from commands2 import Command
from commands2.sysid import SysIdRoutine

import wpilib
from wpimath.units import rotationsToRadians
from wpimath.geometry import Transform2d, Rotation2d

from phoenix6 import swerve

from subsystems.drivetrain.swerve_tuner import TunerConstants
from subsystems.drivetrain.telemetry import Telemetry

from util.custom_controller import CommandXboxController
from util.robot_zone_checker import RobotZoneChecker
from util.nt_util import NTTable
from util.flip_util import FlipUtil
from util.math_helpers import distanceFromPose2dtoPose2d

from constants.drive import kDriveConfig
from constants.drive import kAutoAlign
from constants.field import kHub

from .driveio import CustomSwerve


class SwerveDriveTrain(commands2.Subsystem):
    def __init__(self):
        self._drivetrain = TunerConstants.create_drivetrain()

        self._drive = swerve.requests.FieldCentric().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        self._logger = Telemetry(kDriveConfig.SPEED_MULT)
        self._drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

        self._field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self._field)

        self.target_align_rotation_rate = 0
        self.do_target_align = False

        self.nt = NTTable("Drivetrain")
        self.nt.bool("Do Target Align")
        self.nt.float("Target Align Rotation Rate")
        self.nt.float("Distance to Hub")

    def drive_with_controller(
        self,
        controller: CommandXboxController,
        velocity_x=None,
        velocity_y=None,
        rotation_rate=None,
        velocity_mult=1,
        rotation_mult=1,
    ):
        _velocity_x = velocity_x if velocity_x else -controller.getLeftY()
        _velocity_y = velocity_y if velocity_y else -controller.getLeftX()
        _rotational_rate = rotation_rate if rotation_rate else -controller.getRightX()

        if self.do_target_align:
            velocity_mult *= kAutoAlign.ROBOT_VELOCITY_MULT

        self._drivetrain.set_control(
            self._drive.with_velocity_x(
                _velocity_x
                * kDriveConfig.SPEED_MULT
                * velocity_mult
                * TunerConstants.speed_at_12_volts
            )  # Drive forward with negative Y (forward)
            .with_velocity_y(
                _velocity_y
                * kDriveConfig.SPEED_MULT
                * velocity_mult
                * TunerConstants.speed_at_12_volts
            )  # Drive left with negative X (left)
            .with_rotational_rate(
                self.target_align_rotation_rate
                if self.do_target_align
                else (
                    _rotational_rate
                    * kDriveConfig.MAX_ANGULAR_RATE
                    * kDriveConfig.ROTATION_MULT
                    * rotation_mult
                )
            )  # Drive counterclockwise with negative X (left)
        )

    def drive_with_values(self, velocity_x=0, velocity_y=0, rotation_rate=0):
        self._drivetrain.set_control(
            self._drive.with_velocity_x(velocity_x)
            .with_velocity_y(velocity_y)
            .with_rotational_rate(
                self.target_align_rotation_rate
                if self.do_target_align
                else rotation_rate
            )
        )

    def change_speed_mult(self, speed=1, rotation=1):
        kDriveConfig.SPEED_MULT = speed
        kDriveConfig.ROTATION_MULT = rotation

    def set_target_align_rotation_rate(self, rotation_rate=0):
        self.target_align_rotation_rate = rotation_rate
        self.do_target_align = True

    def stop_target_align(self):
        self.do_target_align = False

    def should_stop_shooting(self):
        """Uses a bit of velocity projection to detect if robot should stop shooting to transition between bump/trench"""
        state = self.get_state()
        robot_pose = state.pose
        robot_velocity_raw = state.velocity.translation().rotateBy(robot_pose.rotation())
        robot_velocity = Transform2d(robot_velocity_raw, Rotation2d())

        projected_pose = robot_pose.transformBy(
            robot_velocity * kDriveConfig.LOOKAHEAD_SECONDS
        )
        half_projected_pose = robot_pose.transformBy(
            robot_velocity * kDriveConfig.LOOKAHEAD_SECONDS * 0.5
        )
        return (
            RobotZoneChecker.is_near_transition_zone(projected_pose)
            or RobotZoneChecker.is_near_transition_zone(half_projected_pose)
            or RobotZoneChecker.is_near_transition_zone(robot_pose)
        )

    def _stop(self):
        self.drive_with_values(velocity_x=0, velocity_y=0, rotation_rate=0)

    def get_state(self) -> CustomSwerve.DriveState:
        return CustomSwerve.BuildDriveState(self._drivetrain.get_state())

    def seed_field_centric(self):
        return self._drivetrain.seed_field_centric()

    # ── SysId characterization ──────────────────────────────────────

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        cmd = self._drivetrain.sys_id_quasistatic(direction)
        cmd.addRequirements(self)  # also require the outer wrapper so default drive is interrupted
        return cmd

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        cmd = self._drivetrain.sys_id_dynamic(direction)
        cmd.addRequirements(self)  # also require the outer wrapper so default drive is interrupted
        return cmd

    def set_sys_id_routine(self, routine: str):
        """Switch active SysId routine: 'translation', 'steer', or 'rotation'."""
        routines = {
            "translation": self._drivetrain._sys_id_routine_translation,
            "steer": self._drivetrain._sys_id_routine_steer,
            "rotation": self._drivetrain._sys_id_routine_rotation,
        }
        if routine in routines:
            self._drivetrain._sys_id_routine_to_apply = routines[routine]

    def periodic(self):
        pose = self.get_state().pose
        self._field.setRobotPose(pose)

        self.nt.set(
            "Distance to Hub",
            distanceFromPose2dtoPose2d(pose, FlipUtil.fieldPose(kHub.POS)),
        )
        self.nt.set("Do Target Align", self.do_target_align)
        self.nt.set("Target Align Rotation Rate", self.target_align_rotation_rate)

from dataclasses import dataclass

import commands2

import wpilib
from wpimath.geometry import Transform2d, Rotation2d

from phoenix6 import swerve

from util.custom_controller import XboxController
from util.robot_zone_checker import RobotZoneChecker
from util.nt_util import NTTable
from util.flip_util import FlipUtil
from util.math_helpers import distanceFromPose2dtoPose2d

from subsystems.drivetrain.swerve_tuner import TunerConstants
from subsystems.drivetrain.telemetry import Telemetry

from constants.drive import kDriveConfig, kAutoAlign
from constants.field import kHub

from .driveio import CustomSwerve


@dataclass
class DriveModifiers:
    speed: float = 1.0
    rotation: float = 1.0


class SwerveDriveTrain(commands2.Subsystem):
    NORMAL = DriveModifiers(speed=1.0, rotation=1.0)
    SLOW   = DriveModifiers(speed=kDriveConfig.SLOW_SPEED_MULT, rotation=0.75)
    SLOW_ROTATE = DriveModifiers(speed=kDriveConfig.SLOW_SPEED_MULT, rotation=0.2)

    def __init__(self):
        super().__init__()
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

        self._modifiers = DriveModifiers()
        self._align_rotation: float | None = None

        self.nt = NTTable("Drivetrain")
        self._field = wpilib.Field2d()
        self.nt.sendable("Field", self._field)
        self.nt.bool("Align Active")
        self.nt.float("Align Rotation Rate")
        self.nt.float("Distance to Hub")
        self.nt.float("Align Tuner")

    def drive(self, vx: float, vy: float, omega: float) -> None:
        """
        Apply pre-computed velocities directly to the swerve hardware.

        ``vx`` and ``vy`` are in **m/s** (field-relative); ``omega`` is in
        **rad/s**.  No modifiers or overrides are applied — this is the
        interface for autonomous and path-following commands that already
        produce real-unit outputs.

        For human-driven motion use ``drive_from_controller()`` instead.
        """
        self._drivetrain.set_control(
            self._drive
            .with_velocity_x(vx)
            .with_velocity_y(vy)
            .with_rotational_rate(omega)
        )

    def drive_from_controller(self, controller: XboxController) -> None:
        """
        Read joystick axes, apply the active modifier pipeline and any
        alignment-rotation override, then drive the robot.

        Modifier pipeline (applied in order):
          1. Raw joystick input [-1, 1]
          2. ``kDriveConfig.SPEED_MULT``          — global base scale
          3. ``self._modifiers.speed``             — preset (slow, normal …)
          4. ``TunerConstants.speed_at_12_volts``  — converts to m/s
          5. ``kAutoAlign.ROBOT_VELOCITY_MULT``    — damps translation while
                                                    an alignment override is active

        To add a new modifier dimension, add a field to ``DriveModifiers``
        and reference it here.
        """
        # Guard: if no joystick is connected on this port (common in sim when
        # no virtual joystick is configured), stop rather than read garbage axes.
        if not wpilib.DriverStation.isJoystickConnected(controller.getHID().getPort()):
            self.stop()
            return

        raw_vx    = -controller.getLeftY()
        raw_vy    = -controller.getLeftX()
        raw_omega = -controller.getRightX()
        
        speed_scale = (
            kDriveConfig.SPEED_MULT
            * self._modifiers.speed
            * TunerConstants.speed_at_12_volts
        )
        if self._align_rotation is not None:
            speed_scale *= kAutoAlign.ROBOT_VELOCITY_MULT

        actual_omega = (
            self._align_rotation
            if self._align_rotation is not None
            else raw_omega * kDriveConfig.MAX_ANGULAR_RATE * self._modifiers.rotation
        )

        self._drivetrain.set_control(
            self._drive
            .with_velocity_x(raw_vx * speed_scale)
            .with_velocity_y(raw_vy * speed_scale)
            .with_rotational_rate(actual_omega)
        )

    def stop(self) -> None:
        """Immediately zero all drive outputs."""
        self._drivetrain.set_control(
            self._drive
            .with_velocity_x(0)
            .with_velocity_y(0)
            .with_rotational_rate(0)
        )

    def set_modifiers(self, modifiers: DriveModifiers) -> None:
        """Activate a modifier preset (e.g. ``SwerveDriveTrain.SLOW``)."""
        self._modifiers = modifiers

    def reset_modifiers(self) -> None:
        """Return to default (unmodified) drive behaviour."""
        self._modifiers = DriveModifiers()

    def set_align_rotation(self, rate: float) -> None:
        """
        Override the rotational output with a fixed rad/s value.

        Called each execute cycle by an alignment command.  Translation is
        automatically damped while an override is active.  Always paired
        with ``clear_align_rotation()`` in the command's ``end()``.
        """
        self._align_rotation = rate

    def clear_align_rotation(self) -> None:
        """Remove the rotation override and return to normal joystick control."""
        self._align_rotation = None

    def should_stop_shooting(self) -> bool:
        """
        Project the robot forward and return ``True`` if it is approaching
        or inside the bump/trench transition zone.
        """
        state = self.get_state()
        robot_pose = state.pose
        robot_velocity_raw = state.velocity.translation().rotateBy(robot_pose.rotation())
        robot_velocity = Transform2d(robot_velocity_raw, Rotation2d())

        projected = robot_pose.transformBy(
            robot_velocity * kDriveConfig.LOOKAHEAD_SECONDS
        )
        half_projected = robot_pose.transformBy(
            robot_velocity * kDriveConfig.LOOKAHEAD_SECONDS * 0.5
        )
        return (
            RobotZoneChecker.is_near_transition_zone(projected)
            or RobotZoneChecker.is_near_transition_zone(half_projected)
            or RobotZoneChecker.is_near_transition_zone(robot_pose)
        )

    def get_state(self) -> CustomSwerve.DriveState:
        return CustomSwerve.BuildDriveState(self._drivetrain.get_state())

    def reset_pose(self, pose) -> None:
        self._drivetrain.reset_pose(pose)

    def add_vision_measurement(self, pose, timestamp_seconds, std_devs) -> None:
        self._drivetrain.add_vision_measurement(pose, timestamp_seconds, std_devs)

    def seed_field_centric(self) -> None:
        self._drivetrain.seed_field_centric()

    def periodic(self) -> None:
        state = self.get_state()
        pose  = state.pose

        self._field.setRobotPose(pose)

        self.nt.set(
            "Distance to Hub",
            round(distanceFromPose2dtoPose2d(pose, FlipUtil.fieldPose(kHub.POS)), 2),
        )
        self.nt.set("Align Active", self._align_rotation is not None)
        self.nt.set("Align Rotation Rate", self._align_rotation or 0.0)
        self.nt.set("Align Tuner", round(kAutoAlign.ALIGN_TUNER_OFFSET, 1))
        self.nt.update_sendables()  # pushes Field2d current pose to NT every loop

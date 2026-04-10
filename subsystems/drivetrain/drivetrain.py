from dataclasses import dataclass

import commands2

import wpilib
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

from phoenix6 import swerve

from util.robot_zone_checker import RobotZoneChecker
from util.nt_util import NTTable
from util.flip_util import FlipUtil
from util.math_helpers import distanceFromPose2dtoPose2d

from subsystems.drivetrain.swerve_tuner import TunerConstants
from subsystems.drivetrain.telemetry import Telemetry

from constants.drive import kDriveConfig, kAutoAlign
from constants.field import kHub


print("LOADING NEW DRIVETRAIN", __file__)

@dataclass
class DriveModifiers:
    speed: float = 0.45
    rotation: float = 0.2


class SwerveDriveTrain(commands2.Subsystem):
    NORMAL      = DriveModifiers(speed=0.45, rotation=0.2)
    FAST        = DriveModifiers(speed=1.0, rotation=0.3)
    SLOW        = DriveModifiers(speed=kDriveConfig.SLOW_SPEED_MULT, rotation=0.5)
    SLOW_ROTATE = DriveModifiers(speed=kDriveConfig.SLOW_SPEED_MULT, rotation=0.2)

    # ------------------------------------------------------------------
    # Constructor
    # ------------------------------------------------------------------

    def __init__(self):
        super().__init__()
        self._drivetrain = TunerConstants.create_drivetrain()

        self._apply_robot_speeds = swerve.requests.ApplyRobotSpeeds()
        self._brake = swerve.requests.SwerveDriveBrake()

        self._logger = Telemetry(TunerConstants.speed_at_12_volts)
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

    # ------------------------------------------------------------------
    # Velocity
    # ------------------------------------------------------------------

    def run_velocity(self, speeds: ChassisSpeeds, ignore_modifiers=False) -> None:
        """Run the drivetrain at the given robot-relative ChassisSpeeds.

        Applies the speed modifier to translation, discretizes for skew
        correction, then passes to CTRE ApplyRobotSpeeds (which handles
        kinematics and desaturation internally).
        """
        
        scaled = ChassisSpeeds(
            speeds.vx * (1 if ignore_modifiers else self._modifiers.speed),
            speeds.vy * (1 if ignore_modifiers else self._modifiers.speed),
            self._align_rotation if self._align_rotation else speeds.omega,
        )

        self._drivetrain.set_control(
            self._apply_robot_speeds
            .with_speeds(ChassisSpeeds.discretize(scaled, 0.02))
        )

    def stop(self) -> None:
        self.run_velocity(ChassisSpeeds())

    def stop_with_brake(self) -> None:
        self._drivetrain.set_control(self._brake)

    # ------------------------------------------------------------------
    # Modifiers
    # ------------------------------------------------------------------

    def set_modifiers(self, modifiers: DriveModifiers) -> None:
        self._modifiers = modifiers

    def reset_modifiers(self) -> None:
        self._modifiers = self.NORMAL

    @property
    def modifiers(self) -> DriveModifiers:
        return self._modifiers

    # ------------------------------------------------------------------
    # Alignment overlay
    # ------------------------------------------------------------------

    def set_align_rotation(self, rate: float) -> None:
        """Override rotational output with a fixed rad/s value.

        Called each execute cycle by an alignment command. Always paired
        with ``clear_align_rotation()`` in the command's ``end()``.
        """
        self._align_rotation = rate

    def clear_align_rotation(self) -> None:
        self._align_rotation = None

    # ------------------------------------------------------------------
    # Shooting gate
    # ------------------------------------------------------------------

    def should_stop_shooting(self) -> bool:
        """Return True if robot is approaching the bump/trench transition zone."""
        pose = self.get_pose()
        speeds = self.get_speeds()
        velocity_raw = Translation2d(speeds.vx, speeds.vy).rotateBy(pose.rotation())
        velocity = Transform2d(velocity_raw, Rotation2d())

        projected = pose.transformBy(velocity * kDriveConfig.LOOKAHEAD_SECONDS)
        half_projected = pose.transformBy(velocity * kDriveConfig.LOOKAHEAD_SECONDS * 0.5)

        return (
            RobotZoneChecker.is_near_transition_zone(projected)
            or RobotZoneChecker.is_near_transition_zone(half_projected)
            or RobotZoneChecker.is_near_transition_zone(pose)
        )

    # ------------------------------------------------------------------
    # Getters
    # ------------------------------------------------------------------

    def get_pose(self) -> Pose2d:
        return self._drivetrain.get_state().pose

    def get_rotation(self) -> Rotation2d:
        return self.get_pose().rotation()

    def get_speeds(self) -> ChassisSpeeds:
        return self._drivetrain.get_state().speeds

    def get_max_linear_speed(self) -> float:
        return TunerConstants.speed_at_12_volts

    def get_max_angular_speed(self) -> float:
        return kDriveConfig.MAX_ANGULAR_RATE

    def get_state(self):
        """Backward compatibility — returns a DriveState with pose and velocity as Transform2d."""
        from .driveio import CustomSwerve
        return CustomSwerve.BuildDriveState(self._drivetrain.get_state())

    # ------------------------------------------------------------------
    # Pose management
    # ------------------------------------------------------------------

    def reset_pose(self, pose: Pose2d) -> None:
        self._drivetrain.reset_pose(pose)

    def add_vision_measurement(self, pose, timestamp_seconds, std_devs) -> None:
        self._drivetrain.add_vision_measurement(pose, timestamp_seconds, std_devs)

    def seed_field_centric(self) -> None:
        self._drivetrain.seed_field_centric()

    # ------------------------------------------------------------------
    # Periodic
    # ------------------------------------------------------------------

    def periodic(self) -> None:
        pose = self.get_pose()

        self._field.setRobotPose(pose)

        self.nt.set(
            "Distance to Hub",
            round(distanceFromPose2dtoPose2d(pose, FlipUtil.fieldPose(kHub.POS)), 2),
        )
        self.nt.set("Align Active", self._align_rotation is not None)
        self.nt.set("Align Rotation Rate", self._align_rotation or 0.0)
        self.nt.update_sendables()

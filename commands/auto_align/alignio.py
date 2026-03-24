import math

from wpimath.geometry import Pose2d, Translation2d
from wpimath.controller import PIDController
from wpimath.units import degrees, meters

from util.flip_util import FlipUtil
from util.nt_util import NTTable
from util import math_helpers

from subsystems.drivetrain.drivetrain import SwerveDriveTrain

from constants.drive import kAutoAlign
from constants.shooter import kShooterConfig


class RotationCalculator:
    """
    Plain helper (not a Command) that computes the rotation rate needed to aim
    a shooter at a field target, with optional velocity-lead compensation.

    Usage::

        calc = RotationCalculator(drivetrain, target_blue_pose)
        calc.initialize()                # call once when the owning command starts
        calc.update_pid()                # call each execute() loop to sync NT gains
        rate = calc.calculate_rotation() # clamped [-1, 1] rotation rate
        calc.current_accuracy            # absolute yaw error in degrees

    The target pose is stored in blue-alliance coordinates and flipped on
    ``initialize()`` so that ``calc.target`` is always the correct alliance side.
    """

    def __init__(self, drivetrain: SwerveDriveTrain, target: Pose2d):
        self._drivetrain = drivetrain
        self.target_pose_blue = target
        self.target = FlipUtil.fieldPose(self.target_pose_blue)

        self.shooter_offset = kShooterConfig.SHOOTER_OFFSET
        self.shooter_direction = kShooterConfig.SHOOTER_DIRECTION
        self.flight_time_scalar = kAutoAlign.FLIGHT_TIME_SCALAR

        self.rotation_PID = PIDController(
            kAutoAlign.ROTATION_PID.p,
            kAutoAlign.ROTATION_PID.i,
            kAutoAlign.ROTATION_PID.d,
        )
        self.rotation_PID.enableContinuousInput(-180.0, 180.0)

        self.current_accuracy: float = 180.0

        self.nt = NTTable("Auto Align")
        self.nt.float("k_p", kAutoAlign.ROTATION_PID.p)
        self.nt.float("k_i", kAutoAlign.ROTATION_PID.i)
        self.nt.float("k_d", kAutoAlign.ROTATION_PID.d)


    def initialize(self) -> None:
        """Flip target to the current alliance side. Call once at command start."""
        self.target = FlipUtil.fieldPose(self.target_pose_blue)

    def update_pid(self) -> None:
        """Sync PID gains from NetworkTables. Call once per execute() loop."""
        self.rotation_PID.setP(self.nt.get("k_p"))
        self.rotation_PID.setI(self.nt.get("k_i"))
        self.rotation_PID.setD(self.nt.get("k_d"))

    def get_target_yaw(self, robot_pose: Pose2d, target_pose: Pose2d) -> degrees:
        """Return the field-frame yaw (degrees) the shooter must face."""
        shooter_field_pos = robot_pose.translation() + self.shooter_offset.rotateBy(
            robot_pose.rotation()
        )
        vector_pointer = target_pose.translation() - shooter_field_pos
        target_robot_yaw = math.degrees(
            math.atan2(vector_pointer.Y(), vector_pointer.X())
        )
        return target_robot_yaw - self.shooter_direction

    def with_target(self, target: Pose2d) -> "RotationCalculator":
        """Override the active target (already alliance-flipped). Returns self."""
        self.target = target
        return self

    def calculate_rotation(self) -> float:
        """
        Compute a clamped [-1, 1] rotation rate toward ``self.target``.
        Also updates ``self.current_accuracy`` (absolute yaw error in degrees).
        """
        drive_state = self._drivetrain.get_state()
        hub_translation = self.target.translation()
        robot_translation = drive_state.pose.translation()

        distance_to_hub = hub_translation.distance(robot_translation)
        ball_flight_time = (
            self._estimate_flight_time(distance_to_hub) * self.flight_time_scalar
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

        target_yaw = (
            self.get_target_yaw(drive_state.pose, target_pose)
            + kAutoAlign.ALIGN_TUNER_OFFSET
        )
        rotation_rate = self.rotation_PID.calculate(drive_state.heading, target_yaw)

        error = drive_state.heading - target_yaw
        self.current_accuracy = abs((error + 180) % 360 - 180)

        return math_helpers.clamp(rotation_rate, -1.0, 1.0)

    @staticmethod
    def _estimate_flight_time(distance: meters) -> float:
        # Flight-time lead compensation is not yet tuned; returns 0 (no lead).
        return 0

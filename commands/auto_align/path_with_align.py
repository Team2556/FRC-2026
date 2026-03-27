from wpimath.geometry import Rotation2d, Pose2d

from commands.path_commands.drive_to_a_spot import DriveToASpot
from commands.auto_align.alignio import RotationCalculator

from constants.drive import kDriveConfig


class DriveWithAlign(DriveToASpot):
    """
    DriveToASpot with hub-align rotation instead of path-computed heading.
    Suitable for simultaneous drive-and-aim sequences in auto or teleop.

    The ``alignment_target`` pose is stored in blue-alliance coordinates and
    flipped automatically on ``initialize()``.
    """

    def __init__(
        self,
        alignment_target: Pose2d = Pose2d(),
        *args,
        **kwargs,
    ):
        DriveToASpot.__init__(self, *args, **kwargs)
        self._calc = RotationCalculator(self.drivetrain, alignment_target)

    def initialize(self) -> None:
        super().initialize()
        self._calc.initialize()

    def calcutate_angular_velocity(self) -> Rotation2d:
        self._calc.update_pid()
        rotation_rate = self._calc.calculate_rotation()
        rotation_radians = rotation_rate * kDriveConfig.MAX_ANGULAR_RATE
        return Rotation2d(rotation_radians)

    def isFinished(self) -> bool:
        distance_from_target = self.pose_estimate.translation().distance(
            self.target_pose.translation()
        )
        self.is_within_distance = distance_from_target < self.end_tolerance
        return self.is_within_distance

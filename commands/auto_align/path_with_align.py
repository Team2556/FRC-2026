from wpimath.geometry import Rotation2d, Pose2d

from constants.drive import kDriveConfig

from subsystems.shooter.dual_shooter import DualMotorShooter

from commands.path_commands.drive_to_a_spot import DriveToASpot
from commands.auto_align import alignio


class DriveWithAlign(DriveToASpot):
    """
    DriveToASpot but with hub align rotation instead 
    (functions as a better command for both a target_align command and drive_to_a_spot)
    """

    def __init__(
        self,
        alignment_target: Pose2d = Pose2d(),
        *args, **kwargs
    ):
        DriveToASpot.__init__(self, *args, **kwargs)
        self._hub_align = alignio.TurretTargetWithVelocity(
            self.drivetrain, alignment_target
        )

    def initialize(self):
        super().initialize()
        self._hub_align.initialize()

    def calcutate_angular_velocity(self) -> Rotation2d:
        rotation_rate = self._hub_align.calculate_rotation()
        rotation_radians = rotation_rate * kDriveConfig.MAX_ANGULAR_RATE
        return Rotation2d(rotation_radians)
    
    def isFinished(self):
        # Translation stuff
        distance_from_target = self.pose_estimate.translation().distance(self.target_pose.translation())
        self.is_within_distance = distance_from_target < self.end_tolerance
        
        return self.is_within_distance

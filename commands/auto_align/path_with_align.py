from wpimath.geometry import Rotation2d, Pose2d

from constants.drive import kDriveConfig

from subsystems.controlled_motor import ControlledTalonMotor

from commands.path_commands.drive_to_a_spot import DriveToASpot
from commands.auto_align import alignio


class DriveWithAlign(DriveToASpot):
    """
    DriveToASpot but with hub align rotation instead 
    (functions as a better command for both a target_align command and drive_to_a_spot)
    """

    def __init__(
        self,
        shooter: ControlledTalonMotor,
        hood: None,
        alignment_target: Pose2d = Pose2d(),
        **args
    ):
        DriveToASpot.__init__(self, **args)
        self._hub_align = alignio.TurretTargeWithVelocity(
            self.drivetrain, shooter, hood, alignment_target
        )

    def initialize(self):
        super().initialize()
        self._hub_align.initialize()

    def calcutate_angular_velocity(self) -> Rotation2d:
        rotation_rate = self._hub_align.calculate_rotation()
        rotation_radians = rotation_rate * kDriveConfig.MAX_ANGULAR_RATE
        return Rotation2d(rotation_radians)

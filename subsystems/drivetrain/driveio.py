from dataclasses import dataclass, field

from phoenix6.swerve.swerve_drivetrain import SwerveDrivetrain

from wpiutil.wpistruct import make_wpistruct

from wpimath.geometry import Rotation2d, Pose2d, Transform2d, Translation2d
from wpimath.units import degrees


class CustomSwerve:

    @dataclass
    class DriveState:
        robot_state: SwerveDrivetrain.SwerveDriveState = field(
            default_factory=SwerveDrivetrain.SwerveDriveState
        )
        pose: Pose2d = field(default_factory=Pose2d)
        rotation: Rotation2d = field(default_factory=Rotation2d)
        heading: degrees = 0.0
        velocity: Transform2d = field(default_factory=Transform2d)

    @staticmethod
    def BuildDriveState(robot_state: SwerveDrivetrain.SwerveDriveState):
        return CustomSwerve.DriveState(
            robot_state=robot_state,
            pose=robot_state.pose,
            rotation=robot_state.pose.rotation(),
            heading=robot_state.pose.rotation().degrees(),
            velocity=Transform2d(
                Translation2d(robot_state.speeds.vx, robot_state.speeds.vy),
                Rotation2d(robot_state.speeds.omega)
            ),
        )

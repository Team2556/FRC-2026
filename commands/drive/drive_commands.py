import commands2

from subsystems.drivetrain import drivetrain

from wpimath.geometry import Pose2d, Rotation2d
from util.flip_util import FlipUtil
from wpilib import DriverStation
from math import pi

from constants.field import kField

class ControllerDrive(commands2.Command):
    def __init__(self, drivetrain: drivetrain.SwerveDriveTrain, controller):
        super().__init__()
        self._drivetrain = drivetrain
        self._controller = controller

        self.addRequirements(self._drivetrain)

    def execute(self):
        self._drivetrain.drive_with_controller(self._controller)

    def end(self, interrupt):
        self._drivetrain._stop()

class InitialPose(commands2.Command):
    def __init__(self, drivetrain: drivetrain.SwerveDriveTrain, pose : Pose2d):
        self._drivetrain = drivetrain
        self.pose = pose

    def execute(self):
        self._drivetrain._drivetrain.reset_pose(
            FlipUtil.fieldPose(
                Pose2d(
                    self.pose.X(),
                    self.pose.Y(),
                    Rotation2d(self.pose.rotation().radians() + pi)
                )
            )
        )

    def isFinished(self):
        return True

class AutoDrive(commands2.Command):
    def __init__(self, drivetrain: drivetrain.SwerveDriveTrain):
        super().__init__()
        self._drivetrain = drivetrain

        self.addRequirements(self._drivetrain)

    def execute(self):
        self._drivetrain.drive_with_values()

class OverrideRotation(commands2.Command):
    def __init__(self, drivetrain: drivetrain.SwerveDriveTrain):
        super().__init__()
        self._drivetrain = drivetrain

        self.addRequirements(self._drivetrain)

    def initialize(self):
        self.pose = self._drivetrain.get_state().pose
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self._drivetrain._drivetrain.reset_pose(
                Pose2d(
                    self.pose.X(),
                    self.pose.Y(),
                    Rotation2d()
                )
            )
        else:
            self._drivetrain._drivetrain.reset_pose(
                Pose2d(
                    self.pose.X(),
                    self.pose.Y(),
                    Rotation2d()
                )
            )
    
    def isFinished(self):
        return True

class SetOdometryBackLeft(commands2.Command):
    def __init__(self, drivetrain: drivetrain.SwerveDriveTrain):
        super().__init__()
        self._drivetrain = drivetrain

        self.addRequirements(self._drivetrain)

    def initialize(self):
        self.pose = self._drivetrain.get_state().pose
        self._drivetrain._drivetrain.reset_pose(
            FlipUtil.fieldPose(
                Pose2d(
                    0.35, kField.WIDTH - 0.35,
                    Rotation2d()
                )
            )
        )
    
    def isFinished(self):
        return True

class SetOdometryBackRight(commands2.Command):
    def __init__(self, drivetrain: drivetrain.SwerveDriveTrain):
        super().__init__()
        self._drivetrain = drivetrain

        self.addRequirements(self._drivetrain)

    def initialize(self):
        self.pose = self._drivetrain.get_state().pose
        self._drivetrain._drivetrain.reset_pose(
            FlipUtil.fieldPose(
                Pose2d(
                    0.35, 0.35,
                    Rotation2d()
                )
            )
        )
    
    def isFinished(self):
        return True
from math import pi

import commands2

from wpimath.geometry import Pose2d, Rotation2d

from util.flip_util import FlipUtil
from util.custom_controller import XboxController

from subsystems.drivetrain import drivetrain

from constants.field import kField

class ControllerDrive(commands2.Command):
    def __init__(self, drivetrain: drivetrain.SwerveDriveTrain, controller: XboxController):
        super().__init__()
        self._drivetrain = drivetrain
        self._controller = controller

        self.addRequirements(self._drivetrain)

    def initialize(self):
        pass

    def execute(self):
        self._drivetrain.drive_from_controller(self._controller)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self._drivetrain.stop()

class InitialPose(commands2.Command):
    def __init__(self, drivetrain: drivetrain.SwerveDriveTrain, pose : Pose2d):
        super().__init__()
        self._drivetrain = drivetrain
        self.pose = pose
        self.addRequirements(self._drivetrain)

    def initialize(self):
        pass

    def execute(self):
        self._drivetrain.reset_pose(
            FlipUtil.fieldPose(
                Pose2d(
                    self.pose.X(),
                    self.pose.Y(),
                    Rotation2d(self.pose.rotation().radians() + pi)
                )
            )
        )

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool):
        pass

class AutoDrive(commands2.Command):
    def __init__(self, drivetrain: drivetrain.SwerveDriveTrain):
        super().__init__()
        self._drivetrain = drivetrain

        self.addRequirements(self._drivetrain)

    def initialize(self):
        pass

    def execute(self):
        self._drivetrain.drive(0, 0, 0)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self._drivetrain.stop()

class OverrideRotation(commands2.Command):
    def __init__(self, drivetrain: drivetrain.SwerveDriveTrain):
        super().__init__()
        self._drivetrain = drivetrain

        self.addRequirements(self._drivetrain)

    def initialize(self):
        self.pose = self._drivetrain.get_state().pose
        self._drivetrain.reset_pose(
            Pose2d(
                self.pose.X(),
                self.pose.Y(),
                Rotation2d()
            )
        )

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool):
        pass

class SetOdometryBackLeft(commands2.Command):
    def __init__(self, drivetrain: drivetrain.SwerveDriveTrain):
        super().__init__()
        self._drivetrain = drivetrain

        self.addRequirements(self._drivetrain)

    def initialize(self):
        self._drivetrain.reset_pose(
            FlipUtil.fieldPose(
                Pose2d(
                    0.35, kField.WIDTH - 0.35,
                    Rotation2d()
                )
            )
        )

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool):
        pass

class SetOdometryBackRight(commands2.Command):
    def __init__(self, drivetrain: drivetrain.SwerveDriveTrain):
        super().__init__()
        self._drivetrain = drivetrain

        self.addRequirements(self._drivetrain)

    def initialize(self):
        self._drivetrain.reset_pose(
            FlipUtil.fieldPose(
                Pose2d(
                    0.35, 0.35,
                    Rotation2d()
                )
            )
        )

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool):
        pass
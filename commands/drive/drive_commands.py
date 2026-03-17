import commands2

from subsystems.drivetrain import drivetrain

from wpimath.geometry import Pose2d
from util.flip_util import FlipUtil

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
        self._drivetrain._drivetrain.reset_pose(FlipUtil.fieldPose(self.pose))

    def isFinished(self):
        return True

class AutoDrive(commands2.Command):
    def __init__(self, drivetrain: drivetrain.SwerveDriveTrain):
        super().__init__()
        self._drivetrain = drivetrain

        self.addRequirements(self._drivetrain)

    def execute(self):
        self._drivetrain.drive_with_values()
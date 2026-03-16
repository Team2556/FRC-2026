from typing import Callable, Tuple

from wpilib import SmartDashboard
from wpimath import applyDeadband
from wpimath.geometry import Pose2d

from commands2 import Command

from util.nt_util import NTTable
from util.custom_controller import XboxController

from subsystems.shooter.shooter_hood import ShooterHood

from constants.shooter import kHoodMotor
from constants.shooter import kShooterData


class ResetShooterHood(Command):
    def __init__(self, shooter_hood: ShooterHood):
        self.shooter_hood = shooter_hood

        self.nt = NTTable("Shooter").get_subtable("Hood")
        self.nt.bool("Resetting")

        self.addRequirements(self.shooter_hood)

    def initialize(self):
        self.shooter_hood.set_speed(kHoodMotor.RESET_HOME_SPEED)

        self.nt.set("Resetting", True)

    def isFinished(self):
        return self.shooter_hood.is_hard_stopped()

    def end(self, interrupted):
        self.shooter_hood.set_speed(0)

        self.nt.set("Resetting", False)


class ManualShooterHood(Command):
    def __init__(self, shooter_hood: ShooterHood, _controller: XboxController,
                 get_pose_and_target: Callable[[], Tuple[Pose2d, Pose2d]]):
        self._shooter_hood = shooter_hood
        self._controller = _controller
        self._get_pose_and_target = get_pose_and_target

        self.addRequirements(self._shooter_hood)

    def execute(self):
        # Deadband prevents hood creep from stick drift _FCC_
        x = applyDeadband(self._controller.getRightX(), 0.2)
        if x != 0:
            self._shooter_hood.increment(x)
        else:
            # Auto-correct hood angle by distance when stick is idle _FCC_
            pose, target = self._get_pose_and_target()
            self._shooter_hood.angle_by_position(pose, target)


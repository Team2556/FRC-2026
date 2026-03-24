from typing import Callable, Tuple

from wpilib import SmartDashboard
from wpimath import applyDeadband
from wpimath.geometry import Pose2d

from commands2 import Command, InterruptionBehavior

from util.nt_util import NTTable
from util.custom_controller import XboxController

from subsystems.shooter.shooter_hood import ShooterHood, HoodStates
from subsystems.drivetrain.drivetrain import SwerveDriveTrain

from constants.shooter import kHoodMotor
from constants.shooter import kShooterData

from util.robot_zone_checker import RobotZoneChecker


class ResetShooterHood(Command):
    def __init__(self, shooter_hood: ShooterHood):
        super().__init__()
        self._hood = shooter_hood

        self.addRequirements(self._hood)

    def initialize(self):
        self._hood.set_state(HoodStates.RESETTING)
        self._hood.set_speed(kHoodMotor.RESET_HOME_SPEED)

    def isFinished(self):
        return self._hood.is_hard_stopped()

    def end(self, interrupted):
        self._hood.set_speed(0)
        self._hood._motor.set_position(kHoodMotor.to_revs(kHoodMotor.HOME_ANGLE_DEG))


# class ManualShooterHood(Command):
#     def __init__(self, shooter_hood: ShooterHood, _controller: XboxController,
#                  get_pose_and_target: Callable[[], Tuple[Pose2d, Pose2d]]):
#         self._shooter_hood = shooter_hood
#         self._controller = _controller
#         self._get_pose_and_target = get_pose_and_target

#         self.addRequirements(self._shooter_hood)

#     def execute(self):
#         # Deadband prevents hood creep from stick drift _FCC_
#         x = applyDeadband(self._controller.getRightX(), 0.2)
#         if x != 0:
#             self._shooter_hood.increment(x)
#         else:
#             # Auto-correct hood angle by distance when stick is idle _FCC_
#             pose, target = self._get_pose_and_target()
#             self._shooter_hood.angle_by_position(pose, target)


class UpdateHoodPositionVariable(Command):
    def __init__(self, shooter_hood: ShooterHood, drivetrain: SwerveDriveTrain):
        super().__init__()
        self.shooter_hood = shooter_hood
        self.drivetrain = drivetrain

        self.addRequirements(self.shooter_hood)

    def execute(self):
        if self.drivetrain.should_stop_shooting():
            self.shooter_hood.set_state(
                HoodStates.HIDE
                
            )


class SetShooterHoodState(Command):
    def __init__(self, shooter_hood: ShooterHood, state: HoodStates):
        super().__init__()

        self._hood = shooter_hood
        self._state = state

        self.addRequirements(self._hood)

    def execute(self):
        self._hood.set_state(self._state)

    def getInterruptionBehavior(self):
        return InterruptionBehavior.kCancelSelf

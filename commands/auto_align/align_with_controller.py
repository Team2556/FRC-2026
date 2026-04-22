import wpilib


import commands2
from commands2 import InterruptionBehavior

from wpimath.geometry import Pose2d

from util.robot_zone_checker import RobotZoneChecker
from util.flip_util import FlipUtil

from subsystems.shooter.dual_shooter import DualMotorShooter
from subsystems.led.LED_controller import CANdleLEDController
from subsystems.shooter.shooter_hood import ShooterHood
from subsystems.trasnfer.transfer_subsystem import TransferSubsystem
from subsystems.drivetrain.drivetrain import SwerveDriveTrain

from commands.auto_align.alignio import RotationCalculator

from constants.field import kHub, kPassSpots
from constants.drive import kAutoAlign
from constants.intake import kIntakeRoller
from constants.shooter import kShooterMotor, kHoodMotor


class ConditionalAlignAndShoot(commands2.Command):
    def __init__(
        self,
        drivetrain: SwerveDriveTrain,
        shooter: DualMotorShooter,
        transfer_subsystem: TransferSubsystem,
        hood: ShooterHood,
    ):
        super().__init__()
        self._drivetrain = drivetrain
        self._shooter = shooter
        self._transfer = transfer_subsystem
        self._hood = hood
        self._calc = RotationCalculator(drivetrain, kHub.POS)

        self.addRequirements(transfer_subsystem, shooter, hood)

    def initialize(self) -> None:
        self._calc.initialize()
        self._shooter.enable()

    def execute(self) -> None:
        self._calc.update_pid()

        drive_state = self._drivetrain.get_state()
        robot_pose = drive_state.pose

        target_pose = self._find_target(robot_pose)

        if target_pose is not None:
            self._calc.with_target(target=target_pose)

        self._drivetrain.set_align_rotation(
            self._calc.calculate_rotation() * kAutoAlign.AUTO_ALIGN_MAX_ANGULAR_RATE
        )

        distance = robot_pose.translation().distance(
            self._calc.leading_target.translation()
        )
        self._hood.set_target_angle(ShooterHood.get_angle_by_distance(distance))
        self._shooter.set_target_rpm(DualMotorShooter.get_rpm_by_distance(distance))

        if (
            self._calc.current_accuracy < kAutoAlign.REQUIRED_SHOOT_ACCURACY_DEGREES
            and self._shooter.is_at_rpm()
            and self._hood.is_at_angle()
        ):
            self._transfer.activate()
        else:
            pass
            # self._transfer.stop()

    def end(self, interrupted: bool) -> None:
        self._drivetrain.clear_align_rotation()
        self._transfer.stop()
        self._hood.set_target_angle(kHoodMotor.HOME_ANGLE_DEG)

    def getInterruptionBehavior(self):
        return InterruptionBehavior.kCancelSelf

    def _find_target(self, pose: Pose2d) -> Pose2d | None:
        """Update the calculator target based on the robot's current zone."""
        if RobotZoneChecker.is_in_hub_shooting_zone(pose):
            return FlipUtil.fieldPose(kHub.POS)
        elif RobotZoneChecker.is_in_left_passing_zone(pose):
            return FlipUtil.fieldPose(kPassSpots.PASS_SPOT_LEFT)
        elif RobotZoneChecker.is_in_right_passing_zone(pose):
            return FlipUtil.fieldPose(kPassSpots.PASS_SPOT_RIGHT)

        return None

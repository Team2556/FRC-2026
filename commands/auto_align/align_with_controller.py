import commands2

from wpimath.geometry import Pose2d

from util.robot_zone_checker import RobotZoneChecker
from util.flip_util import FlipUtil

from subsystems.shooter.dual_shooter import DualMotorShooter
from subsystems.led.LED_controller import CANdleLEDController
from subsystems.shooter.shooter_hood import ShooterHood, HoodStates
from subsystems.trasnfer.transfer_subsystem import TransferSubsystem
from subsystems.drivetrain.drivetrain import SwerveDriveTrain

from commands.auto_align.alignio import RotationCalculator

from constants.field import kHub, kPassSpots
from constants.drive import kAutoAlign
from constants.shooter import kShooterMotor


class ConditionalAlignAndShoot(commands2.Command):
    """
    Aligns the robot toward the best shooting target and fires once ready.

    Target selection (checked each execute loop):
      - Left passing zone  → left pass spot
      - Right passing zone → right pass spot
      - Alliance zone      → hub

    Rotation is computed by a :class:`RotationCalculator` helper; this command
    does not use any multi-layer inheritance.
    """

    def __init__(
        self,
        drivetrain : SwerveDriveTrain,
        shooter: DualMotorShooter,
        transfer_subsystem: TransferSubsystem,
        hood: ShooterHood,
        LED_controller: CANdleLEDController | None = None,
    ):
        super().__init__()
        self._drivetrain = drivetrain
        self._shooter = shooter
        self._transfer = transfer_subsystem
        self._hood = hood
        self._calc = RotationCalculator(drivetrain, kHub.POS)

        # Only own transfer — drivetrain is influenced via the set_align_rotation()
        # overlay, not direct control, so it must remain free for AutoDrive (or the
        # default drive command) to own simultaneously.
        self.addRequirements(transfer_subsystem)

    def initialize(self) -> None:
        self._calc.initialize()
        self._hood.set_state(HoodStates.AUTO)
        self._shooter.enable()

    def execute(self) -> None:
        self._calc.update_pid()

        robot_pose = self._drivetrain.get_state().pose
        self._find_target(robot_pose)

        rotation_rate = self._calc.calculate_rotation()
        self._drivetrain.set_align_rotation(
            rotation_rate * kAutoAlign.AUTO_ALIGN_MAX_ANGULAR_RATE
        )

        self._hood.add_auto_hood_measurement(
            self._drivetrain.get_state(), self._calc.target
        )

        # Don't fire until BOTH yaw and shooter speed are on target
        if (
            self._calc.current_accuracy < kAutoAlign.REQUIRED_SHOOT_ACCURACY_DEGREES
            and self._shooter.is_charged
        ):
            self._transfer.activate()

        if RobotZoneChecker.is_in_opposing_alliance_zone(robot_pose):
            kShooterMotor.CURRENT_TARGET_RPM = kShooterMotor.TARGET_RPM_FAR
        else:
            kShooterMotor.CURRENT_TARGET_RPM = kShooterMotor.TARGET_RPM

    # def isFinished(self) -> bool:
    #     return self._drivetrain.should_stop_shooting()

    def end(self, interrupted: bool) -> None:
        print("-----------------------------------------------------------------------------")
        print("-----------------------------------------------------------------------------")
        print("-----------------------------------------------------------------------------")
        print("-----------------------------------------------------------------------------")
        print("-----------------------------------------------------------------------------")
        print("-----------------------------------------------------------------------------")
        print("-----------------------------------------------------------------------------")
        print("-----------------------------------------------------------------------------")
        print("-----------------------------------------------------------------------------")
        print("-----------------------------------------------------------------------------")
        print("-----------------------------------------------------------------------------")
        print("-----------------------------------------------------------------------------")
        self._drivetrain.clear_align_rotation()
        self._transfer.stop()

    def _find_target(self, pose: Pose2d) -> None:
        """Update the calculator target based on the robot's current zone."""
        if RobotZoneChecker.is_in_left_passing_zone(pose):
            self._calc.with_target(FlipUtil.fieldPose(kPassSpots.PASS_SPOT_LEFT))
        if RobotZoneChecker.is_in_right_passing_zone(pose):
            self._calc.with_target(FlipUtil.fieldPose(kPassSpots.PASS_SPOT_RIGHT))
        if RobotZoneChecker.is_in_hub_shooting_zone(pose):
            self._calc.with_target(FlipUtil.fieldPose(kHub.POS))

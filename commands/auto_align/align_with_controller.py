import wpilib
import commands2

from wpimath.geometry import Pose2d

from util.robot_zone_checker import RobotZoneChecker
from util.flip_util import FlipUtil

from subsystems.shooter.dual_shooter import DualMotorShooter
from subsystems.led.LED_controller import CANdleLEDController
from subsystems.shooter.shooter_hood import ShooterHood, HoodStates
from subsystems.trasnfer.transfer_subsystem import TransferSubsystem
from subsystems.intake.intake import IntakeSubsystem

from commands.auto_align.alignio import RotationCalculator

from constants.field import kHub, kPassSpots
from constants.drive import kAutoAlign
from constants.intake import kIntakeRoller


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

    # Intake oscillation timing constants
    _OSCILLATE_PERIOD = 1.0    # seconds per cycle
    _INTAKE_DURATION  = 0.8    # seconds of intake (forward) per cycle
    # Remaining 0.9 s per cycle = eject (reverse) to free stuck balls

    def __init__(
        self,
        drivetrain,
        shooter: DualMotorShooter,
        transfer_subsystem: TransferSubsystem,
        hood: ShooterHood,
        intake_subsystem: IntakeSubsystem,
        LED_controller: CANdleLEDController | None = None,
    ):
        super().__init__()
        self._drivetrain = drivetrain
        self._shooter = shooter
        self._transfer = transfer_subsystem
        self._hood = hood
        self._intake = intake_subsystem
        self._calc = RotationCalculator(drivetrain, kHub.POS)

        # Own transfer and intake.  Drivetrain is influenced via the
        # set_align_rotation() overlay so it stays free for the drive command.
        self.addRequirements(transfer_subsystem, intake_subsystem)

    def initialize(self) -> None:
        self._calc.initialize()
        self._hood.set_state(HoodStates.AUTO)
        self._shooter.enable()

    def execute(self) -> None:
        self._calc.update_pid()

        drive_state = self._drivetrain.get_state()
        robot_pose = drive_state.pose
        self._find_target(robot_pose)

        rotation_rate = self._calc.calculate_rotation()
        self._drivetrain.set_align_rotation(
            rotation_rate * kAutoAlign.AUTO_ALIGN_MAX_ANGULAR_RATE
        )

        # Update hood angle and shooter RPM from distance interpolation tables
        self._hood.add_auto_hood_measurement(drive_state, self._calc.leading_target)
        self._shooter.add_auto_rpm_measurement(robot_pose, self._calc.leading_target)

        phase = wpilib.Timer.getFPGATimestamp() % self._OSCILLATE_PERIOD
        if phase < self._INTAKE_DURATION:
            self._intake.set_roller_speed(kIntakeRoller.TARGET_RPM)
        else:
            self._intake.set_roller_speed(kIntakeRoller.TARGET_RPM * -0.2)

        # Don't fire until BOTH yaw and shooter speed are on target
        if (
            self._calc.current_accuracy < kAutoAlign.REQUIRED_SHOOT_ACCURACY_DEGREES
            and self._shooter.is_charged
        ):
            self._transfer.activate()

    def isFinished(self) -> bool:
        return self._drivetrain.should_stop_shooting()

    def end(self, interrupted: bool) -> None:
        self._drivetrain.clear_align_rotation()
        self._shooter.clear_auto_target()
        self._intake.stop_roller()
        self._transfer.stop()

    def _find_target(self, pose: Pose2d) -> None:
        """Update the calculator target based on the robot's current zone."""
        if RobotZoneChecker.is_in_left_passing_zone(pose):
            self._calc.with_target(FlipUtil.fieldPose(kPassSpots.PASS_SPOT_LEFT))
        if RobotZoneChecker.is_in_right_passing_zone(pose):
            self._calc.with_target(FlipUtil.fieldPose(kPassSpots.PASS_SPOT_RIGHT))
        if RobotZoneChecker.is_in_alliance_zone(pose):
            self._calc.with_target(FlipUtil.fieldPose(kHub.POS))

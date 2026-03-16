from wpilib import SmartDashboard
from wpimath.geometry import Pose2d

from constants.field import kHub, kPassSpots
from constants.drive import kAutoAlign, kDriveConfig

from util.robot_zone_checker import RobotZoneChecker
from util.flip_util import FlipUtil

from subsystems.shooter.dual_shooter import DualMotorShooter
from subsystems.led.LED_controller import CANdleLEDController
from subsystems.led.LED_helpers import ColorFactories, CANdle_Color
from subsystems.shooter.shooter_hood import ShooterHood
from subsystems.trasnfer.transfer_subsystem import TransferSubsystem

from commands.auto_align import alignio

from commands.shooter.hood_commands import ResetShooterHood


class TurretToPose(alignio.TurretTargetWithVelocity):
    def __init__(
        self,
        drivetrain,
        target: Pose2d,
        shooter: DualMotorShooter,
    ):
        super().__init__(drivetrain, shooter, target)

    def execute(self):
        rotation_rate = self.calculate_rotation()
        self._drivetrain.set_target_align_rotation_rate(
            rotation_rate
            * kAutoAlign.ROBOT_VELOCITY_MULT
            * kDriveConfig.MAX_ANGULAR_RATE
        )

    def end(self, interrupted):
        self._drivetrain.stop_target_align()


class HubAlign(alignio.TurretTargetWithVelocity):
    def __init__(
        self,
        drivetrain,
        shooter: DualMotorShooter,
        hood: ShooterHood,
        LED_Controller: CANdleLEDController | None = None,
    ):
        super().__init__(drivetrain, shooter, kHub.POS)
        self._hood = hood

        SmartDashboard.putNumber(
            "Hub Align Flight Time Scalar", self.flight_time_scalar
        )

        self._LED_Controller = LED_Controller
        if self._LED_Controller:
            self.LED_state = self._LED_Controller.create_state(
                "hub_align", ColorFactories.solid_color(CANdle_Color.GREEN), 10, False
            )

        # Doesn't need requirement because to only modifies the drivetrain's override_rotation
        # self.addRequirements(drivetrain)

    def initialize(self):
        if self._LED_Controller:
            self.LED_state.enable()

    def execute(self):
        self.flight_time_scalar = SmartDashboard.getNumber(
            "Hub Align Flight Time Scalar", self.flight_time_scalar
        )

        rotation_rate = self.calculate_rotation()
        self._drivetrain.set_target_align_rotation_rate(
            rotation_rate * kDriveConfig.MAX_ANGULAR_RATE
        )

        self._hood.angle_by_position(
            self._drivetrain.get_state().pose, self._estimated_target_pose
        )

    def end(self, interrupted):
        if self._LED_Controller:
            self.LED_state.disable()
        self._drivetrain.stop_target_align()


class ConditionalAlignAndShoot(HubAlign):
    """Align to spot with velocity command that targets the best spot to shoot automatically depending on position"""

    def __init__(
        self,
        drivetrain,
        shooter: DualMotorShooter,
        transfer_subsystem: TransferSubsystem,
        hood: ShooterHood,
        LED_controller: CANdleLEDController | None = None,
    ):
        super().__init__(drivetrain, shooter, hood, LED_controller)
        
        self._hood = hood
        self.transfer_subsystem = transfer_subsystem

    def initialize(self):
        super().initialize()
        self._shooter.enable()

    def execute(self):
        super().execute()
        robot_pose = self._drivetrain.get_state().pose

        self.find_target(robot_pose)

        # Don't fire until BOTH yaw and hood angle are on target _FCC_
        if (
            self.current_accuracy < kAutoAlign.REQUIRED_SHOOT_ACCURACY_DEGREES
            and self._hood.is_at_angle()
        ):
            self.transfer_subsystem.activate()
        else:
            self.transfer_subsystem.stop()

    def find_target(self, pose):
        if RobotZoneChecker.is_in_left_neutral_zone(pose):
            self.target = FlipUtil.fieldPose(kPassSpots.PASS_SPOT_LEFT)
        if RobotZoneChecker.is_in_right_neutral_zone(pose):
            self.target = FlipUtil.fieldPose(kPassSpots.PASS_SPOT_RIGHT)
        if RobotZoneChecker.is_in_alliance_zone(pose):
            self.target = FlipUtil.fieldPose(kHub.POS)
        self.with_target(self.target)

    def isFinished(self):
        return self._drivetrain.should_stop_shooting()

    def end(self, interrupted):
        super().end(interrupted)

        ResetShooterHood(self._hood).schedule()
        self.transfer_subsystem.stop()
        self._shooter.disable()

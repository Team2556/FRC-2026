from wpimath.geometry import Pose2d

from util import custom_controller
from util.robot_zone_checker import RobotZoneChecker
from util.flip_util import FlipUtil

from subsystems.shooter.dual_shooter import DualMotorShooter
from subsystems.led.LED_controller import CANdleLEDController
from subsystems.led.LED_helpers import ColorFactories, CANdle_Color
from subsystems.shooter.shooter_hood import ShooterHood, HoodStates
from subsystems.trasnfer.transfer_subsystem import TransferSubsystem

from commands.auto_align import alignio
from commands.shooter.hood_commands import ResetShooterHood

from constants.field import kHub, kPassSpots
from constants.drive import kAutoAlign, kDriveConfig
from constants.shooter import kShooterMotor


class TurretToPose(alignio.TurretTargetWithVelocity):
    def __init__(
        self,
        drivetrain,
        target: Pose2d,
        shooter: DualMotorShooter,
    ):
        super().__init__(drivetrain, target)
        self._shooter = shooter

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
        super().__init__(drivetrain, kHub.POS)
        self._hood = hood
        self._shooter = shooter

    def initialize(self):
        self._hood.set_state(HoodStates.AUTO)

    def execute(self):
        rotation_rate = self.calculate_rotation()
        self._drivetrain.set_target_align_rotation_rate(
            rotation_rate * kAutoAlign.AUTO_ALIGN_MAX_ANGULAR_RATE
        )

        self._hood.add_auto_hood_measurement(self._drivetrain.get_state(), self.target)

    def end(self, interrupted):
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
        self.transfer_subsystem = transfer_subsystem
        self.addRequirements(transfer_subsystem)

    def initialize(self):
        super().initialize()
        self._shooter.enable()

    def execute(self):
        self.periodic()
        super().execute()
        robot_pose = self._drivetrain.get_state().pose

        self.find_target(robot_pose)

        # Don't fire until BOTH yaw and hood angle are on target _FCC_
        if (
            self.current_accuracy < kAutoAlign.REQUIRED_SHOOT_ACCURACY_DEGREES
            # and self._hood.is_at_angle() bad
            and self._shooter.is_charged
        ):
            self.transfer_subsystem.activate()
        # else:
        #     self.transfer_subsystem.stop()
        
        if RobotZoneChecker.is_in_opposing_alliance_zone(self._drivetrain.get_state().pose):
            kShooterMotor.CURRENT_TARGET_RPM = kShooterMotor.TARGET_RPM_FAR
        else:
            kShooterMotor.CURRENT_TARGET_RPM = kShooterMotor.TARGET_RPM

    def find_target(self, pose):
        if RobotZoneChecker.is_in_left_passing_zone(pose):
            self.target = FlipUtil.fieldPose(kPassSpots.PASS_SPOT_LEFT)
        if RobotZoneChecker.is_in_right_passing_zone(pose):
            self.target = FlipUtil.fieldPose(kPassSpots.PASS_SPOT_RIGHT)
        if RobotZoneChecker.is_in_alliance_zone(pose):
            self.target = FlipUtil.fieldPose(kHub.POS)
        self.with_target(self.target)

    def isFinished(self):
        return self._drivetrain.should_stop_shooting()

    def end(self, interrupted):
        super().end(interrupted)

        self.transfer_subsystem.stop()
        # self._shooter.disable()

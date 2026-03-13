from wpilib import SmartDashboard
from wpimath.geometry import Pose2d

from constants.field import kHub, kPassSpots
from constants.drive import kAutoAlign, kDriveConfig

from util import custom_controller
from util.robot_zone_checker import RobotZoneChecker
from util.flip_util import FlipUtil

from subsystems.controlled_motor import ControlledTalonMotor
from subsystems.led.LED_controller import CANdleLEDController
from subsystems.led.LED_helpers import ColorFactories, CANdle_Color

from commands.auto_align import alignio


class TurretToPose(alignio.TurretTargeWithVelocity):
    def __init__(
        self,
        drivetrain,
        controller,
        target: Pose2d,
        shooter: ControlledTalonMotor,
        hood: None,
    ):
        super().__init__(drivetrain, shooter, hood, target)
        self._controller = controller

    def execute(self):
        rotation_rate = self.calculate_rotation()
        self._drivetrain.set_target_align_rotation_rate(
            rotation_rate
            * kAutoAlign.ROBOT_VELOCITY_MULT
            * kDriveConfig.MAX_ANGULAR_RATE
        )

    def end(self, interrupted):
        self._drivetrain.stop_target_align()

class HubAlign(alignio.TurretTargeWithVelocity):
    def __init__(
        self,
        drivetrain,
        controller: custom_controller.XboxController,
        shooter: ControlledTalonMotor,
        hood,
        LED_Controller: CANdleLEDController | None = None,
    ):
        super().__init__(drivetrain, shooter, hood, kHub.POS)
        self._controller = controller

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

    def end(self, interrupted):
        if self._LED_Controller:
            self.LED_state.disable()
        self._drivetrain.stop_target_align()


class ConditionalAlignAndShoot(HubAlign):
    """Align to spot with velocity command that targets the best spot to shoot automatically depending on position"""

    def __init__(
        self,
        drivetrain,
        controller: custom_controller.XboxController,
        shooter: ControlledTalonMotor,
        spindex: ControlledTalonMotor,
        transfer1: ControlledTalonMotor,
        transfer2: ControlledTalonMotor,
        hood,
        LED_controller: CANdleLEDController | None = None,
    ):

        super().__init__(drivetrain, controller, shooter, hood, LED_controller)

        self._spindex = spindex
        self._transfer1 = transfer1
        self._transfer2 = transfer2

    def initialize(self):
        super().initialize()

    def execute(self):
        self.find_target(self._drivetrain.get_state().pose)
        super().execute()
        if self.current_accuracy < kAutoAlign.REQUIRED_SHOOT_ACCURACY_DEGREES:
            self.activate_shooter()
        else:
            self.stop_shooter()

    def find_target(self, pose):
        if RobotZoneChecker.is_in_left_neutral_zone(pose):
            self.target = FlipUtil.fieldPose(kPassSpots.PASS_SPOT_LEFT)
        if RobotZoneChecker.is_in_right_neutral_zone(pose):
            self.target = FlipUtil.fieldPose(kPassSpots.PASS_SPOT_RIGHT)
        if RobotZoneChecker.is_in_alliance_zone(pose):
            self.target = FlipUtil.fieldPose(kHub.POS)
        self.with_target(self.target)

    def end(self, interrupted):
        super().end(interrupted)
        self.stop_shooter()

    def activate_shooter(self):
        self._spindex.spin()
        self._transfer1.spin()
        self._transfer2.spin()
        self._shooter.spin()

    def stop_shooter(self):
        self._spindex.stop_motor()
        self._transfer1.stop_motor()
        self._transfer2.stop_motor()
        self._shooter.stop_motor()

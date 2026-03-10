from wpilib import SmartDashboard
from wpimath.geometry import Pose2d

from constants.field import kHub, kPassSpots
from constants.drive import kAutoAlign
from constants.drive import kDriveConfig

from util import custom_controller
from util.robot_zone_checker import RobotZoneChecker
from util.flip_util import FlipUtil

from subsystems.shooter.controlled_motor import ControlledMotor
from subsystems.controlled_motor import ControlledTalonMotor

from commands.auto_align import alignio


class TurretToPose(alignio.TurretTargeWithVelocity):
    def __init__(self, drivetrain, controller, target: Pose2d, shooter: ControlledTalonMotor, hood: None):
        super().__init__(drivetrain, shooter, hood, target)
        self._controller = controller

    def execute(self):
        rotation_rate = self.calculate_rotation()
        self._drivetrain.set_target_align_rotation_rate(
            rotation_rate * kAutoAlign.ROBOT_VELOCITY_MULT * kDriveConfig.MAX_ANGULAR_RATE
        )
    
    def end(self, interrupted):
        self._drivetrain.stop_target_align()

class HubAlign(alignio.TurretTargeWithVelocity):
    def __init__(self, drivetrain, controller : custom_controller.XboxController, shooter: ControlledMotor, hood):
        super().__init__(drivetrain, shooter, hood, kHub.POS)
        self._controller = controller

        SmartDashboard.putNumber(
            "Hub Align Flight Time Scalar", self.flight_time_scalar
        )

        # Doesn't need requirement because to only modifies the drivetrain's override_rotation
        # self.addRequirements(drivetrain)

    def execute(self):
        self.flight_time_scalar = SmartDashboard.getNumber(
            "Hub Align Flight Time Scalar", self.flight_time_scalar
        )
        
        rotation_rate = self.calculate_rotation()
        self._drivetrain.set_target_align_rotation_rate(
            rotation_rate * kDriveConfig.MAX_ANGULAR_RATE
        )
    
    def end(self, interrupted):
        self._drivetrain.stop_target_align()

class ConditionalAlign(HubAlign):
    def __init__(self, drivetrain, controller : custom_controller.XboxController, shooter: ControlledMotor, hood):
        super().__init__(drivetrain, controller, shooter, hood)

    def execute(self):
        self.find_target(self._drivetrain.get_state().pose)
        super().execute()
    
    def find_target(self, pose):
        if RobotZoneChecker.is_in_left_neutral_zone(pose):
            self.target = FlipUtil.fieldPose(kPassSpots.PASS_SPOT_LEFT)
        if RobotZoneChecker.is_in_right_neutral_zone(pose):
            self.target = FlipUtil.fieldPose(kPassSpots.PASS_SPOT_RIGHT)
        if RobotZoneChecker.is_in_alliance_zone(pose):
            self.target = FlipUtil.fieldPose(kHub.POS)
        self.with_target(self.target)
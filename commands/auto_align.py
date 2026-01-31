import commands2
from wpilib import SmartDashboard
from wpimath.geometry import Translation2d
import math

from subsystems import limelight_camera, drivetrain

from constants._configs import kShooter, kField
from constants.tuners import kAutoAlign

from _utils import custom_controller, math_helpers


class StationaryAlign(commands2.Command):
    def __init__(
        self,
        limelight: limelight_camera.LimelightCamera,
        drivetrain: drivetrain.SwerveDriveTrain,
    ):
        super().__init__()
        self._limelight = limelight
        self._drivetrain = drivetrain

        self._limelight.setPipeline(1)

        self.max_x = 38
        self.accuracy = 0.1
        SmartDashboard.putNumber("Rotation Accuracy", self.accuracy)

        self.speed_mult = 0.5
        SmartDashboard.putNumber("Rotation Mult", self.speed_mult)

        self.addRequirements(self._drivetrain)

    def execute(self):
        self.speed_mult = SmartDashboard.getNumber("Rotation Mult", -0.1)
        self.accuracy = SmartDashboard.getNumber("Rotation Accuracy", 0.1)

        x = self._limelight.getX()
        if abs(x) < self.accuracy:
            x = 0

        x_normal = x / self.max_x

        SmartDashboard.putNumber("April Tag X", x)

        turn_speed = x_normal * self.speed_mult
        self._drivetrain.drive_with_values(rotation_rate=turn_speed)

    def end(self, interrupted):
        self._drivetrain._stop()


class MobileAlign(commands2.Command):
    def __init__(
        self,
        limelight: limelight_camera.LimelightCamera,
        drivetrain: drivetrain.SwerveDriveTrain,
        controller: custom_controller.XboxController,
    ):
        super().__init__()
        self._limelight = limelight
        self._drivetrain = drivetrain
        self._controller = controller

        self._limelight.setPipeline(1)

        self.max_x = 38
        self.accuracy = 0.05
        SmartDashboard.putNumber("Rotation Accuracy", self.accuracy)

        self.speed_mult = 0.5
        SmartDashboard.putNumber("Rotation Mult", self.speed_mult)

        self.addRequirements(self._drivetrain)

    def execute(self):
        self.speed_mult = SmartDashboard.getNumber("Rotation Mult", -0.1)
        self.accuracy = SmartDashboard.getNumber("Rotation Accuracy", 0.1)

        x = self._limelight.getX()
        if abs(x) < self.accuracy:
            x = 0

        x_normal = x / self.max_x

        SmartDashboard.putNumber("April Tag X", x)

        turn_speed = x_normal * self.speed_mult
        self._drivetrain.drive_with_controller(
            self._controller, rotation_rate=turn_speed
        )

    def end(self, interrupted):
        self._drivetrain._stop()


class HubAlign(commands2.Command):
    def __init__(
        self,
        drivetrain: drivetrain.SwerveDriveTrain,
        controller: custom_controller.XboxController,
    ):
        self._drivetrain = drivetrain
        self._controller = controller

        self.shooter_offset = kShooter.SHOOTER_OFFSET
        SmartDashboard.putNumber("Shooter/Offset/X", self.shooter_offset.x)
        SmartDashboard.putNumber("Shooter/Offset/Y", self.shooter_offset.y)

        self.shooter_direction = kShooter.SHOOTER_DIRECTION
        self.shooter_direction_tuner = kAutoAlign.DIRECTION_TUNING
        SmartDashboard.putNumber("Shooter/Direction/Tune", self.shooter_direction_tuner)
        
        self.hub_pos = kField.RED_HUB_POS  # Translation2d

        # PID : PIDController(5.0 / 180.0, 0, 0.1)
        self.rotation_PID = kAutoAlign.ROTATIONAL_PID
        self.rotation_PID.enableContinuousInput(-180.0, 180.0)

        self.addRequirements(drivetrain)

    def calculate_target_yaw(self, robot_pose):
        shooter_field_pos = robot_pose.translation() + self.shooter_offset.rotateBy(
            robot_pose.rotation()
        )
        to_hub = self.hub_pos - shooter_field_pos

        return math.degrees(math.atan2(to_hub.Y(), to_hub.X()))

    def execute(self):
        self.shooter_offset = Translation2d(
            SmartDashboard.getNumber("Shooter/Offset/Y", self.shooter_offset.y),
            SmartDashboard.getNumber("Shooter/Offset/X", self.shooter_offset.x),
        )
        self.shooter_direction_tuner = SmartDashboard.getNumber("Shooter/Direction/Tune", self.shooter_direction_tuner)
        SmartDashboard.putNumberArray('Shooter/Offset/Array', [
            kShooter.SHOOTER_OFFSET.X(), kShooter.SHOOTER_OFFSET.Y()
        ])

        robot_state = self._drivetrain._drivetrain.get_state()
        robot_pose = robot_state.pose

        robot_heading = robot_pose.rotation().degrees()
        target_heading = self.calculate_target_yaw(robot_pose) - self.shooter_direction - self.shooter_direction_tuner

        rotation_rate = self.rotation_PID.calculate(robot_heading, target_heading)

        heading_error = self.rotation_PID.getError()

        SmartDashboard.putNumber("HubAlign/TargetHeading", target_heading)
        SmartDashboard.putNumber("HubAlign/RobotHeading", robot_heading)
        SmartDashboard.putNumber("HubAlign/HeadingError", heading_error)
        SmartDashboard.putNumber("HubAlign/RotationRate", rotation_rate)

        rotation_rate = math_helpers.clamp(rotation_rate, -1.0, 1.0)

        self._drivetrain.drive_with_controller(
            self._controller,
            rotation_rate=rotation_rate,
            velocity_mult=kAutoAlign.ROBOT_VELOCITY_MULT,
        )

import commands2

from wpilib import SmartDashboard

from phoenix6 import swerve

from subsystems import limelight_camera, drivetrain

from _utils import custom_controller

import math

from wpimath.controller import PIDController


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
    def __init__(self, drivetrain: drivetrain.SwerveDriveTrain, controller):
        self._drivetrain = drivetrain
        self._controller = controller

        self.shooter_offset_x = 0
        self.shooter_offset_y = 0
        self.shooter_direction = -90

        self.hub_pos_x = 12
        self.hub_pos_y = 4

        self.rotation_PID = PIDController(4, 0, 0.1)
        self.rotation_PID.enableContinuousInput(-1, 1)
        
        self.accuracy = 0
        
        self.addRequirements(drivetrain)

    def calculate_target_yaw(self):
        robot_state = self._drivetrain._drivetrain.get_state()
        robot_pos = robot_state.pose
        robot_x = robot_pos.X()
        robot_y = robot_pos.Y()

        dx = self.hub_pos_x - robot_x
        dy = self.hub_pos_y - robot_y

        target_yaw_rad = math.atan2(dy, dx)
        target_yaw_deg = math.degrees(target_yaw_rad)
        # target_yaw_deg += self.shooter_offset_x

        return target_yaw_deg

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def execute(self):
        robot_state = self._drivetrain._drivetrain.get_state()
        robot_pos = robot_state.pose

        robot_heading = float(robot_pos.rotation().degrees()) + self.shooter_direction
        target_heading = self.calculate_target_yaw()

        d_heading = robot_heading - target_heading
        normal_d_heading = d_heading / 180
        
        rotation_rate = self.rotation_PID.calculate(normal_d_heading, 0.0)
        
        SmartDashboard.putNumber("D HEading", d_heading)
        SmartDashboard.putNumber("Rotation Rate to Hub", rotation_rate)
        
        if abs(d_heading) < self.accuracy:
            rotation_rate = 0
            
        rotation_rate = self.clamp(rotation_rate, -1, 1)
            
        # rotation_rate = self.clamp(rotation_rate, -1, 1)

        self._drivetrain.drive_with_controller(
            self._controller, rotation_rate=rotation_rate, velocity_mult = 0.5
        )

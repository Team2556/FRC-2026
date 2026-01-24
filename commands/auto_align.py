import commands2

from wpilib import SmartDashboard

from phoenix6 import swerve

from subsystems import limelight_camera, drivetrain

from _utils import custom_controller


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
        self._drivetrain.drive_with_controller(self._controller, rotation_rate=turn_speed)

    def end(self, interrupted):
        self._drivetrain._stop()

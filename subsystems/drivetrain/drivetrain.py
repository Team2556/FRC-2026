import commands2

import wpilib
from wpimath.units import rotationsToRadians

from phoenix6 import swerve

from subsystems.drivetrain.swerve_tuner import TunerConstants
from subsystems.drivetrain.telemetry import Telemetry

from constants.drive import kDriveConfig

from .driveio import CustomSwerve


class SwerveDriveTrain(commands2.Subsystem):
    def __init__(self):
        self._drivetrain = TunerConstants.create_drivetrain()

        self._drive = swerve.requests.FieldCentric().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        self._logger = Telemetry(kDriveConfig.MAX_SPEED)
        self._drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

        self._field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self._field)

    def drive_with_controller(
        self,
        controller,
        velocity_x=None,
        velocity_y=None,
        rotation_rate=None,
        velocity_mult=1,
        rotation_mult=1,
    ):
        _velocity_x = velocity_x if velocity_x else -controller.getLeftY()
        _velocity_y = velocity_y if velocity_y else -controller.getLeftX()
        _rotational_rate = rotation_rate if rotation_rate else -controller.getRightX()

        self._drivetrain.set_control(
            self._drive.with_velocity_x(
                _velocity_x * kDriveConfig.MAX_SPEED * velocity_mult * TunerConstants.speed_at_12_volts
            )  # Drive forward with negative Y (forward)
            .with_velocity_y(
                _velocity_y * kDriveConfig.MAX_SPEED * velocity_mult * TunerConstants.speed_at_12_volts
            )  # Drive left with negative X (left)
            .with_rotational_rate(
                _rotational_rate * kDriveConfig.MAX_ANGULAR_RATE * rotation_mult
            )  # Drive counterclockwise with negative X (left)
        )

    def drive_with_values(self, velocity_x=0, velocity_y=0, rotation_rate=0):
        # This version of drive_with_values is the better version (if you're merging right now)
        self._drivetrain.set_control(
            self._drive.with_velocity_x(velocity_x)
            .with_velocity_y(velocity_y)
            .with_rotational_rate(rotation_rate)
        )

    def _stop(self):
        self.drive_with_values(velocity_x=0, velocity_y=0, rotation_rate=0)

    def get_state(self) -> CustomSwerve.DriveState:
        return CustomSwerve.BuildDriveState(self._drivetrain.get_state())
    
    def seed_field_centric(self):
        return self._drivetrain.seed_field_centric()
    
    def periodic(self):
        self._field.setRobotPose(self.get_state().pose)
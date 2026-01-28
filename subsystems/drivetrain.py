from constants.swerve_tuner import TunerConstants
from _utils.telemetry import Telemetry

import commands2
from wpimath.units import rotationsToRadians
from phoenix6 import swerve


class SwerveDriveTrain(commands2.Subsystem):
    def __init__(self):
        self._drivetrain = TunerConstants.create_drivetrain()

        self._max_speed = (
            1.0 * TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.75
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(
                self._max_angular_rate * 0.1
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        self._logger = Telemetry(self._max_speed)

    def drive_with_controller(
        self, controller, velocity_x=None, velocity_y=None, rotation_rate=None
    ):
        _velocity_x = velocity_x if velocity_x else -controller.getLeftY()
        _velocity_y = velocity_y if velocity_y else -controller.getLeftX()
        _rotational_rate = rotation_rate if rotation_rate else -controller.getRightX()

        self._drivetrain.set_control(
            self._drive.with_velocity_x(
                _velocity_x * self._max_speed
            )  # Drive forward with negative Y (forward)
            .with_velocity_y(
                _velocity_y * self._max_speed
            )  # Drive left with negative X (left)
            .with_rotational_rate(
                _rotational_rate * self._max_angular_rate
            )  # Drive counterclockwise with negative X (left)
        )

    def drive_with_values(self, velocity_x=0, velocity_y=0, rotation_rate=0):
        self._drivetrain.set_control(
            self._drive.with_velocity_x(
                -velocity_x * self._max_speed
            )  # Drive forward with negative Y (forward)
            .with_velocity_y(
                -velocity_y * self._max_speed
            )  # Drive left with negative X (left)
            .with_rotational_rate(
                -rotation_rate * self._max_angular_rate
            )  # Drive counterclockwise with negative X (left)
        )

    def _stop(self):
        self.drive_with_values(velocity_x=0, velocity_y=0, rotation_rate=0)

    def _add_vision_measurements(self, vision_robot_pose, timestamp):
        self._drivetrain.add_vision_measurement(
            vision_robot_pose=vision_robot_pose, timestamp=timestamp
        )

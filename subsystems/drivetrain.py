from constants.swerve_tuner import TunerConstants
from _utils.telemetry import Telemetry
from subsystems.vision.visionsubsystem import VisionObservation
from phoenix6.swerve.swerve_drivetrain import SwerveDrivetrain
import commands2
from wpimath.units import rotationsToRadians
from wpimath.interpolation import TimeInterpolatablePose2dBuffer
from wpimath.geometry import Pose2d, Transform2d, Rotation2d
from phoenix6 import swerve
import wpilib

from math import sqrt


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
            swerve.requests.FieldCentric().with_deadband(self._max_speed * 0.1)
            # .with_rotational_deadband(
            #     self._max_angular_rate * 0.1
            # )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        self._logger = Telemetry(self._max_speed)

        self.poseBuffer = TimeInterpolatablePose2dBuffer(2.0)

        self._field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self._field)
        
        self.odoStdDevs = (0.1, 0.1, 0.1)
        
    def add_odometry_measurement(self, timestamp, pose: Pose2d):
        self.poseBuffer.addSample(timestamp, pose)

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
                _velocity_x * self._max_speed * velocity_mult
            )  # Drive forward with negative Y (forward)
            .with_velocity_y(
                _velocity_y * self._max_speed * velocity_mult
            )  # Drive left with negative X (left)
            .with_rotational_rate(
                _rotational_rate * self._max_angular_rate * rotation_mult
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

    def _add_vision_measurements(self, measurement: VisionObservation):
        if self.poseBuffer.getInternalBuffer()[-1][0] - 2.0 > measurement.timestamp:
            return

        sample = self.poseBuffer.sample(measurement.timestamp)
        if sample is None:
            return

        odometry_pose = self.get_robot_pose()

        sampleToOdometryTransform = Transform2d(sample, odometry_pose)
        odometryToSampleTransform = Transform2d(odometry_pose, sample)

        estimateAtTime = odometry_pose + odometryToSampleTransform

        # new vision matrix
        r = [i * i for i in measurement.std]

        # Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        # and C = I. See wpimath/algorithms.md.
        visionK = [0.0, 0.0, 0.0]

        for i in range(3):
            stdDev = self.odoStdDevs[i]
            if stdDev == 0.0:
                visionK[i] = 0.0
            else:
                visionK[i] = stdDev / (stdDev + sqrt(stdDev * r[i]))

        transform = Transform2d(estimateAtTime, measurement.visionPose)
        kTimesTransform = [
            visionK[i] * k
            for i, k in enumerate(
                [transform.X(), transform.Y(), transform.rotation().radians()]
            )
        ]

        scaledTransform = Transform2d(
            kTimesTransform[0], kTimesTransform[1], kTimesTransform[2]
        )

        estimatedPose = estimateAtTime + scaledTransform + sampleToOdometryTransform

        self._drivetrain.add_vision_measurement(
            vision_robot_pose=estimatedPose, timestamp=measurement.timestamp
        )

    def get_robot_state(self) -> SwerveDrivetrain.SwerveDriveState:
        return self._drivetrain.get_state()

    def get_robot_pose(self) -> Pose2d:
        robot_state = self.get_robot_state()
        return robot_state.pose
    
    def get_robot_rotation(self) -> Rotation2d:
        return self.get_robot_pose().rotation()

    def periodic(self):
        drive_state = self.get_robot_state()
        self._field.setRobotPose(drive_state.pose)
        self.add_odometry_measurement(drive_state.timestamp, drive_state.pose)

import math

import commands2

from wpimath import units
from wpilib import SmartDashboard

from phoenix6.swerve.swerve_drivetrain import SwerveDrivetrain
from phoenix6.hardware.pigeon2 import Pigeon2

from util import limelight_helpers
from util.nt_util import NTTable

from subsystems.drivetrain.swerve_tuner import TunerConstants

from constants.vision import kOdometry


class Vision(commands2.Subsystem):
    """
    Only uses measurments of 1 Limelight at a time. (Can take inputs from multiple)"""

    def __init__(self, *camera_names):
        self._cameras = camera_names
        self._pigeon = Pigeon2(TunerConstants._pigeon_id)
        self.offset = 0
        self.tilt = 0
        
        self.nt = NTTable('Vision')
        self.nt.bool('Drive State Provided')
        self.nt.float('Robot Tilt')

        self.nt = NTTable("Vision")
        self.nt.bool("Drive State Provided")
        self.nt.float("Robot Tilt")
        self.nt.float("Required April Tags", kOdometry.MIN_APRILTAGS)
        self.nt.float("Maximum Tilt Error", kOdometry.MAX_TILT_ERROR)
        self.nt.bool("Ignore Tilt", kOdometry.IGNORE_TILT)

    def get_best_measurement(self, estimates: list[limelight_helpers.PoseEstimate]):
        if not estimates:
            return None

        if all(estimate.tagCount == 0 for estimate in estimates):
            return None

        # Gets the limelight measurement with the most tags seen.
        # If the number of tags seen is the same, it will choose the measurement
        # which is closest to the tags.
        measurement = max(
            estimates,
            key=lambda estimate: (
                estimate.tagCount,
                (
                    -estimate.avgTagDist
                    if estimate.tagCount > kOdometry.MIN_APRILTAGS
                    else float("-inf")
                ),
            ),
        )
        if measurement.tagCount < kOdometry.MIN_APRILTAGS:
            return None

        return measurement

    def get_vision_odometry(
        self, drive_state: SwerveDrivetrain.SwerveDriveState = None, use_megatag2=False
    ):
        """Chooses the most accurate limelight and calculates the odometry"""

        self.nt.set("Drive State Provided", drive_state is not None)
        if drive_state is None:
            return

        headingDeg = drive_state.pose.rotation().degrees()
        omegaRPS = units.radiansToRotations(drive_state.speeds.omega)
        if kOdometry.MAX_RPS < abs(omegaRPS):
            return None
        if self.tilt > kOdometry.MAX_TILT_ERROR and kOdometry.IGNORE_TILT == False:
            return None

        entry_name = "botpose_orb_wpiblue" if use_megatag2 else "botpose_wpiblue"

        vision_estimates = []
        for camera in self._cameras:
            limelight_helpers.set_robot_orientation(camera, headingDeg, 0, 0, 0, 0, 0)
            limelight_measurement = limelight_helpers.get_bot_pose_estimate(
                camera, entry_name, use_megatag2
            )
            if limelight_measurement:
                vision_estimates.append(limelight_measurement)

        measurement = self.get_best_measurement(vision_estimates)
        return measurement

    def periodic(self):
        self.tilt = math.sqrt(
            self._pigeon.get_pitch().value ** 2 + self._pigeon.get_roll().value ** 2
        )
        self.nt.set("Robot Tilt", self.tilt)

        kOdometry.MAX_TILT_ERROR = self.nt.get("Maximum Tilt Error")
        kOdometry.MIN_APRILTAGS = self.nt.get("Required April Tags")
        kOdometry.IGNORE_TILT = self.nt.get("Ignore Tilt")

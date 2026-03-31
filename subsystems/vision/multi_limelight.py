import math

import commands2

from wpimath import units

from phoenix6.swerve.swerve_drivetrain import SwerveDrivetrain
from phoenix6.hardware.pigeon2 import Pigeon2

from util.limelight import LimelightHelpers, PoseEstimate
from util.nt_util import NTTable

from subsystems.drivetrain.swerve_tuner import TunerConstants

from constants.vision import kOdometry


class Vision(commands2.Subsystem):
    """
    Accepts pose estimates from one or more Limelights.

    Supports both MegaTag1 and MegaTag2 pipelines, switchable at runtime
    via the NT "Use MegaTag2" entry without redeploying.
    """

    def __init__(self, *camera_names: str):
        super().__init__()
        self._cameras = camera_names
        self._pigeon = Pigeon2(TunerConstants._pigeon_id)
        self.tilt = 0.0

        self.nt = NTTable("Vision")
        self.nt.bool("Drive State Provided", False)
        self.nt.float("Robot Tilt", 0.0)
        self.nt.float("Maximum Tilt Error", kOdometry.MAX_TILT_ERROR)
        self.nt.bool("Ignore Tilt", kOdometry.IGNORE_TILT)
        self.nt.bool("Use MegaTag2", kOdometry.USE_MEGATAG2)

    def get_valid_measurements(self, estimates: list[PoseEstimate]) -> list[PoseEstimate]:
        """Filter estimates to only those meeting the acceptance criteria.

        Criteria:
          - tag_count >= min_tags for the active pipeline
          - avg_tag_dist <= MAX_TAG_DIST
          - no individual tag has ambiguity > MAX_TAG_AMBIGUITY
            (if raw_fiducials are populated; skipped when the array is empty)
        All passing estimates are returned so the pose estimator can fuse
        every camera independently rather than discarding any valid data.
        """
        min_tags = kOdometry.MT2_MIN_APRILTAGS if kOdometry.USE_MEGATAG2 else kOdometry.MT1_MIN_APRILTAGS
        valid = []
        for e in estimates:
            if e.tag_count < min_tags:
                continue
            if e.avg_tag_dist > kOdometry.MAX_TAG_DIST:
                continue
            if e.raw_fiducials and any(
                f is not None and f.ambiguity > kOdometry.MAX_TAG_AMBIGUITY
                for f in e.raw_fiducials
            ):
                continue
            valid.append(e)
        return valid

    def get_vision_odometry(
        self, drive_state: SwerveDrivetrain.SwerveDriveState = None
    ) -> list[PoseEstimate]:
        """Return all valid pose estimates across every camera.

        Returns an empty list when global filters (rotation rate, tilt) fail
        or no camera has a qualifying measurement.  The caller should call
        ``add_vision_measurement`` once per estimate so the Kalman filter fuses
        all cameras rather than only the "best" one.
        """
        self.nt.set("Drive State Provided", drive_state is not None)
        if drive_state is None:
            return []

        heading_deg  = drive_state.pose.rotation().degrees()
        omega_rps    = units.radiansToRotations(drive_state.speeds.omega)
        yaw_rate_dps = math.degrees(drive_state.speeds.omega)  # rad/s → deg/s

        if abs(omega_rps) > kOdometry.MAX_RPS:
            return []
        if self.tilt > kOdometry.MAX_TILT_ERROR and not kOdometry.IGNORE_TILT:
            return []

        use_mt2 = kOdometry.USE_MEGATAG2
        raw: list[PoseEstimate] = []

        for camera in self._cameras:
            if use_mt2:
                LimelightHelpers.set_robot_orientation(
                    camera, heading_deg, yaw_rate_dps, 0, 0, 0, 0
                )
                result = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(camera)
            else:
                result = LimelightHelpers.get_botpose_estimate_wpiblue(camera)

            if result is not None and result.tag_count > 0:
                raw.append(result)

        return self.get_valid_measurements(raw)

    def periodic(self):
        self.tilt = math.sqrt(
            self._pigeon.get_pitch().value ** 2 + self._pigeon.get_roll().value ** 2
        )
        self.nt.set("Robot Tilt", self.tilt)

        kOdometry.MAX_TILT_ERROR = self.nt.get("Maximum Tilt Error")
        kOdometry.IGNORE_TILT    = self.nt.get("Ignore Tilt")
        kOdometry.USE_MEGATAG2   = self.nt.get("Use MegaTag2")

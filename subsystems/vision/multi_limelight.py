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

    def get_valid_measurements(
        self, estimates: list[PoseEstimate]
    ) -> list[PoseEstimate]:
        min_tags = (
            kOdometry.MT2_MIN_APRILTAGS
            if kOdometry.USE_MEGATAG2
            else kOdometry.MT1_MIN_APRILTAGS
        )
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

    def get_strong_mt1_measurement(
        self, drive_state: SwerveDrivetrain.SwerveDriveState = None
    ) -> PoseEstimate | None:
        if drive_state is None:
            return None
        if abs(units.radiansToRotations(drive_state.speeds.omega)) > kOdometry.MAX_RPS:
            return None

        candidates: list[PoseEstimate] = []
        for camera in self._cameras:
            result = LimelightHelpers.get_botpose_estimate_wpiblue(camera)
            if result is None or result.tag_count < kOdometry.MT1_MIN_APRILTAGS:
                continue
            if result.avg_tag_dist > kOdometry.MT1_RESET_MAX_TAG_DIST:
                continue
            if result.raw_fiducials and any(
                f is not None and f.ambiguity > kOdometry.MT1_RESET_MAX_AMBIGUITY
                for f in result.raw_fiducials
            ):
                continue
            candidates.append(result)

        if not candidates:
            return None

        return min(candidates, key=lambda e: e.avg_tag_dist)

    def get_vision_odometry(
        self, drive_state: SwerveDrivetrain.SwerveDriveState = None
    ) -> list[PoseEstimate]:
        self.nt.set("Drive State Provided", drive_state is not None)
        if drive_state is None:
            return []

        heading_deg = drive_state.pose.rotation().degrees()
        omega_rps = units.radiansToRotations(drive_state.speeds.omega)
        yaw_rate_dps = math.degrees(drive_state.speeds.omega)

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
        kOdometry.IGNORE_TILT = self.nt.get("Ignore Tilt")
        kOdometry.USE_MEGATAG2 = self.nt.get("Use MegaTag2")

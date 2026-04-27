from __future__ import annotations

from typing import TYPE_CHECKING

from constants.vision import kOdometry

if TYPE_CHECKING:
    from subsystems.vision.multi_limelight import Vision
    from subsystems.drivetrain.drivetrain import SwerveDriveTrain


def update_odometry(vision: "Vision", drivetrain: "SwerveDriveTrain") -> None:
    """Push fresh vision measurements into drivetrain pose estimation.

    Call directly from robotPeriodic so it runs every loop regardless of
    command scheduler state.
    """
    drive_state = drivetrain.get_state().robot_state

    strong = vision.get_strong_mt1_measurement(drive_state)
    if strong is not None:
        drivetrain.add_vision_measurement(
            strong.pose,
            strong.timestamp_seconds,
            (kOdometry.MT1_RESET_XY_STD, kOdometry.MT1_RESET_XY_STD, kOdometry.MT1_RESET_THETA_STD),
        )
        return

    current_pose = drive_state.pose

    for m in vision.get_vision_measurements(drive_state):
        if kOdometry.USE_MEGATAG2:
            pose_error = m.pose.translation().distance(current_pose.translation())
            if pose_error > kOdometry.MT2_MAX_POSE_ERROR:
                continue
            xy_std = kOdometry.MT2_XY_COEFF * m.avg_tag_dist / m.tag_count
            std_dev = (xy_std, xy_std, kOdometry.MT2_THETA_STD_DEV)
        else:
            xy_std    = kOdometry.MT1_XY_COEFF    * m.avg_tag_dist / m.tag_count
            theta_std = kOdometry.MT1_THETA_COEFF * m.avg_tag_dist / m.tag_count
            std_dev = (xy_std, xy_std, theta_std)

        drivetrain.add_vision_measurement(m.pose, m.timestamp_seconds, std_dev)

import commands2
from wpimath import units

from phoenix6.swerve.swerve_drivetrain import SwerveDrivetrain

from constants.vision import kOdometry

from util import limelight_helpers


class Vision(commands2.Subsystem):
    def __init__(self, *camera_names):
        self._cameras = camera_names

    def get_ideal_limelight(self):
        # TODO add logic for choosing the best limelight
        return self._cameras[0]

    def get_vision_odometry(
        self, drive_state: SwerveDrivetrain.SwerveDriveState = None, use_megatag2=False
    ):
        """Chooses the most accurate limelight and calculates the odometry"""
        if drive_state is None:
            raise ("Drive state not provided for vision based odometry")

        limelight_name = self.get_ideal_limelight()

        headingDeg = drive_state.pose.rotation().degrees()
        omegaRPS = units.radiansToRotations(drive_state.speeds.omega)
        if kOdometry.MAX_RPS < abs(omegaRPS):
            return None

        if use_megatag2:
            limelight_helpers.set_robot_orientation(
                limelight_name, headingDeg, 0, 0, 0, 0, 0
            )
            entry_name = "botpose_orb_wpiblue"
        else:
            entry_name = "botpose_wpiblue"

        limelight_measurement = limelight_helpers.get_bot_pose_estimate(
            limelight_name, entry_name, use_megatag2
        )
        return limelight_measurement

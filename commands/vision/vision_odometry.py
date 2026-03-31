import commands2

from subsystems.vision import multi_limelight
from subsystems.drivetrain import drivetrain

from constants.vision import kOdometry


class UpdateOdometry(commands2.Command):
    def __init__(
        self, vision: multi_limelight.Vision, drivetrain: drivetrain.SwerveDriveTrain
    ):
        super().__init__()
        self._vision = vision
        self._drivetrain = drivetrain
        self.addRequirements(vision)

    def initialize(self):
        pass

    def execute(self):
        drive_state = self._drivetrain.get_state().robot_state
        
        strong = self._vision.get_strong_mt1_measurement(drive_state)
        if strong is not None:
            self._drivetrain.add_vision_measurement(
                strong.pose,
                strong.timestamp_seconds,
                (
                    kOdometry.MT1_RESET_XY_STD,
                    kOdometry.MT1_RESET_XY_STD,
                    kOdometry.MT1_RESET_THETA_STD,
                ),
            )
            return

        current_pose = drive_state.pose

        for m in self._vision.get_vision_measurements(drive_state):
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

            self._drivetrain.add_vision_measurement(
                m.pose,
                m.timestamp_seconds,
                std_dev,
            )

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        pass

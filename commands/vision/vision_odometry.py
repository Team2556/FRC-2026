import commands2
from subsystems.vision import mono_limelight
from subsystems.drivetrain import drivetrain


class UpdateOdometry(commands2.Command):
    def __init__(
        self, vision: mono_limelight.Vision, drivetrain: drivetrain.SwerveDriveTrain
    ):
        self._vision = vision
        self._drivetrain = drivetrain

        self.std_dev = (0.1, 0.1, 0.1)
        
        self.addRequirements(vision)

    def execute(self):
        drive_state = self._drivetrain.get_state().robot_state
        ll_measurement = self._vision.get_vision_odometry(drive_state, False)
        if ll_measurement != None:
            self._drivetrain._drivetrain.add_vision_measurement(
                ll_measurement.pose, ll_measurement.timestampSeconds, self.std_dev
            )

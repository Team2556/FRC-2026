import commands2

from constants.tuners import kVisionOdometry

from subsystems import vision, drivetrain


class LimelightOdometry(commands2.Command):
    def __init__(
        self,
        vision_subsystem: vision.Vision,
        drivetrain_subsystem: drivetrain.SwerveDriveTrain,
    ):
        self._vision = vision_subsystem
        self._drivetrain = drivetrain_subsystem

        self.addRequirements(self._vision)

    def execute(self):
        drive_state = self._drivetrain._drivetrain.get_state()
        ll_measurement = self._vision.get_vision_odometry(
            drive_state=drive_state, use_megatag2=kVisionOdometry.USE_MEGATAG_2
        )

        if ll_measurement != None:
            self._drivetrain._add_vision_measurements(
                ll_measurement.pose, ll_measurement.timestampSeconds
            )

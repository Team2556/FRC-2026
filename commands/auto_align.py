import commands2

from wpilib import SmartDashboard

from phoenix6 import swerve

from subsystems.limelight_camera import LimelightCamera


class StationaryAlign(commands2.Command):
    def __init__(
        self,
        limelight: LimelightCamera,
    ):
        super().__init__()
        self._limelight = limelight

        self._limelight.setPipeline(1)
        
        self.addRequirements(self._limelight)
        
    def execute(self):
        x = self._limelight.getX()
        if abs(x) < 0.01:
            x = 0
            
        SmartDashboard.putNumber('April Tag X', x)

        turn_speed = -0.005 * x


from commands2 import Command

from subsystems.shooter.shooter_hood import ShooterHood
from subsystems.drivetrain.drivetrain import SwerveDriveTrain

from constants.shooter import kHoodMotor


class ResetShooterHood(Command):
    def __init__(self, shooter_hood: ShooterHood):
        super().__init__()
        self._hood = shooter_hood
        self.addRequirements(self._hood)

    def initialize(self):
        self._hood.start_reset()
        self._hood.set_speed(kHoodMotor.RESET_HOME_SPEED)

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return self._hood.is_hard_stopped()

    def end(self, interrupted: bool):
        self._hood.set_speed(0)
        self._hood.zero_encoder()
        self._hood.end_reset()


class UpdateHoodPositionVariable(Command):
    def __init__(self, shooter_hood: ShooterHood, drivetrain: SwerveDriveTrain):
        super().__init__()
        self._hood = shooter_hood
        self._drivetrain = drivetrain
        self.addRequirements(self._hood)

    def initialize(self):
        pass

    def execute(self):
        self._hood.set_target_angle(kHoodMotor.HOME_ANGLE_DEG)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        pass

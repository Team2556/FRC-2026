from commands2 import Command, InterruptionBehavior

from subsystems.shooter.shooter_hood import ShooterHood, HoodStates
from subsystems.drivetrain.drivetrain import SwerveDriveTrain

from constants.shooter import kHoodMotor


class ResetShooterHood(Command):
    def __init__(self, shooter_hood: ShooterHood):
        super().__init__()
        self._hood = shooter_hood
        self.addRequirements(self._hood)

    def initialize(self):
        self._hood.set_state(HoodStates.RESETTING)
        self._hood.set_speed(kHoodMotor.RESET_HOME_SPEED)

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return self._hood.is_hard_stopped()

    def end(self, interrupted: bool):
        self._hood.set_speed(0)
        self._hood.zero_encoder()


class UpdateHoodPositionVariable(Command):
    def __init__(self, shooter_hood: ShooterHood, drivetrain: SwerveDriveTrain):
        super().__init__()
        self.shooter_hood = shooter_hood
        self.drivetrain = drivetrain
        self.addRequirements(self.shooter_hood)

    def initialize(self):
        pass

    def execute(self):
        if self.drivetrain.should_stop_shooting():
            self.shooter_hood.set_state(HoodStates.HIDE)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        pass


class SetShooterHoodState(Command):
    def __init__(self, shooter_hood: ShooterHood, state: HoodStates):
        super().__init__()
        self._hood = shooter_hood
        self._state = state
        self.addRequirements(self._hood)

    def initialize(self):
        pass

    def execute(self):
        self._hood.set_state(self._state)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        pass

    def getInterruptionBehavior(self):
        return InterruptionBehavior.kCancelSelf

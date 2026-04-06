from commands2 import Command

from phoenix6 import signals

from subsystems.intake.intake_pivot import IntakePivot

from constants.intake import kIntakePivot


class IntakePivotDefaultCommand(Command):
    def __init__(self, pivot: IntakePivot, drivetrain):
        super().__init__()
        self._pivot = pivot
        self._drivetrain = drivetrain
        self._retract_commanded = False
        self.addRequirements(pivot)

    def initialize(self):
        self._retract_commanded = False

    def execute(self):
        if (
            self._pivot.is_any_motor_not_retracted()
            and self._pivot.state != "deployed"
            and not self._retract_commanded
        ):
            self._pivot.set_deployer_position(0)
            self._pivot.state = "default reverse"
            self._retract_commanded = True

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        pass


class IntakePivotForward(Command):
    def __init__(self, pivot: IntakePivot):
        super().__init__()
        self._pivot = pivot
        self.addRequirements(pivot)

    def initialize(self):
        self._pivot.set_deployer_position(kIntakePivot.DEPLOYED_POSITION)
        self._pivot.state = "deploying"

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return self._pivot.is_at_forward_limit()

    def end(self, interrupted: bool):
        self._pivot.set_pivot_neutral_mode(signals.NeutralModeValue.COAST)
        self._pivot.set_deployer_speed(0)
        self._pivot.state = "deployed"


class IntakePivotReverse(Command):
    def __init__(self, pivot: IntakePivot):
        super().__init__()
        self._pivot = pivot
        self.addRequirements(pivot)

    def initialize(self):
        self._pivot.set_deployer_position(0)
        self._pivot.state = "undeploying"

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return self._pivot.is_at_reverse_limit()

    def end(self, interrupted: bool):
        self._pivot.set_pivot_neutral_mode(signals.NeutralModeValue.BRAKE)
        self._pivot.state = "undeployed"

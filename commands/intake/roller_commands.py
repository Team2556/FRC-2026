import wpilib

from commands2 import Command

from subsystems.drivetrain.drivetrain import SwerveDriveTrain
from subsystems.intake.intake_pivot import IntakePivot
from subsystems.intake.intake_roller import IntakeRoller

from constants.intake import kIntakePivot, kIntakeRoller


class IntakeRollerDefaultCommand(Command):
    def __init__(self, roller: IntakeRoller, pivot: IntakePivot, drivetrain: SwerveDriveTrain):
        super().__init__()
        self._roller = roller
        self._pivot = pivot
        self._drivetrain = drivetrain
        self.addRequirements(roller)

    def initialize(self):
        pass

    def execute(self):
        if self._pivot.state == "deployed":
            robot_vel_x = self._drivetrain.get_state().velocity.translation().x
            toward_intake = robot_vel_x * kIntakePivot.INTAKE_DIRECTION
            if toward_intake >= kIntakeRoller.AUTO_INTAKE_SPEED_THRESHOLD:
                self._roller.set_target_rpm(kIntakeRoller.TARGET_RPM)
            else:
                self._roller.stop_roller()
        else:
            self._roller.stop_roller()

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self._roller.stop_roller()


class IntakeRollerForward(Command):
    def __init__(self, roller: IntakeRoller):
        super().__init__()
        self._roller = roller
        self.addRequirements(roller)

    def initialize(self):
        pass

    def execute(self):
        self._roller.set_target_rpm(kIntakeRoller.TARGET_RPM)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self._roller.stop_roller()


class IntakeRollerBackward(Command):
    def __init__(self, roller: IntakeRoller):
        super().__init__()
        self._roller = roller
        self.addRequirements(roller)

    def initialize(self):
        pass

    def execute(self):
        self._roller.set_target_rpm(kIntakeRoller.TARGET_RPM * -1)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self._roller.stop_roller()


class IntakeRollerOscillate(Command):
    PERIOD = 0.6
    INTAKE_DURATION = 0.45

    def __init__(self, roller: IntakeRoller):
        super().__init__()
        self._roller = roller
        self.addRequirements(roller)

    def initialize(self):
        pass

    def execute(self):
        phase = wpilib.Timer.getFPGATimestamp() % self.PERIOD
        if phase < self.INTAKE_DURATION:
            self._roller.set_target_rpm(kIntakeRoller.TARGET_RPM)
        else:
            self._roller.set_target_rpm(kIntakeRoller.TARGET_RPM * -1)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self._roller.stop_roller()

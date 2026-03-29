import math

import wpilib

from commands2 import Command

from phoenix6 import signals

from subsystems.intake.intake import IntakeSubsystem

from constants.intake import kIntakePivot, kIntakeRoller


class IntakeCommandDeploy(Command):
    def __init__(self, intake_subsystem: IntakeSubsystem):
        super().__init__()
        self.intake_subsystem = intake_subsystem
        self.reverse_limit = self.intake_subsystem.left_pivot_motor.get_reverse_limit()
        self.addRequirements(intake_subsystem)

    def initialize(self):
        self.intake_subsystem.set_deployer_position(kIntakePivot.DEPLOYED_POSITION)
        self.intake_subsystem.change_deployer_slot(0)
        self.intake_subsystem.set_roller_speed(kIntakeRoller.TARGET_RPM)
        self.intake_subsystem.state = "deploying"

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return self.reverse_limit.value == signals.ReverseLimitValue.CLOSED_TO_GROUND

    def end(self, interrupted: bool):
        self.intake_subsystem.set_pivot_neutral_mode(signals.NeutralModeValue.COAST)
        self.intake_subsystem.state = "deployed"


class IntakeCommandUndeploy(Command):
    def __init__(self, intake_subsystem: IntakeSubsystem):
        super().__init__()
        self.intake_subsystem = intake_subsystem
        self.foward_limit = self.intake_subsystem.left_pivot_motor.get_forward_limit()
        self.addRequirements(intake_subsystem)

    def initialize(self):
        self._start_time = wpilib.Timer.getFPGATimestamp()
        self.intake_subsystem.stop_roller()
        self.intake_subsystem.set_deployer_position(0)
        self.intake_subsystem.change_deployer_slot(0)
        self.intake_subsystem.state = "undeploying"

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return self.foward_limit.value == signals.ForwardLimitValue.CLOSED_TO_GROUND

    def end(self, interrupted: bool):
        self.intake_subsystem.set_internal_deployer_position(0)
        self.intake_subsystem.stop_roller()
        self.intake_subsystem.set_pivot_neutral_mode(signals.NeutralModeValue.BRAKE)
        self.intake_subsystem.state = "undeployed"


class IntakeForceRetract(Command):
    """Drives the deployer to position 0 until the reverse limit switch triggers, then zeros the encoder."""

    def __init__(self, intake_subsystem: IntakeSubsystem):
        super().__init__()
        self.intake_subsystem = intake_subsystem
        self.reverse_limit = self.intake_subsystem.left_pivot_motor.get_reverse_limit()
        self.addRequirements(intake_subsystem)

    def initialize(self):
        self.intake_subsystem.stop_roller()
        self.intake_subsystem.set_deployer_position(0)
        self.intake_subsystem.state = "force_retracting"

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return self.reverse_limit.value == signals.ReverseLimitValue.CLOSED_TO_GROUND

    def end(self, interrupted: bool):
        self.intake_subsystem.set_internal_deployer_position(0)
        self.intake_subsystem.set_pivot_neutral_mode(signals.NeutralModeValue.BRAKE)
        self.intake_subsystem.state = "force_retracted" if not interrupted else "undeployed"


class IntakeCommandManualForward(Command):
    def __init__(self, intake_subsystem: IntakeSubsystem):
        super().__init__()
        self.intake_subsystem = intake_subsystem
        self.addRequirements(intake_subsystem)

    def initialize(self):
        self.intake_subsystem.set_deployer_position(kIntakePivot.DEPLOYED_POSITION)
        self.intake_subsystem.state = "deploying"

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return self.intake_subsystem.left_pivot_motor.get_forward_limit().value == signals.ForwardLimitValue.CLOSED_TO_GROUND

    def end(self, interrupted: bool):
        self.intake_subsystem.set_pivot_neutral_mode(signals.NeutralModeValue.COAST)
        # I need to stop the intake from continuing to try to move, not stop the rollers, so I don't want to set the ideal roller RPM to 0 here
        self.intake_subsystem.set_deployer_speed(0) # this may wack up the motionmagic, right??
        

        self.intake_subsystem.state = "deployed"


class IntakeCommandManualReverse(Command):
    def __init__(self, intake_subsystem: IntakeSubsystem):
        super().__init__()
        self.intake_subsystem = intake_subsystem
        self.addRequirements(intake_subsystem)

    def initialize(self):
        self.intake_subsystem.set_deployer_position(0)
        self.intake_subsystem.state = "undeploying"

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return self.intake_subsystem.left_pivot_motor.get_reverse_limit().value == signals.ReverseLimitValue.CLOSED_TO_GROUND

    def end(self, interrupted: bool):
        self.intake_subsystem.set_pivot_neutral_mode(signals.NeutralModeValue.BRAKE)
        self.intake_subsystem.state = "undeployed"


# Temporary Commands
class IntakeCommandManualForwardAuto(Command):
    def __init__(self, intake_subsystem: IntakeSubsystem):
        super().__init__()
        self.intake_subsystem = intake_subsystem
        self.addRequirements(intake_subsystem)

    def initialize(self):
        self.intake_subsystem.set_deployer_position(kIntakePivot.DEPLOYED_POSITION)
        self.intake_subsystem.state = "deploying"
        self.intake_subsystem.set_roller_speed(kIntakeRoller.TARGET_RPM)

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return self.intake_subsystem.left_pivot_motor.get_forward_limit().value == signals.ForwardLimitValue.CLOSED_TO_GROUND

    def end(self, interrupted: bool):
        self.intake_subsystem.set_pivot_neutral_mode(signals.NeutralModeValue.COAST)
        self.intake_subsystem.state = "deployed"


class IntakeCommandManualReverseAuto(Command):
    def __init__(self, intake_subsystem: IntakeSubsystem):
        super().__init__()
        self.intake_subsystem = intake_subsystem
        self.addRequirements(intake_subsystem)

    def initialize(self):
        self.intake_subsystem.set_deployer_position(0)
        self.intake_subsystem.state = "undeploying"
        self.intake_subsystem.stop_roller()

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return self.intake_subsystem.left_pivot_motor.get_reverse_limit().value == signals.ReverseLimitValue.CLOSED_TO_GROUND

    def end(self, interrupted: bool):
        self.intake_subsystem.set_pivot_neutral_mode(signals.NeutralModeValue.BRAKE)
        self.intake_subsystem.state = "undeployed"


class IntakeDefaultCommand(Command):
    """Default intake command.

    While the intake is deployed and the robot is moving toward the intake
    side at or above ``AUTO_INTAKE_SPEED_THRESHOLD``, the rollers spin at
    TARGET_RPM to automatically pick up balls.  When the robot slows or
    stops, the rollers idle.  The existing deployer-correction behaviour is
    preserved for when the intake is between positions.
    """

    def __init__(self, intake_subsystem: IntakeSubsystem, drivetrain):
        super().__init__()
        self.intake_subsystem = intake_subsystem
        self._drivetrain = drivetrain
        self._retract_commanded = False
        self.addRequirements(intake_subsystem)

    def initialize(self):
        self._retract_commanded = False

    def execute(self):
        is_deployed = self.intake_subsystem.state == "deployed"

        if is_deployed:
            # Robot-relative X velocity, projected onto the intake's axis.
            # Positive result means the robot is moving toward its intake side.
            robot_vel_x = self._drivetrain.get_state().velocity.translation().x
            toward_intake = robot_vel_x * kIntakePivot.INTAKE_DIRECTION
            if toward_intake >= kIntakeRoller.AUTO_INTAKE_SPEED_THRESHOLD:
                self.intake_subsystem.set_roller_speed(kIntakeRoller.TARGET_RPM)
            else:
                self.intake_subsystem.stop_roller()
        if ((self.intake_subsystem.left_pivot_motor.get_reverse_limit().value == signals.ReverseLimitValue.OPEN
             or self.intake_subsystem.right_pivot_motor.get_reverse_limit().value == signals.ReverseLimitValue.OPEN)
        and not self.intake_subsystem.state == "deployed"
        and not self._retract_commanded):
            self.intake_subsystem.set_deployer_position(0)
            self.intake_subsystem.state = "default reverse"
            self._retract_commanded = True

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self.intake_subsystem.stop_roller()


class IntakeRollerForward(Command):
    def __init__(self, intake_subsystem: IntakeSubsystem):
        super().__init__()
        self.intake_subsystem = intake_subsystem
        self.addRequirements(intake_subsystem)

    def initialize(self):
        pass

    def execute(self):
        self.intake_subsystem.set_roller_speed(kIntakeRoller.TARGET_RPM)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self.intake_subsystem.stop_roller()


class IntakeRollerBackward(Command):
    def __init__(self, intake_subsystem: IntakeSubsystem):
        super().__init__()
        self.intake_subsystem = intake_subsystem
        self.addRequirements(intake_subsystem)

    def initialize(self):
        pass

    def execute(self):
        self.intake_subsystem.set_roller_speed(kIntakeRoller.TARGET_RPM * -1)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self.intake_subsystem.stop_roller()


class IntakeRollerStop(Command):
    def __init__(self, intake_subsystem: IntakeSubsystem):
        super().__init__()
        self.intake_subsystem = intake_subsystem
        self.addRequirements(intake_subsystem)

    def initialize(self):
        self.intake_subsystem.stop_roller()

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool):
        pass


class IntakeRollerOscillate(Command):
    PERIOD = 1.0          # seconds per full cycle
    INTAKE_DURATION = 0.9  # seconds of forward (intake) per cycle

    def __init__(self, intake_subsystem: IntakeSubsystem):
        super().__init__()
        self.intake_subsystem = intake_subsystem
        self.addRequirements(intake_subsystem)

    def initialize(self):
        pass

    def execute(self):
        phase = wpilib.Timer.getFPGATimestamp() % self.PERIOD
        if phase < self.INTAKE_DURATION:
            self.intake_subsystem.set_roller_speed(kIntakeRoller.TARGET_RPM)
        else:
            self.intake_subsystem.set_roller_speed(kIntakeRoller.TARGET_RPM * -1)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self.intake_subsystem.stop_roller()

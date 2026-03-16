import wpilib
from wpilib import SmartDashboard

from commands2 import Command, InterruptionBehavior

from phoenix6 import signals

from subsystems.intake.intake import IntakeSubsystem

from constants.intake import kIntakePivot, kIntakeRoller


class IntakeCommandDeploy(Command):
    def __init__(self, intake_subsystem: IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
        self.forward_limit = self.intake_subsystem.left_pivot_motor.get_forward_limit()
        self.addRequirements(intake_subsystem)

    def initialize(self):
        self.intake_subsystem.set_deployer_position(kIntakePivot.DEPLOYED_POSITION)
        self.intake_subsystem.change_deployer_slot(0)
        self.intake_subsystem.set_roller_speed(kIntakeRoller.TARGET_RPM)
        self.intake_subsystem.state = "deploying"

    def isFinished(self):
        return self.forward_limit.value == signals.ForwardLimitValue.CLOSED_TO_GROUND

    def end(self, interrupted):
        self.intake_subsystem.change_deployer_slot(1)
        self.intake_subsystem.state = "deployed"


class IntakeCommandUndeploy(Command):
    def __init__(self, intake_subsystem: IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
        self.reverse_limit = self.intake_subsystem.left_pivot_motor.get_reverse_limit()
        self.addRequirements(intake_subsystem)

    def initialize(self):
        self._start_time = wpilib.Timer.getFPGATimestamp()
        self.intake_subsystem.stop_roller()
        self.intake_subsystem.set_deployer_position(0)
        self.intake_subsystem.change_deployer_slot(0)
        self.intake_subsystem.state = "undeploying"

    def isFinished(self):
        return self.reverse_limit.value == signals.ReverseLimitValue.CLOSED_TO_GROUND

    def end(self, interrupted):
        self.intake_subsystem.set_internal_deployer_position(0)
        self.intake_subsystem.stop_roller()
        self.intake_subsystem.state = "undeployed"


class IntakeForceRetract(Command):
    """Slowly drives the deployer backward until the reverse limit switch triggers, then zeros the encoder."""
    RETRACT_SPEED = -0.15  # gentle duty cycle toward home

    def __init__(self, intake_subsystem: IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
        self.reverse_limit = self.intake_subsystem.left_pivot_motor.get_reverse_limit()
        self.addRequirements(intake_subsystem)

    def initialize(self):
        self.intake_subsystem.stop_roller()
        self.intake_subsystem.state = "force_retracting"

    def execute(self):
        self.intake_subsystem.left_pivot_motor.set(self.RETRACT_SPEED)

    def isFinished(self):
        return self.reverse_limit.value == signals.ReverseLimitValue.CLOSED_TO_GROUND

    def end(self, interrupted):
        self.intake_subsystem.left_pivot_motor.set(0)
        self.intake_subsystem.set_internal_deployer_position(0)
        self.intake_subsystem.state = "force_retracted" if not interrupted else "undeployed"

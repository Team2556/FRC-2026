import wpilib
from wpilib import SmartDashboard

from commands2 import Command, InterruptionBehavior

from phoenix6 import signals

from subsystems.intake.intake import IntakeSubsystem

from constants.intake import kIntakePivot, kIntakeRoller


class IntakeCommandDeploy(Command):
    def __init__(self, intake_subsystem: IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
        self.reverse_limit = self.intake_subsystem.left_pivot_motor.get_reverse_limit()
        self.addRequirements(intake_subsystem)

    def initialize(self):
        self.intake_subsystem.set_deployer_position(kIntakePivot.DEPLOYED_POSITION)
        self.intake_subsystem.change_deployer_slot(0)
        self.intake_subsystem.set_roller_speed(kIntakeRoller.TARGET_RPM)
        self.intake_subsystem.state = "deploying"

    def isFinished(self):
        return self.reverse_limit.value == signals.ReverseLimitValue.CLOSED_TO_GROUND

    def end(self, interrupted):
        # self.intake_subsystem.change_deployer_slot(1)
        self.intake_subsystem.state = "deployed"


class IntakeCommandUndeploy(Command):
    def __init__(self, intake_subsystem: IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
        self.foward_limit = self.intake_subsystem.left_pivot_motor.get_forward_limit()
        self.addRequirements(intake_subsystem)

    def initialize(self):
        self._start_time = wpilib.Timer.getFPGATimestamp()
        self.intake_subsystem.stop_roller()
        self.intake_subsystem.set_deployer_position(0)
        self.intake_subsystem.change_deployer_slot(0)
        self.intake_subsystem.state = "undeploying"

    def isFinished(self):
        return self.foward_limit.value == signals.ForwardLimitValue.CLOSED_TO_GROUND

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

class IntakeCommandManualForward(Command):
    def __init__(self, intake_subsystem : IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
        self.addRequirements(intake_subsystem)
        
    def initialize(self):
        self.intake_subsystem.set_deployer_speed(kIntakePivot.DEPLOYED_SPEED)
        self.intake_subsystem.state = "deployed"
        # self.intake_subsystem.set_roller_speed(kIntakeRoller.TARGET_RPM)
    
    def isFinished(self):
        return self.intake_subsystem.left_pivot_motor.get_forward_limit().value == signals.ForwardLimitValue.CLOSED_TO_GROUND
    
    def end(self, interrupted):
        self.intake_subsystem.set_deployer_speed(0)
    
class IntakeCommandManualReverse(Command):
    def __init__(self, intake_subsystem : IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
        self.addRequirements(intake_subsystem)
        
    def initialize(self):
        self.intake_subsystem.set_deployer_speed(kIntakePivot.DEPLOYED_SPEED * -1 * 1.5)
        self.intake_subsystem.state = "undeployed"
        # self.intake_subsystem.stop_roller()
        
class IntakeCommandManualForwardAuto(Command):
    def __init__(self, intake_subsystem : IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
        self.addRequirements(intake_subsystem)
        
    def initialize(self):
        self.intake_subsystem.set_deployer_speed(kIntakePivot.DEPLOYED_SPEED)
        self.intake_subsystem.state = "deployed"
        # self.intake_subsystem.set_roller_speed(kIntakeRoller.TARGET_RPM)
    
    def isFinished(self):
        return self.intake_subsystem.left_pivot_motor.get_forward_limit().value == signals.ForwardLimitValue.CLOSED_TO_GROUND
    
    def end(self, interrupted):
        self.intake_subsystem.set_deployer_speed(0)
    
# Temporary Commands
class IntakeCommandManualReverseAuto(Command):
    def __init__(self, intake_subsystem : IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
        self.addRequirements(intake_subsystem)
        
    def initialize(self):
        self.intake_subsystem.set_deployer_speed(kIntakePivot.DEPLOYED_SPEED * -1)
        self.intake_subsystem.state = "undeployed"
        self.intake_subsystem.stop_roller()
    
    def isFinished(self):
        return self.intake_subsystem.left_pivot_motor.get_reverse_limit().value == signals.ReverseLimitValue.CLOSED_TO_GROUND
    
    def end(self, interrupted):
        self.intake_subsystem.set_deployer_speed(0)

class IntakeDefaultCommand(Command):
    def __init__(self, intake_subsystem : IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
        self.addRequirements(intake_subsystem)
        
    def execute(self):
        if ((self.intake_subsystem.left_pivot_motor.get_reverse_limit().value == signals.ReverseLimitValue.OPEN
             or self.intake_subsystem.right_pivot_motor.get_reverse_limit().value == signals.ReverseLimitValue.OPEN)
        and not self.intake_subsystem.state == "deployed"):
            self.intake_subsystem.set_deployer_speed(kIntakePivot.DEPLOYED_SPEED * -1.5)
            self.intake_subsystem.state = "default reverse"
            self.intake_subsystem.stop_roller()

class IntakeRollerForward(Command):
    def __init__(self, intake_subsystem : IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
        
    def execute(self):
        self.intake_subsystem.set_roller_speed(kIntakeRoller.TARGET_RPM)
    
    def end(self,interrupted):
        self.intake_subsystem.stop_roller()

class IntakeRollerBackward(Command):
    def __init__(self, intake_subsystem : IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
        
    def execute(self):
        self.intake_subsystem.set_roller_speed(kIntakeRoller.TARGET_RPM * -1)
        
    def end(self, interrupted):
        self.intake_subsystem.stop_roller()
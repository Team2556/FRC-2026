from subsystems.intake.intake import IntakeSubsystem
from commands2 import Command, InterruptionBehavior
from phoenix6 import signals
from constants.intake import kIntakeDeployer, kIntakeSpinner
from wpilib import SmartDashboard
   
class IntakeCommandDeploy(Command):
    def __init__(self, intake_subsystem : IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
        self.forward_limit = self.intake_subsystem.left_deployer.get_forward_limit()
        self.addRequirements(intake_subsystem)
        
    def initialize(self):
        self.intake_subsystem.set_deployer_position(kIntakeDeployer.DEPLOYED_POSITION)
        self.intake_subsystem.change_deployer_slot(0)
        self.intake_subsystem.set_spinny_speed(kIntakeSpinner.TARGET_RPM)
        self.intake_subsystem.state = "deploying"    
    
    def isFinished(self):
        return self.forward_limit.value == signals.ForwardLimitValue.CLOSED_TO_GROUND
    
    def end(self, interrupted):
        self.intake_subsystem.change_deployer_slot(1)
        self.intake_subsystem.state = "deployed"

class IntakeCommandUndeploy(Command):
    def __init__(self, intake_subsystem : IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
        self.reverse_limit = self.intake_subsystem.left_deployer.get_reverse_limit()
        self.addRequirements(intake_subsystem)
        
    def initialize(self):
        self.intake_subsystem.set_spinny_speed(0)
        self.intake_subsystem.set_deployer_position(0)
        self.intake_subsystem.change_deployer_slot(0)
        self.intake_subsystem.state = "undeploying"
    
    def isFinished(self):
        return self.reverse_limit.value == signals.ReverseLimitValue.CLOSED_TO_GROUND

    def end(self, interrupted):
        self.intake_subsystem.set_internal_deployer_position(0)
        self.intake_subsystem.set_spinny_speed(0)
        self.intake_subsystem.state = "undeployed"

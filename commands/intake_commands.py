from subsystems.intake import IntakeSubsystem
from commands2 import Command
from phoenix6 import signals
from constants.intake import kIntakeDeployer, kIntakeSpinner
from wpilib import SmartDashboard

class spinny_on(Command):
    def _init_(self, intake_subsystem : IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
    
    def execute(self):
        self.intake_subsystem.spinny_motor.set_control(self.intake_subsystem.velocity_voltage.with_velocity(kIntakeSpinner.TARGET_RPS))
        return super().execute()
    
    def isFinished(self, interrupted):
        return kIntakeDeployer.STATE == "undeployed" or  "undeploying"
    
class spinny_off(Command):
    def _init_ (self, intake_subsystem : IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
    
    def execute(self):
        self.intake_subsystem.spinny_motor.set_control(self.intake_subsystem.velocity_voltage.with_velocity(0))
        return super().execute()
    
    def isFinished(self):
       return kIntakeDeployer.STATE == "deploying" or "deployed"
    
class IntakeCommandDeploy(Command):
    def __init__(self, intake_subsystem : IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
        self.intake_subsystem.set_deployer_positon(1)
        self.forward_limit = self.intake_subsystem.left_deployer.get_forward_limit()
        
    def initialize(self):
        self.intake_subsystem.set_deployer_positon(kIntakeDeployer.DEPLOYED_POSITION)
        kIntakeDeployer.STATE = "deploying"    
    
    def execute(self):            
        if self.forward_limit.value is signals.ForwardLimitValue.CLOSED_TO_GROUND and kIntakeDeployer.STATE == "deploying":
            spinny_on()
            self.intake_subsystem.left_deployer.set_control(self.intake_subsystem.deployer_position_voltage.with_slot(1))
            kIntakeDeployer.STATE = "deployed"
            # detects if the left deployer motor is down and if true it sets it as "deployed"
        return super().execute()
    
    def end(self, interrupted):
        self.intake_subsystem.undeploy()

class IntakeCommandUndeploy(Command):
    def __init__(self, intake_subsystem : IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
        self.intake_subsystem.set_deployer_positon(0)
        self.reverse_limit = self.intake_subsystem.left_deployer.get_reverse_limit()
    def initialize(self):
        self.intake_subsystem.deploy()
        kIntakeDeployer.STATE = "undeploying"
    
    def execute(self):
        if self.reverse_limit.value is signals.ForwardLimitValue.CLOSED_TO_GROUND and kIntakeDeployer.STATE == "undeploying":
            spinny_off()
            self.intake_subsystem.left_deployer.set_control(self.intake_subsystem.deployer_position_voltage.with_slot(1))
            kIntakeDeployer.STATE = "undeployed"
            #detects if left deployer motor is up and sets the state to "undeployed"
        return super().execute()
    
    def isFinished(self):
        return super().isFinished()
    
    def end(self, interrupted):
        self.intake_subsystem.undeploy()

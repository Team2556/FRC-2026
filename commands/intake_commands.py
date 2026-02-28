from subsystems.intake import IntakeSubsystem
from commands2 import Command
from phoenix6 import signals

class IntakeCommandDeploy(Command):
    def __init__(self, intake_subsystem : IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
        self.intake_subsystem.set_deployer_positon(1)
        self.forward_limit = self.intake_subsystem.left_deployer.get_forward_limit()
    def initialize(self):
        self.intake_subsystem.deploy()
        print("hello")
    
    def execute(self):
        if self.forward_limit.value is signals.ForwardLimitValue.CLOSED_TO_GROUND and self.state == "deploying":
            self.intake_subsystem.left_deployer.set_control(self.intake_subsystem.deployer_position_voltage.with_slot(1))
            self.state = "deployed"
            # detects if the left deployer motor is down and if true it sets it as "deployed"
        return super().execute()
    
    def isFinished(self):
        return True
        return super().isFinished()
    
    def end(self, interrupted):
        self.intake_subsystem.undeploy()

class IntakeCommandUndeploy(Command):
    def __init__(self, intake_subsystem : IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
        self.intake_subsystem.set_deployer_positon(0)
        self.reverse_limit = self.intake_subsystem.left_deployer.get_reverse_limit()
    def initialize(self):
        self.intake_subsystem.deploy()
    
    def execute(self):
        if self.reverse_limit.value is signals.ForwardLimitValue.CLOSED_TO_GROUND and self.state == "undeploying":
            self.intake_subsystem.left_deployer.set_control(self.intake_subsystem.deployer_position_voltage.with_slot(1))
            self.state = "undeployed"
            #detects if left deployer motor is up and sets the state to "undeployed"
        return super().execute()
    
    def isFinished(self):
        return super().isFinished()
    
    def end(self, interrupted):
        self.intake_subsystem.undeploy()

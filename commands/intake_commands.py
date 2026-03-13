from subsystems.intake import IntakeSubsystem
from commands2 import Command
from phoenix6 import signals
from constants.intake import kIntakeDeployer, kIntakeSpinner
from wpilib import SmartDashboard
   
class intake_command_deploy(Command):
    def __init__(self, intake_subsystem : IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
        self.intake_subsystem.set_deployer_positon(1)
        self.forward_limit = self.intake_subsystem.left_deployer.get_forward_limit()
        
    def initialize(self):
        self.intake_subsystem.set_deployer_positon(kIntakeDeployer.DEPLOYED_POSITION)
        kIntakeDeployer.STATE = "deploying"    
    
    def isFinished(self):
        return self.forward_limit.value is signals.ForwardLimitValue.CLOSED_TO_GROUND
    
    def end(self, interrupted):
        self.intake_subsystem.spinny_motor.set_control(self.intake_subsystem.velocity_voltage.with_velocity(kIntakeSpinner.TARGET_RPS))
        self.intake_subsystem.left_deployer.set_control(self.intake_subsystem.deployer_position_voltage.with_slot(1))
        kIntakeDeployer.STATE = "deployed"

class intake_command_undeploy(Command):
    def __init__(self, intake_subsystem : IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
        self.intake_subsystem.set_deployer_positon(0)
        self.reverse_limit = self.intake_subsystem.left_deployer.get_reverse_limit()
        
    def initialize(self):
        self.intake_subsystem.undeploy()
        kIntakeDeployer.STATE = "undeploying"
    
    def isFinished(self):
        return self.reverse_limit.value is signals.ForwardLimitValue.CLOSED_TO_GROUND

    def end(self, interrupted):
        self.intake_subsystem.left_deployer.set_control(self.intake_subsystem.deployer_position_voltage.with_slot(1))
        kIntakeDeployer.STATE = "undeployed"
        self.intake_subsystem.spinny_motor.set_control(self.intake_subsystem.velocity_voltage.with_velocity(0))
        #detects if left deployer motor is up and sets the state to "undeployed"
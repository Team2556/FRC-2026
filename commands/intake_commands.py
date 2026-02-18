from subsystems.intake import IntakeSubsystem
from commands2 import Command

class IntakeCommand(Command):
    def __init__(self, intake_subsystem : IntakeSubsystem):
        self.intake_subsystem = intake_subsystem
        
    def initialize(self):
        self.intake_subsystem.deploy()
    
    def end(self, interrupted):
        self.intake_subsystem.undeploy()

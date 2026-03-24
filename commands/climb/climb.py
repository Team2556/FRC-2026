from commands2 import Command

from subsystems.climb.climb_subsystem import ClimbSubsystem

class ClimbUp(Command):
    def __init__(self, climb_subsystem: ClimbSubsystem):
        super().__init__()
        self.climb_subsystem = climb_subsystem
        self.addRequirements(climb_subsystem)

    def initialize(self):
        self.climb_subsystem.raise_climb()
    
    def isFinished(self):
        return True

class ClimbDown(Command):
    def __init__(self, climb_subsystem: ClimbSubsystem):
        super().__init__()
        self.climb_subsystem = climb_subsystem
        self.addRequirements(climb_subsystem)

    def initialize(self):
        self.climb_subsystem.lower_climb()

    def isFinished(self):
        return True


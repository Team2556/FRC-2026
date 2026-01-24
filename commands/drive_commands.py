import commands2

from subsystems import (drivetrain)

class ControllerDrive(commands2.Command):
    def __init__(self, drivetrain: drivetrain.SwerveDriveTrain, controller):
        super().__init__()
        self._drivetrain = drivetrain
        self._controller = controller
        
        self.addRequirements(self._drivetrain)
    
    def execute(self):
        self._drivetrain.drive_with_joystick(self._controller)
    
    def end(self, interrupt):
        self._drivetrain._stop()
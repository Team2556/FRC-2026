import commands2

from subsystems import controlled_motor

class SpinMotor(commands2.Command):
    def __init__(self, subsystem: controlled_motor.ControlledMotor):
        super().__init__()
        self._subsystem = subsystem
        
    def initialize(self):
        # This function is called when the command is first scheduled.
        pass
    
    def execute(self):
        self._subsystem.spin()
    
    def isFinished(self):
        # Add conditions for ending the command in this function.
        # If this funciton returns True, the command will end
        return False
    
    def end(self, interrupted):
        # This function is called after the command ends
        # the interrupted variable stores whether or not the command was interuppted or canceled.
        self._subsystem.stop_motor()
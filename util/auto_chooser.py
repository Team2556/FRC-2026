from commands2 import Command

from wpilib import SmartDashboard, SendableChooser

class AutoChooser():
    def __init__(self, autos : dict[str, Command]):
        self.autos = autos
    
    def make_dropdown(self):
        self.chooser = SendableChooser()
        
        first_index = False
        for auto in self.autos:
            if not first_index:
                self.chooser.setDefaultOption(auto, self.autos[auto])
                first_index = True
            else:
                self.chooser.addOption(auto, self.autos[auto])
        
        SmartDashboard.putData("Autonomous Selecter", self.chooser)
    
    def choose_auto(self):
        return self.chooser.getSelected()
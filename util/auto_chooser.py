from wpilib import SendableChooser

from util.nt_util import NTTable

from constants.path.key_poses import kPath

from constants.path.custom_path_commands import CustomPathCommands
from commands.path_commands.drive_to_a_spot_sequence import DriveToASpotSequence

class AutoBuilder:
    '''Handles the complicated auto builder stuff and communicates all path constants between the code and Elastic'''
    
    def __init__(self, teleop_paths: dict[str, DriveToASpotSequence] = None, auto_paths: dict[str, DriveToASpotSequence] = None):
        self.auto_paths = auto_paths
        self.teleop_paths = teleop_paths
        
        self._nt_auto = NTTable("Autonomous")
        
        self._nt_path = NTTable("Sequence Path")
        self._nt_path.float("Max Speed", kPath.default_path_speed)
        self._nt_path.float("Auto Speed", kPath.auto_path_speed)
        self._nt_path.float("Smoothing Radius", kPath.smoothing_radius)

    def make_dropdown(self):
        self.chooser = SendableChooser()

        first = True
        for name, command in self.auto_paths.items():
            if first:
                self.chooser.setDefaultOption(name, command)
                first = False
            else:
                self.chooser.addOption(name, command)

        self._nt_auto.sendable("Selector", self.chooser)

    def update(self) -> None:
        """Call every loop to sync the dashboard selection with the robot.

        SendableChooser writes its current selection back to the 'active' NT
        entry only when the builder is updated. Without this, dashboard
        changes (e.g. from Elastic) are never echoed back and getSelected()
        never sees the new value.
        """
        self._nt_auto.update_sendables()

        # Then get all NetworkTablesStuff (you still need to call update_sequence_paths to update all values in paths)
        # and all that updating should be just a button
        kPath.default_path_speed = self._nt_path.get("Max Speed")
        kPath.auto_path_speed = self._nt_path.get("Auto Speed")
        kPath.smoothing_radius = self._nt_path.get("Smoothing Radius")
    
    def update_sequence_paths(self):
        for auto_path in self.auto_paths:
            self.auto_paths[auto_path].reset_variables()
        for teleop_path in self.teleop_paths:
            self.teleop_paths[teleop_path].reset_variables()

    def choose_auto(self):
        return self.chooser.getSelected()

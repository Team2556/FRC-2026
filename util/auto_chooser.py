from wpilib import SendableChooser, DriverStation

from util.nt_util import NTTable

from constants.path.key_poses import kPath, kPoses

from constants.path.custom_path_commands import CustomPathCommands
from commands.path_commands.drive_to_a_spot_sequence import DriveToASpotSequence
from commands.path_commands.auto_intermediate import AutoIntermediate

from commands2 import cmd, SequentialCommandGroup

from typing import Callable

class AutoBuilder:
    '''Handles the complicated auto builder stuff and communicates all path constants between the code and Elastic'''
    
    path_amount = 6
    
    def __init__(self, auto_paths: dict[str, Callable] = None):
        self.auto_paths = auto_paths
        
        self._nt_path = NTTable("Sequence Path")
        self._nt_path.float("Max Speed", kPath.default_path_speed)
        self._nt_path.float("Auto Speed", kPath.auto_path_speed)
        self._nt_path.float("Bump Speed", kPath.bump_speed)
        self._nt_path.float("Intaking Speed", kPath.intaking_speed)
        self._nt_path.float("While Shooting Speed", kPath.while_shooting_speed)
        
        self._nt_path.float("Smoothing Radius Auto", kPath.smoothing_radius_auto)
        self._nt_path.float("Smoothing Radius Teleop", kPath.smoothing_radius_teleop)
        
        self._nt_path.float("Min Goal End Velocity Mult", kPath.min_goal_end_velocity_mult)
        self._nt_path.float("Smoothing Time Multiplier", kPath.smoothing_time_multiplier)
        
        self._nt_auto = NTTable("Autonomous")
        self._nt_auto.bool("Reverse Paths", kPath.MIRROR_REVERSE_PATHS)
        
        self.make_nt_display()

    def make_nt_display(self):
        self.auto_choosers : list[SendableChooser] = []
        self.nt_subtables : list[NTTable] = []

        for i in range(self.path_amount):
            self.nt_subtables.append(self._nt_auto.get_subtable(f"Path {i + 1}"))
            self.auto_choosers.append(SendableChooser())
            first = True
            for name, command in self.auto_paths.items():
                if first:
                    self.auto_choosers[i].setDefaultOption(name, command)
                    first = False
                else:
                    self.auto_choosers[i].addOption(name, command)
            self.nt_subtables[i].sendable("Path", self.auto_choosers[i])
            
            self.nt_subtables[i].float("Min Time to Start", 0)
            # self.nt_subtables[i].bool("Is Active", False)
            self.nt_subtables[i].bool("Cancel if Too Late", False)
            
        self.initial_pose = SendableChooser()
        first = True
        for name in kPoses.initial_poses:
            if first:
                self.initial_pose.setDefaultOption(name, kPoses.initial_poses[name])
                first = False
            else:
                self.initial_pose.addOption(name, kPoses.initial_poses[name])
        self._nt_auto.sendable("Initial Pose", self.initial_pose)

    def update(self) -> None:
        for i in range(self.path_amount):
            self.nt_subtables[i].update_sendables()
        self._nt_auto.update_sendables()
        
        kPath.MIRROR_REVERSE_PATHS = self._nt_auto.get("Reverse Paths")

        # Then get all NetworkTablesStuff (you still need to call choose_auto to update all values in paths)
        # and all that updating should be just a button
        kPath.default_path_speed = self._nt_path.get("Max Speed")
        kPath.auto_path_speed = self._nt_path.get("Auto Speed")
        kPath.bump_speed = self._nt_path.get("Bump Speed")
        kPath.intaking_speed = self._nt_path.get("Intaking Speed")
        kPath.while_shooting_speed = self._nt_path.get("While Shooting Speed")
        
        kPath.smoothing_radius_auto = self._nt_path.get("Smoothing Radius Auto")
        kPath.smoothing_radius_teleop = self._nt_path.get("Smoothing Radius Teleop")
        
        kPath.min_goal_end_velocity_mult = self._nt_path.get("Min Goal End Velocity Mult")
        kPath.smoothing_time_multiplier = self._nt_path.get("Smoothing Time Multiplier")
    
    def get_initial_pose(self):
        return self.initial_pose.getSelected()

    def choose_auto(self):
        return SequentialCommandGroup(
            (
                self.auto_choosers[i].getSelected()()
            ) for i in range(self.path_amount)
        )

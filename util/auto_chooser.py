from wpilib import SendableChooser, DriverStation

from util.nt_util import NTTable

from constants.path.key_poses import kPath, kPoses

from constants.path.custom_path_commands import CustomPathCommands
from commands.path_commands.drive_to_a_spot_sequence import DriveToASpotSequence
from commands.path_commands.auto_helpers import AutoCheckpoint

from commands2 import cmd, SequentialCommandGroup

from typing import Callable

class AutoBuilder:
    '''Handles the complicated auto builder stuff and communicates all path constants between the code and Elastic'''
    
    def __init__(self, auto_paths: dict[str, Callable] = None):
        self.auto_paths = auto_paths
        
        self.make_nt_display()

    def make_nt_display(self):
        
        # Path NT stuff
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
        
        # Auto NT stuff
        self._nt_auto = NTTable("Autonomous")
        
        self.auto_chooser = SendableChooser()
        first = True
        for name, command in self.auto_paths.items():
            if first:
                self.auto_chooser.setDefaultOption(name, command)
                first = False
            else:
                self.auto_chooser.addOption(name, command)
        self._nt_auto.sendable("Path", self.auto_chooser)
        
        self.initial_pose = SendableChooser()
        first = True
        for name in kPoses.initial_poses:
            if first:
                self.initial_pose.setDefaultOption(name, kPoses.initial_poses[name])
                first = False
            else:
                self.initial_pose.addOption(name, kPoses.initial_poses[name])
        self._nt_auto.sendable("Initial Pose", self.initial_pose)
        
        self._nt_auto.float("Checkpoint 1 Time", kPath.CHECKPOINT_1)
        self._nt_auto.float("Checkpoint 2 Time", kPath.CHECKPOINT_2)
        self._nt_auto.float("Start Delay", kPath.START_DELAY)
        self._nt_auto.bool("Reverse Paths", kPath.MIRROR_REVERSE_PATHS)

    def update(self) -> None:
        # Path NT stuff
        kPath.default_path_speed = self._nt_path.get("Max Speed")
        kPath.auto_path_speed = self._nt_path.get("Auto Speed")
        kPath.bump_speed = self._nt_path.get("Bump Speed")
        kPath.intaking_speed = self._nt_path.get("Intaking Speed")
        kPath.while_shooting_speed = self._nt_path.get("While Shooting Speed")
        
        kPath.smoothing_radius_auto = self._nt_path.get("Smoothing Radius Auto")
        kPath.smoothing_radius_teleop = self._nt_path.get("Smoothing Radius Teleop")
        
        kPath.min_goal_end_velocity_mult = self._nt_path.get("Min Goal End Velocity Mult")
        kPath.smoothing_time_multiplier = self._nt_path.get("Smoothing Time Multiplier")
        
        # Auto NT stuff
        self._nt_auto.update_sendables()
        
        kPath.MIRROR_REVERSE_PATHS = self._nt_auto.get("Reverse Paths")
        kPath.START_DELAY = self._nt_auto.get("Start Delay")
        kPath.CHECKPOINT_1 = self._nt_auto.get("Checkpoint 1 Time")
        kPath.CHECKPOINT_2 = self._nt_auto.get("Checkpoint 2 Time")
    
    def get_initial_pose(self):
        return self.initial_pose.getSelected()

    def choose_auto(self):
        return self.auto_chooser.getSelected()

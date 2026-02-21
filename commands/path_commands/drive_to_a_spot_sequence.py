from typing import List
import commands2
from commands.path_commands.drive_to_a_spot import DriveToASpot
from wpimath.geometry import Pose2d
from wpilib import Timer
from constants import key_poses

class DriveToASpotSequence(commands2.SequentialCommandGroup):
    def __init__(self, *commands: DriveToASpot, max_speed = key_poses.kPath.general_path_speed, smoothing_radius = 0.4, smoothing_time = 0.2) -> None:
        """
        Sequential command group specialized for DriveToASpot commands that have cool things like 
        pose smoothing and flipping the whole thing
        """
        
        # TODO make it calculate these
        self.smoothing_radius = smoothing_radius
        self.smoothing_time = smoothing_time
        
        self.is_during_smoothing = False
        
        self.timer = Timer()
        
        super().__init__(*commands)
        self._commands : list[DriveToASpot]
        
        for command in self._commands:
            command.max_speed = max_speed
            if command == self._commands[-1] or self.smoothing_radius == 0: break
            command = command.with_sequence_pose_values()
    
    def initialize(self):
        super().initialize()
        self.timer.stop()
        self.timer.reset()
        self.is_during_smoothing = False
    
    def execute(self):
        self.add_smoothing()
        
        super().execute()
        
    def next_command(self):
        currentCommand = self._commands[self._currentCommandIndex]
        currentCommand.end(False)
        self._currentCommandIndex += 1
        if self._currentCommandIndex < len(self._commands):
            self._commands[self._currentCommandIndex].initialize()
        
    def add_smoothing(self):
        
        if self.smoothing_radius == 0: return
        
        currentCommand : DriveToASpot = self._commands[self._currentCommandIndex]
        
        if currentCommand == self._commands[-1]: return
        
        if self.is_during_smoothing:
            if self.timer.get() >= self.smoothing_time:
                self.is_during_smoothing = False
                self.next_command()
                return
            
            current_command_weight = 1 - (self.timer.get() / self.smoothing_time)
            
            next_command : DriveToASpot = self._commands[self._currentCommandIndex + 1]
            next_velocity = next_command.calculate_velocity() * (1 - current_command_weight)
            
            currentCommand.command_weight = current_command_weight
            currentCommand.next_command_velocity = next_velocity    
        else:
            currentCommand.command_weight = 1.0
            currentCommand.next_command_velocity = Pose2d()
            
        if currentCommand.get_distance_progress() <= self.smoothing_radius and not self.is_during_smoothing:
            self.is_during_smoothing = True
            self.timer.reset()
            self.timer.start()
    
    def with_red_alliance_poses(self):
        for command in self._commands:
            command = command.with_red_alliance_pose()
        return self
    
    def with_reflected_red_alliance_poses(self):
        for command in self._commands:
            command = command.with_reflected_red_alliance_pose()
        return self
    
    def end(self, interrupted):
        super().end(interrupted)
        for command in self._commands:
            command.reset_variables()
import commands2
from commands.path_commands.drive_to_a_spot import DriveToASpot
from wpimath.geometry import Pose2d
from wpilib import Timer, SmartDashboard
from constants.key_poses import kPath

class DriveToASpotSequence(commands2.SequentialCommandGroup):
    def __init__(
        self, 
        *commands: DriveToASpot
        ) -> None:
        """
        Sequential command group specialized for DriveToASpot commands that have cool things like 
        pose smoothing and flipping the whole thing
        """
        
        self.is_during_smoothing = False
        
        self.timer = Timer()
        
        self.max_speed = kPath.default_path_speed
        self.smoothing_radius = kPath.default_smoothing_radius
        self.smoothing_time = kPath.default_smoothing_time
        
        super().__init__(*commands)
        self._commands : list[DriveToASpot]
        
        for command in self._commands:
            if command == self._commands[-1] or self.smoothing_radius == 0: break
            command = command.with_sequence_pose_values()
        
        SmartDashboard.putNumber("Sequence Path/Max Speed", kPath.default_path_speed)
        SmartDashboard.putNumber("Sequence Path/Smoothing Radius", kPath.default_smoothing_radius)
        SmartDashboard.putNumber("Sequence Path/Smoothing Time", kPath.default_smoothing_time)
        SmartDashboard.putNumber("Sequence Path/Slow Distance Proportional to Max Speed", kPath.percent_slow_distance_proportional_to_max_speed_for_sequence_path)
        SmartDashboard.putNumber("Sequence Path/Slow Transition Multiplier", kPath.path_transition_slow_multiplier)
    
    def initialize(self):
        super().initialize()
        self.timer.stop()
        self.timer.reset()
        self.is_during_smoothing = False
        
        self.max_speed = SmartDashboard.getNumber("Sequence Path/Max Speed", kPath.default_path_speed)
        self.smoothing_radius = SmartDashboard.getNumber("Sequence Path/Smoothing Radius", kPath.default_smoothing_radius)
        self.smoothing_time = SmartDashboard.getNumber("Sequence Path/Smoothing Time", kPath.default_smoothing_time)
        kPath.percent_slow_distance_proportional_to_max_speed_for_sequence_path = SmartDashboard.getNumber("Sequence Path/Slow Distance Proportional to Max Speed", kPath.percent_slow_distance_proportional_to_max_speed_for_sequence_path)
        kPath.path_transition_slow_multiplier = SmartDashboard.getNumber("Sequence Path/Slow Transition Multiplier", kPath.path_transition_slow_multiplier)
        
        for command in self._commands:
            if not command == self._commands[-1]:
                command = command.with_sequence_pose_values() # In case of SmartDashboard Change
            if command.do_override_speed:
                command.max_speed = command.better_max_speed
            else:
                command.max_speed = self.max_speed
            command.reset_variables()
    
    def execute(self):
        self.add_path_smoothing()
        super().execute()
        
    def next_command(self):
        currentCommand = self._commands[self._currentCommandIndex]
        currentCommand.end(False)
        self._currentCommandIndex += 1
        if self._currentCommandIndex < len(self._commands):
            self._commands[self._currentCommandIndex].initialize()
        
    def add_path_smoothing(self):
        
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
    
    def end(self, interrupted):
        super().end(interrupted)
        for command in self._commands:
            command.reset_variables()
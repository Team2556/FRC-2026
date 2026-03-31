import commands2
from wpimath.geometry import Pose2d, Rotation2d
from wpilib import Timer, DriverStation
from commands.path_commands.drive_to_a_spot import DriveToASpot
from commands.path_commands.pathfind_to_start import PathfindToStart
from constants.path.key_poses import kPath
from copy import deepcopy
from math import pi, cos

class DriveToASpotSequence(commands2.SequentialCommandGroup):
    def __init__(
        self,
        *commands: DriveToASpot,
        pathfind_to_start : PathfindToStart = None,
    ) -> None:
        """Sequential command group specialized for DriveToASpot commands that has cool pose smoothing and starting spot logic"""

        # TODO 
        # throw in an ideal starting spot
        # with an "optional commands" that can be used and are stored to be used with specific ideal starting spots

        self.pathfind_to_start = pathfind_to_start
        self.is_during_smoothing = False
        self.timer = Timer()

        super().__init__(*commands)
        self._commands: list[DriveToASpot]
        self.default_commands = commands
        
        for command in self.default_commands:
            if not command == self.default_commands[-1]:
                command = command.with_sequence_pose_values()

    def initialize(self):
        self.timer.stop()
        self.timer.reset()
        self.is_during_smoothing = False
        
        if DriverStation.isAutonomous():
            max_speed = kPath.auto_path_speed
        else:
            max_speed = kPath.default_path_speed

        if self.pathfind_to_start:
            self._commands = self.pathfind_to_start.generate_instructions() + list(self.default_commands)
        else:
            self._commands = self.default_commands
        
        for command in self._commands:
            command.max_speed = max_speed
            command.reset_variables()
            if not command == self._commands[-1]:
                command = command.with_sequence_pose_values()
        
        super().initialize()
        self.calculate_goal_end_velocity()

    def execute(self):
        self.add_path_smoothing()
        super().execute()

    def next_command(self):
        current_command = self._commands[self._currentCommandIndex]
        current_command.end(False)
        self._currentCommandIndex += 1
        if self._currentCommandIndex < len(self._commands):
            self._commands[self._currentCommandIndex].initialize()

        if not self._currentCommandIndex + 1 == len(self._commands): self.calculate_goal_end_velocity()
        
    def calculate_goal_end_velocity(self):
        '''Calculate goal end vleocity for next command (taking into account direction change between commands)'''
        current_command = self._commands[self._currentCommandIndex]
        next_command = self._commands[self._currentCommandIndex + 1]
        
        next_max_speed = next_command.max_speed
        
        current_angle = current_command.calculate_velocity().translation().angle()
        next_angle = (next_command.target_pose.translation() - current_command.target_pose.translation()).angle()
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            current_angle = current_angle.rotateBy(Rotation2d(pi))
        
        angle_difference = abs((next_angle - current_angle).radians())
        
        if angle_difference > pi/2:
            current_command.goal_end_velocity = 0
        else:
            current_command.goal_end_velocity = next_max_speed * cos(angle_difference)
        
        if current_command.goal_end_velocity < kPath.min_goal_end_velocity_mult * current_command.max_speed:
            current_command.goal_end_velocity = kPath.min_goal_end_velocity_mult * current_command.max_speed

    def add_path_smoothing(self):

        currentCommand: DriveToASpot = self._commands[self._currentCommandIndex]
        
        if currentCommand.smoothing_radius == 0:
            return

        if currentCommand == self._commands[-1]:
            return

        if self.is_during_smoothing:
            if (
                self.timer.get() >= currentCommand.smoothing_time 
                # or self.robot_is_within_distance(currentCommand, kPath.smoothing_radius * 0.15)
                ):
                self.is_during_smoothing = False
                self.next_command()
                return

            current_command_weight = 1 - (self.timer.get() / currentCommand.smoothing_time)

            next_command: DriveToASpot = self._commands[self._currentCommandIndex + 1]
            next_velocity = next_command.calculate_velocity() * (1 - current_command_weight)

            currentCommand.command_weight = current_command_weight
            currentCommand.next_command_velocity = next_velocity
        else:
            currentCommand.command_weight = 1.0
            currentCommand.next_command_velocity = Pose2d()

        if currentCommand.get_distance_progress() <= currentCommand.smoothing_radius and not self.is_during_smoothing:
            self.is_during_smoothing = True
            self.timer.reset()
            self.timer.start()

    def end(self, interrupted: bool):
        super().end(interrupted)
        for command in self._commands:
            command.reset_variables()
    
    def robot_is_within_distance(self, command: DriveToASpot, distance):
        robot_pose = command.drivetrain.get_state().pose
        target_pose = command.target_pose
        return robot_pose.translation().distance(target_pose.translation()) < distance
    
    def reset_variables(self):
        # Recalculates all variables assuming there's new constants
        for drive_command in self._commands:
            drive_command.reset_variables()
            drive_command.with_sequence_pose_values()
        
        # Calculate smoothing times after everything
        
        # TODO auto calculate lots of auto constants in a correct AND optimal way
        # kPath.smoothing_time = kPath.smoothing_radius * something

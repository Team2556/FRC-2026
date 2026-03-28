import commands2
from wpimath.geometry import Pose2d
from wpilib import Timer, DriverStation
from commands.path_commands.drive_to_a_spot import DriveToASpot
from commands.path_commands.pathfind_to_start import PathfindToStart
from constants.path.key_poses import kPath
from copy import deepcopy

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

    def initialize(self):
        self.timer.stop()
        self.timer.reset()
        self.is_during_smoothing = False
        
        if DriverStation.isAutonomous():
            max_speed = kPath.auto_path_speed
        else:
            max_speed = kPath.default_path_speed

        if self.pathfind_to_start:
            self._commands = self.pathfind_to_start.generate_instructions() + self.default_commands
        else:
            self._commands = self.default_commands
        
        for command in self._commands:
            command.max_speed = max_speed
            if not command == self._commands[-1]:
                command = command.with_sequence_pose_values()
            command.reset_variables()
        
        super().initialize()

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

        if kPath.smoothing_radius == 0:
            return

        currentCommand: DriveToASpot = self._commands[self._currentCommandIndex]

        if currentCommand == self._commands[-1]:
            return

        if self.is_during_smoothing:
            if self.timer.get() >= kPath.smoothing_time:
                self.is_during_smoothing = False
                self.next_command()
                return

            current_command_weight = 1 - (self.timer.get() / kPath.smoothing_time)

            next_command: DriveToASpot = self._commands[self._currentCommandIndex + 1]
            next_velocity = next_command.calculate_velocity() * (1 - current_command_weight)

            currentCommand.command_weight = current_command_weight
            currentCommand.next_command_velocity = next_velocity
        else:
            currentCommand.command_weight = 1.0
            currentCommand.next_command_velocity = Pose2d()

        if currentCommand.get_distance_progress() <= kPath.smoothing_radius and not self.is_during_smoothing:
            self.is_during_smoothing = True
            self.timer.reset()
            self.timer.start()

    def end(self, interrupted: bool):
        super().end(interrupted)
        for command in self._commands:
            command.reset_variables()
    
    def reset_variables(self):
        # Recalculates all variables assuming there's new constants
        for drive_command in self._commands:
            drive_command.update_variables()

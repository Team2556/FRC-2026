import commands2

from wpimath.geometry import Pose2d

from wpilib import Timer, DriverStation

from util.nt_util import NTTable

from commands.path_commands.drive_to_a_spot import DriveToASpot

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
        self._commands: list[DriveToASpot]

        for command in self._commands:
            if command == self._commands[-1] or self.smoothing_radius == 0:
                break
            command = command.with_sequence_pose_values()

        self._nt = NTTable("Sequence Path")
        self._nt_max_speed      = self._nt.float("Max Speed",      kPath.default_path_speed)
        self._nt_auto_speed     = self._nt.float("Auto Speed",     kPath.auto_path_speed)
        self._nt_smooth_radius  = self._nt.float("Smoothing Radius", kPath.default_smoothing_radius)
        self._nt_smooth_time    = self._nt.float("Smoothing Time",   kPath.default_smoothing_time)
        self._nt_slow_dist      = self._nt.float(
            "Slow Distance Proportional to Max Speed",
            kPath.percent_slow_distance_proportional_to_max_speed_for_sequence_path,
        )
        self._nt_slow_mult      = self._nt.float(
            "Slow Transition Multiplier",
            kPath.path_transition_slow_multiplier,
        )

    def initialize(self):
        super().initialize()
        self.timer.stop()
        self.timer.reset()
        self.is_during_smoothing = False

        if DriverStation.isAutonomous():
            self.max_speed = self._nt_auto_speed.get()
        else:
            self.max_speed = self._nt_max_speed.get()

        self.smoothing_radius = self._nt_smooth_radius.get()
        self.smoothing_time   = self._nt_smooth_time.get()
        kPath.percent_slow_distance_proportional_to_max_speed_for_sequence_path = self._nt_slow_dist.get()
        kPath.path_transition_slow_multiplier = self._nt_slow_mult.get()

        for command in self._commands:
            if not command == self._commands[-1]:
                command = command.with_sequence_pose_values()
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

        if self.smoothing_radius == 0:
            return

        currentCommand: DriveToASpot = self._commands[self._currentCommandIndex]

        if currentCommand == self._commands[-1]:
            return

        if self.is_during_smoothing:
            if self.timer.get() >= self.smoothing_time:
                self.is_during_smoothing = False
                self.next_command()
                return

            current_command_weight = 1 - (self.timer.get() / self.smoothing_time)

            next_command: DriveToASpot = self._commands[self._currentCommandIndex + 1]
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

    def end(self, interrupted: bool):
        super().end(interrupted)
        for command in self._commands:
            command.reset_variables()

from commands2 import Command
from commands.path_commands.drive_to_a_spot_sequence import DriveToASpotSequence
from typing import Callable, List
from wpimath.geometry import Translation2d
from subsystems.drivetrain.drivetrain import SwerveDriveTrain
from wpilib import DriverStation
from constants.key_poses import kTranslations

class TranslationCondition:
    def __init__(
        self,
        drivetrain: SwerveDriveTrain,
        initial_coordinate : Translation2d,
        opposite_coordinate: Translation2d = Translation2d(),
        ):
        '''
        Specific container that stores a rectangle of a possible pose a robot can be and can be asked
        if the robot is in said location. Make sure opposite_coordinate has x and y values greater than
        initial_coordinate
        '''
        self.drivetrain = drivetrain
        self.initial_coordinate = initial_coordinate
        self.opposite_coordinate = opposite_coordinate
        self.mirror_on_red_alliance = True
        
    def get(self):
        '''Returns whether the condition currently is true or not'''
        return self.__call__()
        
    def __call__(self):
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed and self.mirror_on_red_alliance:
            if (
                self.drivetrain.get_state().pose.translation().X() > kTranslations.field_x - self.opposite_coordinate.X()
                and self.drivetrain.get_state().pose.translation().X() < kTranslations.field_x - self.initial_coordinate.X()
                and self.drivetrain.get_state().pose.translation().Y() < kTranslations.field_y - self.opposite_coordinate.Y()
                and self.drivetrain.get_state().pose.translation().Y() < kTranslations.field_y - self.initial_coordinate.Y()
            ): return True
        else:
            if (
                self.drivetrain.get_state().pose.translation().X() > self.initial_coordinate.X()
                and self.drivetrain.get_state().pose.translation().X() < self.opposite_coordinate.X()
                and self.drivetrain.get_state().pose.translation().Y() < self.initial_coordinate.Y()
                and self.drivetrain.get_state().pose.translation().Y() < self.opposite_coordinate.Y()
            ): return True
        return False
    
    def with_no_change_on_red_alliance(self):
        self.mirror_on_red_alliance = False
        return self
        

class ConditionalPathSequence(DriveToASpotSequence):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.condition = lambda: True
        self.overrideInitialize = False
    
    def with_condition(self, condition: Callable):
        self.condition = condition
        return self

class SequenceChooser(Command):
    def __init__(
        self,
        *sequence_commands: ConditionalPathSequence,
    ):
        '''
        Takes a bunch of conditional sequence path commands and runs the first command (in order) that it finds
        with it's condition being true
        '''
        super().__init__()
        
        self.sequence_commands: List[ConditionalPathSequence] = sequence_commands
        for sequence_command in self.sequence_commands:
            sequence_command.overrideInitialize = True
        
        self.chosen_command : ConditionalPathSequence = None
    
    def initialize(self):
        super().initialize()
        for sequence_command in self.sequence_commands:
            if sequence_command.condition():
                sequence_command.schedule()
                self.chosen_command = sequence_command
                break
            
    def with_mirrored_poses_on_red_alliance(self):
        for sequence_command in self.sequence_commands:
            sequence_command = sequence_command.with_mirrored_poses_on_red_alliance()
        return self
    
    def end(self, interrupted):
        if self.chosen_command:
            self.chosen_command.end(True)
from commands.path_commands.pathfind_to_start import PathfindInstruction, PathfindToStart
from wpimath.geometry import Pose2d
from util.robot_zone_checker import RobotZoneChecker
from constants.path.key_poses import kPoses
from subsystems.drivetrain.drivetrain import SwerveDriveTrain

class PathfindInstructionBuilder():
    '''Stores all the factory functions that make individual pathfind instructions'''
    
    @staticmethod
    def alliance_to_neutral_left_trench(): return PathfindInstruction(
        RobotZoneChecker.is_in_alliance_zone,
        kPoses.alliance_left_trench,
        kPoses.neutral_close_left_trench
    )

    @staticmethod
    def transition_to_neutral_left_trench(): return PathfindInstruction(
        RobotZoneChecker.is_in_alliance_zone,
        kPoses.neutral_close_left_trench
    )
    
    @staticmethod
    def alliance_to_neutral_right_trench(): return PathfindInstruction(
        RobotZoneChecker.is_in_alliance_zone,
        kPoses.alliance_right_trench,
        kPoses.neutral_close_right_trench
    )

    @staticmethod
    def transition_to_neutral_right_trench(): return PathfindInstruction(
        RobotZoneChecker.is_in_alliance_transition_zone,
        kPoses.neutral_close_right_trench
    )

class PathfindToStartBuilder():
    '''Stores all the factory functions that make pathfind to start builders'''
    
    @staticmethod
    def neutral_thorugh_left_trench(drivetrain : SwerveDriveTrain):
        return PathfindToStart(
            drivetrain,
            PathfindInstructionBuilder.transition_to_neutral_left_trench(),
            PathfindInstructionBuilder.alliance_to_neutral_left_trench(),
        )

    @staticmethod
    def neutral_thorugh_right_trench(drivetrain : SwerveDriveTrain):
        return PathfindToStart(
            drivetrain,
            PathfindInstructionBuilder.transition_to_neutral_right_trench(),
            PathfindInstructionBuilder.alliance_to_neutral_right_trench(),
        )
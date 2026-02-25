import commands2

from ntcore import NetworkTableInstance
from phoenix6 import swerve
from subsystems.drivetrain.drivetrain import SwerveDriveTrain
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from pathplannerlib.path import PathPlannerPath, PathConstraints, GoalEndState
from pathplannerlib.auto import AutoBuilder
import math
from wpilib import DriverStation

from wpimath.units import rotationsToRadians
from commands.path_commands.drive_to_a_spot import DriveToASpot
from commands.path_commands.drive_to_a_spot_sequence import DriveToASpotSequence
from constants.key_poses import kPoses

class GoBackWithPath(commands2.Command):
    def __init__(self, subsystem: SwerveDriveTrain):
        super().__init__()
        
        self.drivetrain : SwerveDriveTrain = subsystem
        
        self.poseCommands : dict[str, DriveToASpotSequence] = {
            "blue_top": DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.behind_trench_top),
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_top).with_goal_end_velocity(0)
            ),
            "blue_bottom": DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.behind_trench_bottom),
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_bottom).with_goal_end_velocity(0)
            ),
            "blue_top_far": DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.opposing_zone_top),
                DriveToASpot(self.drivetrain, target_pose = kPoses.behind_trench_top),
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_top).with_goal_end_velocity(0)
            ),
            "blue_bottom_far": DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.opposing_zone_bottom),
                DriveToASpot(self.drivetrain, target_pose = kPoses.behind_trench_bottom),
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_bottom).with_goal_end_velocity(0)
            ),
            "red_top": DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.behind_trench_top),
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_top).with_goal_end_velocity(0)
            ).with_red_alliance_poses(),
            "red_bottom": DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.behind_trench_bottom),
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_bottom).with_goal_end_velocity(0)
            ).with_red_alliance_poses(),
            "red_top_far": DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.opposing_zone_top),
                DriveToASpot(self.drivetrain, target_pose = kPoses.behind_trench_top),
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_top).with_goal_end_velocity(0)
            ).with_red_alliance_poses(),
            "red_bottom_far": DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.opposing_zone_bottom),
                DriveToASpot(self.drivetrain, target_pose = kPoses.behind_trench_bottom),
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_bottom).with_goal_end_velocity(0)
            ).with_red_alliance_poses(),
        }
        
        for key in self.poseCommands:
            self.poseCommands[key].addRequirements(self.drivetrain)

    def initialize(self):
        
        current_pose = self.drivetrain.get_state().pose
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            alliance = "blue"
        else:
            alliance = "red"
        
        if current_pose.X() <= 5.0 and alliance == "blue":
            return
        if current_pose.X() >= 11.5 and alliance == "red":
            return
        
        target_command_string : str = alliance
        
        if current_pose.Y() >= 4:
            target_command_string += "_top"
        else:
            target_command_string += "_bottom"
        
        if alliance == "blue" and current_pose.X() >= 12.5:
            target_command_string += "_far"
        if alliance == "red" and current_pose.X() <= 4.0:
            target_command_string += "_far"
        
        self.poseCommands[target_command_string].schedule()
            
    def execute(self):
        pass
    
    def isFinished(self):
        # Add conditions for ending the command in this function.
        # If this funciton returns True, the command will end
        return False
    
    def end(self, interrupted):
        # This function is called after the command ends
        # the interrupted variable stores whether or not the command was interuppted or canceled.
        
        for key in self.poseCommands:
            self.poseCommands[key].cancel()
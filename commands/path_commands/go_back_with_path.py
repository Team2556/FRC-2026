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
            "left": DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_left_trench),
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_left_trench).with_goal_end_velocity(0)
            ),
            "right": DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_right_trench),
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_right_trench).with_goal_end_velocity(0)
            ),
            "left_far": DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.opposing_zone_left_trench),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_left_trench),
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_left_trench).with_goal_end_velocity(0)
            ),
            "right_far": DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.opposing_zone_right_trench),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_right_trench),
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_right_trench).with_goal_end_velocity(0)
            ),
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
        
        target_command_string : str = ""
        
        do_top_path = current_pose.Y() >= 4
        if alliance == "red":
            do_top_path = not do_top_path
        
        if do_top_path:
            target_command_string += "left"
        else:
            target_command_string += "right"
        
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
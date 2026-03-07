from commands2 import cmd, ConditionalCommand, WaitCommand, SequentialCommandGroup
from wpimath.geometry import Pose2d, Rotation2d

from commands.path_commands.drive_to_a_spot import DriveToASpot
from commands.path_commands.drive_to_a_spot_sequence import DriveToASpotSequence

from util.robot_zone_checker import RobotZoneChecker
from util.flip_util import FlipUtil

from constants.key_poses import kPoses

from subsystems.drivetrain import drivetrain


class CustomPathCommands:
    '''"Container" that has all the custom useful path commands'''
    def __init__(self, drivetrain : drivetrain.SwerveDriveTrain):
        
        self.drivetrain = drivetrain
        
        self.back_up_to_outpost = ConditionalCommand(
            ConditionalCommand(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.to_outpost0),
                    DriveToASpot(self.drivetrain, target_pose = kPoses.to_outpost1),
                    DriveToASpot(self.drivetrain, target_pose = kPoses.to_outpost_final
                                 ).with_goal_end_velocity(0).with_override_speed(1.5)
                ),
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.to_outpost1),
                    DriveToASpot(self.drivetrain, target_pose = kPoses.to_outpost_final
                                 ).with_goal_end_velocity(0).with_override_speed(1.5)
                ),
                lambda : RobotZoneChecker.is_within_pose(
                    self.drivetrain.get_state().pose,
                    FlipUtil.fieldPose(Pose2d(0, 4.2, Rotation2d())),
                    FlipUtil.fieldPose(Pose2d(1.2, 8.07, Rotation2d()))
                )
            ),
            cmd.none(),
            lambda : RobotZoneChecker.is_in_alliance_zone(self.drivetrain.get_state().pose)
        )
        
        self.right_trench_retreat = ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_right_trench),
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_right_trench).with_goal_end_velocity(0),
            ),
            cmd.none(),
            lambda : RobotZoneChecker.is_in_neutral_zone(self.drivetrain.get_state().pose)   
        )
        
        self.right_bump_retreat = ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_right_bump),
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_right_bump).with_goal_end_velocity(0),
            ),
            cmd.none(),
            lambda : RobotZoneChecker.is_in_neutral_zone(self.drivetrain.get_state().pose)   
        )
        
        self.left_bump_retreat = ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_left_bump),
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_left_bump).with_goal_end_velocity(0),
            ),
            cmd.none(),
            lambda : RobotZoneChecker.is_in_neutral_zone(self.drivetrain.get_state().pose)   
        )
        
        self.left_trench_retreat = ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_left_trench),
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_left_trench).with_goal_end_velocity(0),
            ),
            cmd.none(),
            lambda : RobotZoneChecker.is_in_neutral_zone(self.drivetrain.get_state().pose)   
        )
        
        self.right_trench_advance = ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_right_trench),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_right_trench).with_goal_end_velocity(0),
            ),
            cmd.none(),
            lambda : RobotZoneChecker.is_in_alliance_zone(self.drivetrain.get_state().pose)   
        )
        
        self.right_bump_advance = ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_right_bump),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_right_bump).with_goal_end_velocity(0),
            ),
            cmd.none(),
            lambda : RobotZoneChecker.is_in_alliance_zone(self.drivetrain.get_state().pose)   
        )
        
        self.left_bump_advance = ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_left_bump),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_left_bump).with_goal_end_velocity(0),
            ),
            cmd.none(),
            lambda : RobotZoneChecker.is_in_alliance_zone(self.drivetrain.get_state().pose)   
        )
        
        self.left_trench_advance = ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_left_trench),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_left_trench).with_goal_end_velocity(0),
            ),
            cmd.none(),
            lambda : RobotZoneChecker.is_in_alliance_zone(self.drivetrain.get_state().pose)   
        )
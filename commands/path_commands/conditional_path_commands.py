from commands2 import cmd, ConditionalCommand
from commands.path_commands.drive_to_a_spot import DriveToASpot
from commands.path_commands.drive_to_a_spot_sequence import DriveToASpotSequence
from util.robot_zone_checker import RobotZoneChecker
from constants.key_poses import kPoses
from subsystems.drivetrain import drivetrain

class ConditionalPathCommands:
    
    def __init__(self, drivetrain : drivetrain.SwerveDriveTrain):
        '''Container that has all the conditional path commands'''
        
        self.drivetrain = drivetrain
        
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
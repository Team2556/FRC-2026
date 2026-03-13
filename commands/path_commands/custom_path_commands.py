from commands2 import cmd, ConditionalCommand, WaitCommand, SequentialCommandGroup
from wpimath.geometry import Pose2d, Rotation2d

from commands.path_commands.drive_to_a_spot import DriveToASpot
from commands.path_commands.drive_to_a_spot_sequence import DriveToASpotSequence
from commands.auto_align.path_with_align import DriveWithAlign

from util.robot_zone_checker import RobotZoneChecker
from util.flip_util import FlipUtil

from constants.key_poses import kPoses
from constants.field import kHub

from subsystems.drivetrain.drivetrain import SwerveDriveTrain
from subsystems.shooter.shooter_hood import ShooterHood
from subsystems.controlled_motor import ControlledTalonMotor

class CustomPathCommands:
    '''"Container" that has all the custom useful path commands'''
    def __init__(
        self,
        drivetrain : SwerveDriveTrain = None,
        hood_subsystem : ShooterHood = None,
        shooter_subsystem : ControlledTalonMotor = None,
        climb_subsyetem : None = None,
        ):
        
        self.drivetrain = drivetrain
        self.hood_subsystem = hood_subsystem
        self.shooter_subsystem = shooter_subsystem
        
        self.make_path_commands()
        self.make_autos()
        
    def make_autos(self):
        
        self.test_auto = SequentialCommandGroup(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.auto1),
                DriveToASpot(self.drivetrain, target_pose = kPoses.auto2),
                DriveToASpot(self.drivetrain, target_pose = kPoses.auto3).with_override_speed(1),
                DriveToASpot(self.drivetrain, target_pose = kPoses.auto4),
                DriveToASpot(self.drivetrain, target_pose = kPoses.auto5),
                DriveWithAlign(
                    shooter = self.shooter_subsystem,
                    hood = self.hood_subsystem,
                    alignment_target = kHub.POS,
                    subsystem = self.drivetrain, 
                    target_pose = kPoses.auto6
                ).with_end_tolerance(0.5).with_goal_end_velocity(0).with_override_speed(0.35),
                DriveToASpot(self.drivetrain, target_pose = kPoses.auto6).with_precise_values()
            )
        )
        
    def make_path_commands(self):
        
        self.back_up_to_outpost = ConditionalCommand(
            ConditionalCommand(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.to_outpost0),
                    DriveToASpot(self.drivetrain, target_pose = kPoses.to_outpost1),
                    DriveToASpot(self.drivetrain, target_pose = kPoses.to_outpost_final
                                 ).with_precise_values()
                ),
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.to_outpost1),
                    DriveToASpot(self.drivetrain, target_pose = kPoses.to_outpost_final
                                 ).with_precise_values()
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
        
        self.right_trench = ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_right_trench),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_right_trench).with_goal_end_velocity(0),
            ),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_right_trench),
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_right_trench).with_goal_end_velocity(0),
            ),
            lambda : RobotZoneChecker.is_in_alliance_zone(self.drivetrain.get_state().pose)   
        )
        
        self.right_bump = ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_right_bump),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_right_bump).with_goal_end_velocity(0),
            ),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_right_bump),
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_right_bump).with_goal_end_velocity(0),
            ),
            lambda : RobotZoneChecker.is_in_alliance_zone(self.drivetrain.get_state().pose)   
        )
        
        self.left_trench = ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_left_trench),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_left_trench).with_goal_end_velocity(0),
            ),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_left_trench),
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_left_trench).with_goal_end_velocity(0),
            ),
            lambda : RobotZoneChecker.is_in_alliance_zone(self.drivetrain.get_state().pose)   
        )
        
        self.left_bump = ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_left_bump),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_left_bump).with_goal_end_velocity(0),
            ),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_left_bump),
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_left_bump).with_goal_end_velocity(0),
            ),
            lambda : RobotZoneChecker.is_in_alliance_zone(self.drivetrain.get_state().pose)   
        )
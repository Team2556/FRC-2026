from math import pi

from commands2 import cmd, ConditionalCommand, WaitCommand, SequentialCommandGroup, ParallelRaceGroup

from wpimath.geometry import Pose2d, Rotation2d

from util.robot_zone_checker import RobotZoneChecker
from util.flip_util import FlipUtil

from subsystems.drivetrain.drivetrain import SwerveDriveTrain
from subsystems.shooter.shooter_hood import ShooterHood
from subsystems.shooter.dual_shooter import DualMotorShooter
from subsystems.trasnfer.transfer_subsystem import TransferSubsystem
from subsystems.intake.intake import IntakeSubsystem
from subsystems.led.LED_controller import CANdleLEDController

from commands.path_commands.drive_to_a_spot import DriveToASpot
from commands.path_commands.drive_to_a_spot_sequence import DriveToASpotSequence
from commands.auto_align.path_with_align import DriveWithAlign
from commands.auto_align.align_with_controller import ConditionalAlignAndShoot
from commands.drive.drive_commands import InitialPose, AutoDrive
from commands.intake.intake_commands import IntakeCommandManualForwardAuto, IntakeCommandManualReverseAuto, IntakeRollerForward
from commands.shooter.shooter_commands import EnableShooter, DisableShooter

from constants.path.key_poses import kPoses, kPath
from constants.path.drive_to_start_instructions import PathfindToStartBuilder
from constants.field import kHub


class CustomPathCommands:
    '''"Container" that has all the custom useful path commands'''
    def __init__(
        self,
        drivetrain : SwerveDriveTrain = None,
        intake_subsystem : IntakeSubsystem = None,
        transfer_subsystem : TransferSubsystem = None,
        shooter_subsystem : DualMotorShooter = None,
        hood_subsystem : ShooterHood = None,
        led_subsystem : CANdleLEDController = None,
        climb_subsyetem : None = None,
        ):
        
        self.drivetrain = drivetrain
        self.shooter_subsystem = shooter_subsystem
        self.transfer_subsystem = transfer_subsystem
        self.intake_subsystem = intake_subsystem
        self.hood_subsystem = hood_subsystem
        self.led_subsystem = led_subsystem
        
        self.make_teleop_paths()
        self.make_auto_paths()
    
    # Might be still useful?
    def opposite_pose_rotation(self, pose : Pose2d):
        return Pose2d(
            pose.X(),
            pose.Y(),
            Rotation2d(pose.rotation().radians() + pi)
        )
        
    def make_auto_paths(self):
        self.auto_paths = {
        "right_half_sweep" : SequentialCommandGroup(
            InitialPose(self.drivetrain, Pose2d(1, 0.55, Rotation2d())),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.right_sweep_1),
                DriveToASpot(self.drivetrain, target_pose = kPoses.right_sweep_2),
                pathfind_to_start = PathfindToStartBuilder.neutral_thorugh_right_trench(self.drivetrain),
            ),
        )
        }
        
    def make_teleop_paths(self):
        self.teleop_paths = {
        "right_trench" : ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.alliance_right_trench)),
                DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.neutral_close_right_trench)).with_goal_end_velocity(0),
            ),
            ConditionalCommand(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_close_right_trench),
                    DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_right_trench).with_goal_end_velocity(0),
                ),
                cmd.none(),
                lambda : RobotZoneChecker.is_in_neutral_zone(self.drivetrain.get_state().pose) 
            ),
            lambda : RobotZoneChecker.is_in_hub_shooting_zone(self.drivetrain.get_state().pose)   
        ),
        "right_bump" : ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.alliance_right_bump)),
                DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.neutral_close_right_bump)).with_goal_end_velocity(0),
            ),
            ConditionalCommand(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_close_right_bump),
                    DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_right_bump).with_goal_end_velocity(0),
                ),
                cmd.none(),
                lambda : RobotZoneChecker.is_in_neutral_zone(self.drivetrain.get_state().pose) 
            ),
            lambda : RobotZoneChecker.is_in_hub_shooting_zone(self.drivetrain.get_state().pose)   
        ),
        "left_trench" : ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.alliance_left_trench)),
                DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.neutral_close_left_trench)).with_goal_end_velocity(0),
            ),
            ConditionalCommand(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_close_left_trench),
                    DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_left_trench).with_goal_end_velocity(0),
                ),
                cmd.none(),
                lambda : RobotZoneChecker.is_in_neutral_zone(self.drivetrain.get_state().pose) 
            ),
            lambda : RobotZoneChecker.is_in_hub_shooting_zone(self.drivetrain.get_state().pose)   
        ),
        "left_bump" : ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.alliance_left_bump)),
                DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.neutral_close_left_bump)).with_goal_end_velocity(0),
            ),
            ConditionalCommand(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_close_left_bump),
                    DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_left_bump).with_goal_end_velocity(0),
                ),
                cmd.none(),
                lambda : RobotZoneChecker.is_in_neutral_zone(self.drivetrain.get_state().pose) 
            ),
            lambda : RobotZoneChecker.is_in_hub_shooting_zone(self.drivetrain.get_state().pose)   
        )
        }
    
    def get_auto_paths(self):
        return self.auto_paths

    def get_teleop_paths(self):
        return self.teleop_paths
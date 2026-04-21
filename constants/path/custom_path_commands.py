from math import pi

from commands2 import cmd, ConditionalCommand, WaitCommand, SequentialCommandGroup, ParallelRaceGroup, ParallelDeadlineGroup, InstantCommand, RepeatCommand, PrintCommand

from wpimath.geometry import Pose2d, Rotation2d

from util.robot_zone_checker import RobotZoneChecker
from util.flip_util import FlipUtil

from subsystems.drivetrain.drivetrain import SwerveDriveTrain
from subsystems.shooter.shooter_hood import ShooterHood
from subsystems.shooter.dual_shooter import DualMotorShooter
from subsystems.trasnfer.transfer_subsystem import TransferSubsystem
from subsystems.intake.intake_roller import IntakeRoller
from subsystems.led.LED_controller import CANdleLEDController

from commands.path_commands.drive_to_a_spot import DriveToASpot
from commands.path_commands.drive_to_a_spot_sequence import DriveToASpotSequence
from commands.auto_align.align_with_controller import ConditionalAlignAndShoot
from commands.auto_align.alignio import AlignIntakeToVelocity
from commands.intake.roller_commands import IntakeRollerForward, IntakeRollerBackward, IntakeRollerOscillate
from commands.shooter.shooter_commands import EnableShooter, DisableShooter
from commands.path_commands.auto_helpers import AutoCheckpoint, DriveBlank

from constants.path.key_poses import kPoses, kPath
from constants.field import kHub


class CustomPathCommands:
    '''"Container" that has all the custom useful path commands'''
    def __init__(
        self,
        drivetrain : SwerveDriveTrain = None,
        roller_subsystem : IntakeRoller = None,
        transfer_subsystem : TransferSubsystem = None,
        shooter_subsystem : DualMotorShooter = None,
        hood_subsystem : ShooterHood = None,
        led_subsystem : CANdleLEDController = None,
        climb_subsyetem : None = None,
        ):
        
        self.drivetrain = drivetrain
        self.shooter_subsystem = shooter_subsystem
        self.transfer_subsystem = transfer_subsystem
        self.roller_subsystem = roller_subsystem
        self.hood_subsystem = hood_subsystem
        self.led_subsystem = led_subsystem
        
        self.make_teleop_paths()
        self.make_auto_paths()
    
    def opposite_pose_rotation(self, pose : Pose2d):
        return Pose2d(
            pose.X(),
            pose.Y(),
            Rotation2d(pose.rotation().radians() + pi)
        )
    
    def shoot_command_builder(self, shoot_time : float = 0):
        '''Makes a command that sits still and shoots for shoot_time seconds'''
        if shoot_time:
            return ParallelDeadlineGroup(
                WaitCommand(shoot_time),
                ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, self.hood_subsystem),
                IntakeRollerOscillate(self.roller_subsystem, self.drivetrain),
                DriveBlank(self.drivetrain)
            )
        else:
            return ParallelDeadlineGroup( # Shoot command that is meant to be in a ParallelDeadlineGroup (if no time is specified)
                ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, self.hood_subsystem),
                IntakeRollerOscillate(self.roller_subsystem, self.drivetrain),
            )
        
    def make_auto_paths(self):
        '''
        Each auto has "checkpoints" that could be made to wait until a certain time in Elastic
        '''
        self.auto_paths = {
        "_none" : InstantCommand(),
        "double_sweep" : SequentialCommandGroup(
            AutoCheckpoint(0),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_1
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_2
                    ).with_override_smoothing_radius(kPath.smoothing_radius_wide),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_3
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep
                    ).with_override_speed(kPath.intaking_speed),
                DriveToASpot(self.drivetrain, target_pose = (kPoses.double_sweep_4, kPoses.double_sweep_left_4)),
                DriveToASpot(self.drivetrain, target_pose = (kPoses.double_sweep_5, kPoses.double_sweep_left_5)
                    ).with_override_speed(kPath.bump_speed
                    ).with_override_max_acceleration(4),
            ),
            ParallelDeadlineGroup(
                DriveToASpot(self.drivetrain, target_pose = (kPoses.double_sweep_6, kPoses.double_sweep_left_6)
                    ).with_override_speed(kPath.while_shooting_speed
                    ).with_override_max_acceleration(1
                    ).with_end_tolerance(0.1),
                self.shoot_command_builder()
            ),
            ConditionalCommand(
                self.shoot_command_builder(3),
                SequentialCommandGroup(
                    ParallelDeadlineGroup(
                        DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_left_extra_6
                            ).with_override_speed(kPath.while_shooting_speed
                            ).with_override_max_acceleration(1
                            ).with_end_tolerance(0.1),
                        self.shoot_command_builder()
                    ),
                    self.shoot_command_builder(0.5),
                ),
                lambda : RobotZoneChecker.is_on_right_side(self.drivetrain.get_state().pose)
            ),
            AutoCheckpoint(1),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = (kPoses.double_sweep_7, kPoses.double_sweep_left_7)
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_8
                    ).with_override_smoothing_radius(kPath.smoothing_radius_wide),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_9
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep)
                    .with_override_speed(kPath.intaking_speed),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_10
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep)
                    .with_override_speed(kPath.intaking_speed),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_11
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep)
                    .with_override_speed(kPath.intaking_speed),
                DriveToASpot(self.drivetrain, target_pose = (kPoses.double_sweep_4, kPoses.double_sweep_left_4))
                    .with_override_speed(kPath.intaking_speed),
                DriveToASpot(self.drivetrain, target_pose = (kPoses.double_sweep_5, kPoses.double_sweep_left_5)
                    ).with_override_speed(kPath.bump_speed
                    ).with_override_max_acceleration(4),
            ),
            ParallelDeadlineGroup(
                DriveToASpot(self.drivetrain, target_pose = (kPoses.double_sweep_6, kPoses.double_sweep_left_6)
                    ).with_override_speed(kPath.while_shooting_speed
                    ).with_override_max_acceleration(1
                    ).with_end_tolerance(0.1),
                ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, self.hood_subsystem)
            ),
            ConditionalCommand(
                self.shoot_command_builder(3),
                SequentialCommandGroup(
                    ParallelDeadlineGroup(
                        DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_left_extra_6
                            ).with_override_speed(kPath.while_shooting_speed
                            ).with_override_max_acceleration(1
                            ).with_end_tolerance(0.1),
                        self.shoot_command_builder()
                    ),
                    self.shoot_command_builder(0.5),
                ),
                lambda : RobotZoneChecker.is_on_right_side(self.drivetrain.get_state().pose)
            ),
        )
        }
        
    def make_teleop_paths(self):
        self.teleop_paths = {
        "right_trench" : ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_right_trench).with_closest_180(),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_close_right_trench).with_closest_180(),
            ),
            ConditionalCommand(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.neutral_close_right_trench)).with_closest_180(),
                    DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.alliance_right_trench)).with_closest_180(),
                ),
                ConditionalCommand(
                    DriveToASpotSequence(
                        DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.opposing_right_trench)).with_closest_180(),
                        DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.alliance_right_trench)).with_closest_180(),
                    ),
                    cmd.none(),
                    lambda : RobotZoneChecker.is_in_opposing_alliance_zone(self.drivetrain.get_state().pose)
                ),
                lambda : RobotZoneChecker.is_in_neutral_zone(self.drivetrain.get_state().pose) 
            ),
            lambda : RobotZoneChecker.is_in_alliance_zone(self.drivetrain.get_state().pose)   
        ),
        "right_bump" : ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_right_bump).with_closest_180(),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_close_right_bump).with_override_speed(kPath.bump_speed).with_closest_180(),
            ),
            ConditionalCommand(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.neutral_close_right_bump)).with_closest_180(),
                    DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.alliance_right_bump)).with_override_speed(kPath.bump_speed).with_closest_180(),
                ),
                ConditionalCommand(
                    DriveToASpotSequence(
                        DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.opposing_right_bump)).with_closest_180(),
                        DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.neutral_far_right_bump)).with_override_speed(kPath.bump_speed).with_closest_180(),
                        DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.neutral_close_right_bump)).with_closest_180(),
                        DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.alliance_right_bump)).with_override_speed(kPath.bump_speed).with_closest_180(),
                    ),
                    cmd.none(),
                    lambda : RobotZoneChecker.is_in_opposing_alliance_zone(self.drivetrain.get_state().pose)
                ),
                lambda : RobotZoneChecker.is_in_neutral_zone(self.drivetrain.get_state().pose) 
            ),
            lambda : RobotZoneChecker.is_in_alliance_zone(self.drivetrain.get_state().pose)   
        ),
        "left_trench" : ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_left_trench).with_closest_180(),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_close_left_trench).with_closest_180(),
            ),
            ConditionalCommand(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.neutral_close_left_trench)).with_closest_180(),
                    DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.alliance_left_trench)).with_closest_180(),
                ),
                ConditionalCommand(
                    DriveToASpotSequence(
                        DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.opposing_left_trench)).with_closest_180(),
                        DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.alliance_left_trench)).with_closest_180(),
                    ),
                    cmd.none(),
                    lambda : RobotZoneChecker.is_in_opposing_alliance_zone(self.drivetrain.get_state().pose)
                ),
                lambda : RobotZoneChecker.is_in_neutral_zone(self.drivetrain.get_state().pose) 
            ),
            lambda : RobotZoneChecker.is_in_alliance_zone(self.drivetrain.get_state().pose)   
        ),
        "left_bump" : ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_left_bump).with_closest_180(),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_close_left_bump).with_override_speed(kPath.bump_speed).with_closest_180(),
            ),
            ConditionalCommand(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.neutral_close_left_bump)).with_closest_180(),
                    DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.alliance_left_bump)).with_override_speed(kPath.bump_speed).with_closest_180(),
                ),
                ConditionalCommand(
                    DriveToASpotSequence(
                        DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.opposing_left_bump)).with_closest_180(),
                        DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.neutral_far_left_bump)).with_override_speed(kPath.bump_speed).with_closest_180(),
                        DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.neutral_close_left_bump)).with_closest_180(),
                        DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.alliance_left_bump)).with_override_speed(kPath.bump_speed).with_closest_180(),
                    ),
                    cmd.none(),
                    lambda : RobotZoneChecker.is_in_opposing_alliance_zone
                ),
                lambda : RobotZoneChecker.is_in_neutral_zone(self.drivetrain.get_state().pose) 
            ),
            lambda : RobotZoneChecker.is_in_alliance_zone(self.drivetrain.get_state().pose)   
        ),
        "opposing_right_trench" : ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_right_trench).with_closest_180(),
                DriveToASpot(self.drivetrain, target_pose = kPoses.opposing_right_trench).with_closest_180(),
            ),
            ConditionalCommand(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_far_right_trench).with_closest_180(),
                    DriveToASpot(self.drivetrain, target_pose = kPoses.opposing_right_trench).with_closest_180(),
                ),
                cmd.none(),
                lambda : RobotZoneChecker.is_in_neutral_zone(self.drivetrain.get_state().pose) 
            ),
            lambda : RobotZoneChecker.is_in_alliance_zone(self.drivetrain.get_state().pose)  
        ),
        "opposing_right_bump" : ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_right_bump).with_closest_180(),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_close_right_bump).with_override_speed(kPath.bump_speed).with_closest_180(),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_far_right_bump).with_closest_180(),
                DriveToASpot(self.drivetrain, target_pose = kPoses.opposing_right_bump).with_override_speed(kPath.bump_speed).with_closest_180(),
            ),
            ConditionalCommand(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_far_right_bump).with_closest_180(),
                    DriveToASpot(self.drivetrain, target_pose = kPoses.opposing_right_bump).with_override_speed(kPath.bump_speed).with_closest_180(),
                ),
                cmd.none(),
                lambda : RobotZoneChecker.is_in_neutral_zone(self.drivetrain.get_state().pose) 
            ),
            lambda : RobotZoneChecker.is_in_alliance_zone(self.drivetrain.get_state().pose)  
        ),
        "opposing_left_trench" : ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_left_trench).with_closest_180(),
                DriveToASpot(self.drivetrain, target_pose = kPoses.opposing_left_trench).with_closest_180(),
            ),
            ConditionalCommand(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_far_left_trench).with_closest_180(),
                    DriveToASpot(self.drivetrain, target_pose = kPoses.opposing_left_trench).with_closest_180(),
                ),
                cmd.none(),
                lambda : RobotZoneChecker.is_in_neutral_zone(self.drivetrain.get_state().pose) 
            ),
            lambda : RobotZoneChecker.is_in_alliance_zone(self.drivetrain.get_state().pose)  
        ),
        "opposing_left_bump" : ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_left_bump).with_closest_180(),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_close_left_bump).with_override_speed(kPath.bump_speed).with_closest_180(),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_far_left_bump).with_closest_180(),
                DriveToASpot(self.drivetrain, target_pose = kPoses.opposing_left_bump).with_override_speed(kPath.bump_speed).with_closest_180(),
            ),
            ConditionalCommand(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_far_left_bump).with_closest_180(),
                    DriveToASpot(self.drivetrain, target_pose = kPoses.opposing_left_bump).with_override_speed(kPath.bump_speed).with_closest_180(),
                ),
                cmd.none(),
                lambda : RobotZoneChecker.is_in_neutral_zone(self.drivetrain.get_state().pose) 
            ),
            lambda : RobotZoneChecker.is_in_alliance_zone(self.drivetrain.get_state().pose)  
        ),
        "extake_left_trench" : ConditionalCommand(
            DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_close_left_trench).with_precise_values(),
            cmd.none(),
            lambda : RobotZoneChecker.is_in_neutral_zone(self.drivetrain.get_state().pose)  
        ),
        "extake_right_trench" : ConditionalCommand(
            DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_close_right_trench).with_precise_values(),
            cmd.none(),
            lambda : RobotZoneChecker.is_in_neutral_zone(self.drivetrain.get_state().pose)  
        ),
        "back_left_corner" : ConditionalCommand(
            DriveToASpot(self.drivetrain, target_pose = Pose2d(0.5, 7.77, Rotation2d(pi))).with_precise_values().with_override_speed(1),
            cmd.none(),
            lambda : RobotZoneChecker.is_in_alliance_zone(self.drivetrain.get_state().pose)  
        ),
        }
    
    def get_auto_paths(self):
        return self.auto_paths

    def get_teleop_paths(self):
        return self.teleop_paths
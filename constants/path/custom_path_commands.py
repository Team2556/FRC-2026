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
from commands.intake.roller_commands import IntakeRollerForward, IntakeRollerBackward, IntakeRollerOscillate
from commands.shooter.shooter_commands import EnableShooter, DisableShooter

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
    
    # Temporary solution to the BUG:
    # (cannot put a ConditionalAlignAndShoot command outside of the DriveToASpot parallel command environment without the last
    # DriveToASpot command in every DriveToASpotSequence not doing anything and staying still forever)
    def shoot_command_builder(self, shoot_time):
        '''Makes a command that sits still and shoots for shoot_time seconds'''
        return ParallelDeadlineGroup(
            WaitCommand(shoot_time),
            DriveToASpot(self.drivetrain, target_pose = Pose2d()
                ).with_parallel_commands(ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, self.hood_subsystem)
                ).with_override_speed(0
                ).with_override_rps(0
                ).with_end_tolerance(0),
        )
        
    def make_auto_paths(self):
        '''
        Auto path naming conventions I made up:
        First capital letter: starting zone (A - Alliance, N - Neutral)
        Second capital letter: finishing zone (A - Alliance, N - Neutral)
        '''
        self.auto_paths = {
        "_none" : lambda: InstantCommand(),
        "TN_short_sweep" : lambda: SequentialCommandGroup(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.short_sweep_1).with_parallel_commands(IntakeRollerForward(self.roller_subsystem)),
                DriveToASpot(self.drivetrain, target_pose = kPoses.short_sweep_2
                    ).with_override_smoothing_radius(kPath.smoothing_radius_wide),
                DriveToASpot(self.drivetrain, target_pose = kPoses.short_sweep_3
                    ).with_override_speed(kPath.intaking_speed),
                DriveToASpot(self.drivetrain, target_pose = kPoses.short_sweep_4
                    ).with_override_speed(kPath.intaking_speed * 1.3
                    ).with_override_rps(0.2),
                DriveToASpot(self.drivetrain, target_pose = kPoses.short_sweep_5
                    ).with_override_speed(kPath.intaking_speed * 1.5)
            ),
        ),
        "TN_wide_sweep" : lambda: SequentialCommandGroup(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.wide_sweep_1).with_parallel_commands(IntakeRollerForward(self.roller_subsystem)),
                DriveToASpot(self.drivetrain, target_pose = kPoses.wide_sweep_2
                    ).with_override_smoothing_radius(kPath.smoothing_radius_wide),
                DriveToASpot(self.drivetrain, target_pose = kPoses.wide_sweep_3
                    ).with_override_speed(kPath.intaking_speed),
                DriveToASpot(self.drivetrain, target_pose = kPoses.wide_sweep_4),
            ),
        ),
        "TA_close_sweep" : lambda: SequentialCommandGroup(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.close_sweep_1).with_parallel_commands(IntakeRollerForward(self.roller_subsystem)),
                DriveToASpot(self.drivetrain, target_pose = kPoses.close_sweep_2),
                DriveToASpot(self.drivetrain, target_pose = kPoses.close_sweep_3
                    ).with_override_speed(kPath.intaking_speed),
                DriveToASpot(self.drivetrain, target_pose = kPoses.close_sweep_4
                    ).with_override_speed(kPath.intaking_speed * 1.3),
            ),
        ),
        "TN_cleanup_sweep" : lambda: SequentialCommandGroup(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.cleanup_sweep_1).with_parallel_commands(IntakeRollerForward(self.roller_subsystem)),
                DriveToASpot(self.drivetrain, target_pose = kPoses.cleanup_sweep_2
                    ).with_override_smoothing_radius(kPath.smoothing_radius_wide),
                DriveToASpot(self.drivetrain, target_pose = kPoses.cleanup_sweep_3
                    ).with_override_speed(kPath.intaking_speed * 1.3),
                DriveToASpot(self.drivetrain, target_pose = kPoses.cleanup_sweep_4
                    ).with_override_speed(kPath.intaking_speed * 2
                    ).with_override_rps(0.3),
                DriveToASpot(self.drivetrain, target_pose = kPoses.cleanup_sweep_5
                    ).with_override_speed(kPath.intaking_speed * 2
                    ).with_override_rps(0.3),
            ),
        ),
        "NT_bump_shoot" : lambda: SequentialCommandGroup(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.bump_shoot_1
                    ).with_override_smoothing_radius(0.1
                    ).with_override_speed(kPath.intaking_speed * 1.2),
                DriveToASpot(self.drivetrain, target_pose = kPoses.bump_shoot_2
                    ).with_override_speed(kPath.bump_speed).with_precise_values(),
            ),
            ConditionalCommand( # different on left side
                SequentialCommandGroup(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.bump_shoot_5
                            ).with_parallel_commands(
                                ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, self.hood_subsystem),
                                # IntakeRollerOscillate(self.roller_subsystem)
                            ).with_override_speed(kPath.while_shooting_speed
                            ).with_goal_end_velocity(0.1
                            ).with_end_tolerance(0.1),
                    # self.shoot_command_builder(4),
                    self.shoot_command_builder(10), # temporary shoot forever
                ),
                SequentialCommandGroup(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.left_bump_shoot_1
                        ).with_parallel_commands(
                            ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, self.hood_subsystem),
                            # IntakeRollerOscillate(self.roller_subsystem)
                        ).with_override_speed(kPath.while_shooting_speed
                        ).with_goal_end_velocity(0.1
                        ).with_end_tolerance(0.1),
                    # self.shoot_command_builder(5.5),
                    self.shoot_command_builder(10), # shoot forever
                    DriveToASpot(self.drivetrain, target_pose = kPoses.bump_shoot_5),
                ),
                lambda : RobotZoneChecker.is_on_right_side(self.drivetrain.get_state().pose)
            )
        ),
        "NT_bump_move_shoot" : lambda: SequentialCommandGroup(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.bump_shoot_1
                    ).with_override_smoothing_radius(0.1
                    ).with_override_speed(kPath.intaking_speed * 1.2),
                DriveToASpot(self.drivetrain, target_pose = kPoses.bump_shoot_2
                    ).with_override_speed(kPath.bump_speed).with_precise_values(),
                DriveToASpot(self.drivetrain, target_pose = kPoses.bump_shoot_3
                            ).with_parallel_commands(
                                ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, self.hood_subsystem),
                                # IntakeRollerOscillate(self.roller_subsystem)
                            ).with_override_speed(kPath.while_shooting_speed
                            ).with_override_smoothing_radius(1.3),
                DriveToASpot(self.drivetrain, target_pose = kPoses.bump_shoot_4
                            ).with_parallel_commands(
                                ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, self.hood_subsystem),
                                # IntakeRollerOscillate(self.roller_subsystem)
                            ).with_override_speed(kPath.while_shooting_speed
                            ).with_goal_end_velocity(0.1),
            ),
            ConditionalCommand( # different on left side
                SequentialCommandGroup(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.bump_shoot_5
                            ).with_parallel_commands(
                                ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, self.hood_subsystem),
                                # IntakeRollerOscillate(self.roller_subsystem)
                            ).with_override_speed(kPath.while_shooting_speed
                            ).with_goal_end_velocity(0.1
                            ).with_end_tolerance(0.1),
                ),
                SequentialCommandGroup(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.left_bump_shoot_1
                        ).with_parallel_commands(
                            ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, self.hood_subsystem),
                            # IntakeRollerOscillate(self.roller_subsystem)
                        ).with_override_speed(kPath.while_shooting_speed
                        ).with_goal_end_velocity(0.1
                        ).with_end_tolerance(0.1),
                    self.shoot_command_builder(1),
                    DriveToASpot(self.drivetrain, target_pose = kPoses.bump_shoot_5),
                ),
                lambda : RobotZoneChecker.is_on_right_side(self.drivetrain.get_state().pose)
            )
        ),
        "NB_stay_bump_shoot" : lambda: SequentialCommandGroup(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.stay_bump_shoot_1
                    ).with_override_smoothing_radius(0.1),
                DriveToASpot(self.drivetrain, target_pose = kPoses.stay_bump_shoot_2
                    ).with_override_speed(kPath.bump_speed).with_precise_values(),
            ),
            self.shoot_command_builder(4),
            DriveToASpot(self.drivetrain, target_pose = kPoses.stay_bump_shoot_3),
        ),
        "BN_bump_middle_sweep" : lambda: SequentialCommandGroup(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.bump_middle_sweep_1
                    ).with_override_speed(kPath.bump_speed
                    ).with_override_smoothing_radius(0.5),
                DriveToASpot(self.drivetrain, target_pose = kPoses.bump_middle_sweep_2
                    ).with_override_smoothing_radius(0.5),
                DriveToASpot(self.drivetrain, target_pose = kPoses.bump_middle_sweep_3
                    ).with_override_speed(kPath.intaking_speed).with_precise_values(
                    ).with_override_smoothing_radius(0.1),
                DriveToASpot(self.drivetrain, target_pose = kPoses.bump_middle_sweep_4
                    ).with_override_speed(kPath.intaking_speed * 2
                    ).with_override_rps(0.3),
            ),
        ),
        "BN_bump_side_sweep" : lambda: SequentialCommandGroup(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.bump_side_sweep_1
                    ).with_override_speed(kPath.bump_speed),
                DriveToASpot(self.drivetrain, target_pose = kPoses.bump_side_sweep_2
                    ).with_override_rps(0.3),
                DriveToASpot(self.drivetrain, target_pose = kPoses.bump_side_sweep_3
                    ).with_override_speed(kPath.intaking_speed * 1.25),
                DriveToASpot(self.drivetrain, target_pose = kPoses.bump_side_sweep_4
                    ).with_override_speed(kPath.intaking_speed * 2
                    ).with_override_rps(0.4),
            ),
        ),
        "NN_sit_middle" : lambda: SequentialCommandGroup(
            DriveToASpot(self.drivetrain, target_pose = kPoses.sit_middle)
        ),
        # "NT_bump_half_shoot" : lambda: SequentialCommandGroup(
        #     DriveToASpotSequence(
        #         DriveToASpot(self.drivetrain, target_pose = kPoses.bump_half_shoot_1),
        #         DriveToASpot(self.drivetrain, target_pose = kPoses.bump_half_shoot_2
        #             ).with_override_speed(kPath.bump_speed).with_precise_values(),
        #     ),
        #     ParallelRaceGroup(
        #         IntakeRollerBackward(self.roller_subsystem),
        #         WaitCommand(1.3),
        #     ),
        #     ConditionalCommand( # different on left side
        #         SequentialCommandGroup(
        #             DriveToASpot(self.drivetrain, target_pose = kPoses.bump_half_shoot_3
        #                     ).with_parallel_commands(
        #                         ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, self.hood_subsystem),
        #                         # IntakeRollerOscillate(self.roller_subsystem)
        #                     ).with_override_speed(kPath.while_shooting_speed
        #                     ).with_goal_end_velocity(0.1
        #                     ).with_end_tolerance(0.1),
        #             self.shoot_command_builder(3),
        #         ),
        #         SequentialCommandGroup(
        #             DriveToASpot(self.drivetrain, target_pose = kPoses.left_bump_shoot_1
        #                 ).with_parallel_commands(
        #                     ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, self.hood_subsystem),
        #                     # IntakeRollerOscillate(self.roller_subsystem)
        #                 ).with_override_speed(kPath.while_shooting_speed
        #                 ).with_goal_end_velocity(0.1
        #                 ).with_end_tolerance(0.1),
        #             self.shoot_command_builder(3),
        #             DriveToASpot(self.drivetrain, target_pose = kPoses.bump_half_shoot_3),
        #         ),
        #         lambda : RobotZoneChecker.is_on_right_side(self.drivetrain.get_state().pose)
        #     )
        # ),
        # "NN_trench_extake" : lambda: SequentialCommandGroup(
        #     DriveToASpot(self.drivetrain, target_pose = kPoses.trench_extake_1
        #         ).with_override_speed(kPath.auto_path_speed),
        #     ParallelRaceGroup(
        #         IntakeRollerBackward(self.roller_subsystem),
        #         WaitCommand(2)
        #     )
        # ),
        "template" : lambda: SequentialCommandGroup(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.short_sweep_1),
            ),
            self.shoot_command_builder(3)
        ),
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
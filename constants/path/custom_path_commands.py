from math import pi

from commands2 import cmd, ConditionalCommand, WaitCommand, SequentialCommandGroup, ParallelRaceGroup, ParallelDeadlineGroup, InstantCommand, RepeatCommand, PrintCommand

from wpimath.geometry import Pose2d, Rotation2d

from util.robot_zone_checker import RobotZoneChecker
from util.flip_util import FlipUtil

from subsystems.drivetrain.drivetrain import SwerveDriveTrain
from subsystems.shooter.shooter_hood import ShooterHood
from subsystems.shooter.dual_shooter import DualMotorShooter
from subsystems.trasnfer.transfer_subsystem import TransferSubsystem
from subsystems.intake.intake_pivot import IntakePivot
from subsystems.intake.intake_roller import IntakeRoller
from subsystems.led.LED_controller import CANdleLEDController

from commands.path_commands.drive_to_a_spot import DriveToASpot
from commands.path_commands.drive_to_a_spot_sequence import DriveToASpotSequence
from commands.auto_align.align_with_controller import ConditionalAlignAndShoot
from commands.auto_align.alignio import AlignIntakeToVelocity
from commands.intake.roller_commands import IntakeRollerForwardInstant, IntakeRollerBackward, IntakeRollerOscillate
from commands.intake.pivot_commands import IntakePivotForward, IntakePivotReverse
from commands.shooter.shooter_commands import EnableShooter, DisableShooter
from commands.path_commands.auto_helpers import AutoCheckpoint, DriveBlank
from commands.drive import drive_commands

from constants.path.key_poses import kPoses, kPath
from constants.field import kHub


class CustomPathCommands:
    '''"Container" that has all the custom useful path commands'''
    def __init__(
        self,
        drivetrain : SwerveDriveTrain = None,
        pivot_subsystem : IntakePivot = None,
        roller_subsystem : IntakeRoller = None,
        transfer_subsystem : TransferSubsystem = None,
        shooter_subsystem : DualMotorShooter = None,
        hood_subsystem : ShooterHood = None,
        led_subsystem : CANdleLEDController = None,
        ):
        
        self.drivetrain = drivetrain
        self.shooter_subsystem = shooter_subsystem
        self.transfer_subsystem = transfer_subsystem
        self.pivot_subsystem = pivot_subsystem
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
            drive_commands.initial_pose(self.drivetrain, kPoses.initial_pose_trench),
            AutoCheckpoint(0),
            IntakeRollerForwardInstant(self.roller_subsystem),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_1
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_2
                    ).with_override_smoothing_radius(kPath.smoothing_radius_wide),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_3
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep * 0.3
                    ).with_override_speed(kPath.intaking_speed),
                DriveToASpot(self.drivetrain, target_pose = (kPoses.double_sweep_4, kPoses.double_sweep_left_4)),
                DriveToASpot(self.drivetrain, target_pose = (kPoses.double_sweep_5, kPoses.double_sweep_left_5)
                    ).with_override_speed(kPath.bump_speed
                    ).with_override_max_acceleration(4),
            ),
            ParallelDeadlineGroup(
                DriveToASpot(self.drivetrain, target_pose = (kPoses.double_sweep_6, kPoses.double_sweep_left_6)
                    ).with_override_speed(kPath.while_shooting_speed * 1.5
                    ).with_override_max_acceleration(1
                    ).with_end_tolerance(0.1),
                self.shoot_command_builder()
            ),
            ConditionalCommand(
                self.shoot_command_builder(6),
                SequentialCommandGroup(
                    ParallelDeadlineGroup(
                        DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_left_extra_6
                            ).with_override_speed(kPath.while_shooting_speed
                            ).with_override_max_acceleration(1
                            ).with_end_tolerance(0.1),
                        self.shoot_command_builder()
                    ),
                    self.shoot_command_builder(4),
                ),
                lambda : RobotZoneChecker.is_on_right_side(self.drivetrain.get_state().pose)
            ),
            IntakeRollerForwardInstant(self.roller_subsystem),
            AutoCheckpoint(1),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = (kPoses.double_sweep_7, kPoses.double_sweep_left_7)
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep
                    ).with_override_speed(kPath.auto_path_speed * 0.67),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_8
                    ).with_override_smoothing_radius(kPath.smoothing_radius_wide
                    ).with_override_speed(kPath.auto_path_speed * 0.80),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_9
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep)
                    .with_override_speed(kPath.auto_path_speed * 0.70),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_10
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep)
                    .with_override_speed(kPath.intaking_speed * 1.5)
                    .with_override_rps(kPath.intaking_speed * 0.6),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_11
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep)
                    .with_override_speed(kPath.auto_path_speed * 0.5),
                DriveToASpot(self.drivetrain, target_pose = (kPoses.double_sweep_12, kPoses.double_sweep_left_12)),
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
                self.shoot_command_builder(7),
                SequentialCommandGroup(
                    ParallelDeadlineGroup(
                        DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_left_extra_6
                            ).with_override_speed(kPath.while_shooting_speed
                            ).with_override_max_acceleration(1
                            ).with_end_tolerance(0.1),
                        self.shoot_command_builder()
                    ),
                    self.shoot_command_builder(5),
                ),
                lambda : RobotZoneChecker.is_on_right_side(self.drivetrain.get_state().pose)
            ),
        ),
        "double_bonus_sweep" : SequentialCommandGroup(
            drive_commands.initial_pose(self.drivetrain, kPoses.initial_pose_trench),
            AutoCheckpoint(0),
            IntakeRollerForwardInstant(self.roller_subsystem),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_7
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_8
                    ).with_override_smoothing_radius(kPath.smoothing_radius_wide
                    ).with_override_speed(kPath.auto_path_speed * 0.80),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_9
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep)
                    .with_override_speed(kPath.intaking_speed),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_10
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep)
                    .with_override_speed(kPath.intaking_speed)
                    .with_override_rps(kPath.intaking_speed * 0.4),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_11
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep)
                    .with_override_speed(kPath.auto_path_speed * 0.5),
                DriveToASpot(self.drivetrain, target_pose = (kPoses.double_sweep_12, kPoses.double_sweep_left_12)),
                DriveToASpot(self.drivetrain, target_pose = (kPoses.double_sweep_5, kPoses.double_sweep_left_5)
                    ).with_override_speed(kPath.bump_speed
                    ).with_override_max_acceleration(4),
            ),
            ParallelDeadlineGroup(
                DriveToASpot(self.drivetrain, target_pose = (kPoses.middle_sweep_7)
                    ).with_override_speed(kPath.while_shooting_speed
                    ).with_override_max_acceleration(1
                    ).with_end_tolerance(0.1),
                ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, self.hood_subsystem)
            ),
            self.shoot_command_builder(3),
            AutoCheckpoint(1),
            IntakeRollerForwardInstant(self.roller_subsystem),
            ConditionalCommand(
                SequentialCommandGroup(
                    DriveToASpotSequence(
                        DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_depot_8_before),
                        DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_depot_8_before_2),
                        DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_depot_8
                            ).with_override_speed(1),
                        DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_depot_9
                            ).with_override_speed(kPath.depot_intake_speed),
                        DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_depot_10),
                    ),
                    self.shoot_command_builder(5)
                ),
                SequentialCommandGroup(
                    DriveToASpotSequence(
                        DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_depot_left_8
                            ).with_override_speed(1),
                        DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_depot_left_9
                            ).with_override_speed(kPath.depot_intake_speed),
                        DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_depot_left_10),
                    ),
                    self.shoot_command_builder(5)
                ),
                lambda : RobotZoneChecker.is_on_right_side(self.drivetrain.get_state().pose)
            ),
        ),
        "trench_double_sweep" : SequentialCommandGroup(
            drive_commands.initial_pose(self.drivetrain, kPoses.initial_pose_trench),
            AutoCheckpoint(0),
            IntakeRollerForwardInstant(self.roller_subsystem),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_1
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_2
                    ).with_override_smoothing_radius(kPath.smoothing_radius_wide),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_3
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep * 0.3
                    ).with_override_speed(kPath.intaking_speed),
                DriveToASpot(self.drivetrain, target_pose = (kPoses.trench_double_sweep_4, kPoses.trench_double_sweep_left_4)),
                DriveToASpot(self.drivetrain, target_pose = (kPoses.trench_double_sweep_5, kPoses.trench_double_sweep_left_5)
                    ).with_override_max_acceleration(4
                    ).with_end_tolerance(0.15),
            ),
            self.shoot_command_builder(5),
            IntakeRollerForwardInstant(self.roller_subsystem),
            AutoCheckpoint(1),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = (kPoses.double_sweep_7, kPoses.double_sweep_left_7)
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep
                    ).with_override_speed(kPath.auto_path_speed * 0.67),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_8
                    ).with_override_smoothing_radius(kPath.smoothing_radius_wide
                    ).with_override_speed(kPath.auto_path_speed * 0.80),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_9
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep
                    ).with_override_speed(kPath.intaking_speed),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_10
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep)
                    .with_override_speed(kPath.intaking_speed)
                    .with_override_rps(kPath.intaking_speed * 0.4),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_sweep_11
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep)
                    .with_override_speed(kPath.auto_path_speed * 0.5),
                DriveToASpot(self.drivetrain, target_pose = (kPoses.trench_double_sweep_extra_12, kPoses.trench_double_sweep_left_extra_12)),
                DriveToASpot(self.drivetrain, target_pose = (kPoses.trench_double_sweep_4, kPoses.trench_double_sweep_left_4)),
                DriveToASpot(self.drivetrain, target_pose = (kPoses.trench_double_sweep_5, kPoses.trench_double_sweep_left_5)
                    ).with_end_tolerance(0.15
                    ).with_override_max_acceleration(4),
            ),
            self.shoot_command_builder(7),
        ),
        "middle_sweep" : SequentialCommandGroup(
            drive_commands.initial_pose(self.drivetrain, kPoses.initial_pose_bump),
            DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_1),
            IntakeRollerForwardInstant(self.roller_subsystem),
            AutoCheckpoint(0),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_2
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep
                    ).with_override_speed(2),
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_3
                    ).with_override_speed(kPath.intaking_speed),
            ),
            AutoCheckpoint(1),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_4
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep
                    ).with_override_speed(kPath.auto_path_speed * 0.7
                    ).with_override_rps(0.3),
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_5
                    ).with_override_speed(kPath.auto_path_speed * 0.7
                    ).with_override_rps(0.3),
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_6
                    ).with_override_speed(kPath.bump_speed),
            ),
            # ParallelDeadlineGroup(
            #     DriveToASpot(self.drivetrain, target_pose = (kPoses.middle_sweep_7, kPoses.middle_sweep_depot_left_7)
            #         ).with_override_speed(kPath.while_shooting_speed
            #         ).with_override_max_acceleration(1
            #         ).with_end_tolerance(0.1),
            #     ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, self.hood_subsystem)
            # ),
            self.shoot_command_builder(8),
            AutoCheckpoint(2),
            IntakeRollerForwardInstant(self.roller_subsystem),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_8),
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_9
                    ).with_override_speed(1.5
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep),
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_10
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep
                    ).with_override_speed(2),
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_11
                    ).with_override_speed(kPath.intaking_speed),
            ),
        ),
        "middle_sweep_depot" : SequentialCommandGroup(
            drive_commands.initial_pose(self.drivetrain, kPoses.initial_pose_bump),
            DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_1),
            IntakeRollerForwardInstant(self.roller_subsystem),
            AutoCheckpoint(0),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_2
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep
                    ).with_override_speed(2),
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_3
                    ).with_override_speed(kPath.intaking_speed),
            ),
            AutoCheckpoint(1),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_4
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep
                    ).with_override_speed(kPath.auto_path_speed * 0.7
                    ).with_override_rps(0.3),
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_5
                    ).with_override_speed(kPath.auto_path_speed * 0.7
                    ).with_override_rps(0.3),
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_6
                    ).with_override_speed(kPath.bump_speed),
            ),
            ParallelDeadlineGroup(
                DriveToASpot(self.drivetrain, target_pose = (kPoses.middle_sweep_7, kPoses.middle_sweep_depot_left_7)
                    ).with_override_speed(kPath.while_shooting_speed
                    ).with_override_max_acceleration(1
                    ).with_end_tolerance(0.1),
                ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, self.hood_subsystem)
            ),
            self.shoot_command_builder(3),
            AutoCheckpoint(2),
            IntakeRollerForwardInstant(self.roller_subsystem),
            ConditionalCommand(
                SequentialCommandGroup(
                    DriveToASpotSequence(
                        DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_depot_8_before),
                        DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_depot_8_before_2),
                        DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_depot_8
                            ).with_override_speed(1),
                        DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_depot_9
                            ).with_override_speed(kPath.depot_intake_speed),
                        DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_depot_10),
                    ),
                    self.shoot_command_builder(5)
                ),
                SequentialCommandGroup(
                    DriveToASpotSequence(
                        DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_depot_left_8
                            ).with_override_speed(1),
                        DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_depot_left_9
                            ).with_override_speed(kPath.depot_intake_speed),
                        DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_depot_left_10),
                    ),
                    self.shoot_command_builder(5)
                ),
                lambda : RobotZoneChecker.is_on_right_side(self.drivetrain.get_state().pose)
            ),
        ),
        "middle_support" : SequentialCommandGroup(
            drive_commands.initial_pose(self.drivetrain, kPoses.initial_pose_bump),
            DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_1),
            IntakeRollerForwardInstant(self.roller_subsystem),
            AutoCheckpoint(0),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_2
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep
                    ).with_override_speed(2),
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_3
                    ).with_override_speed(kPath.intaking_speed),
            ),
            AutoCheckpoint(1),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_support_4
                    ).with_override_speed(1.3).with_override_rps(0.25),
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_support_4_extra
                    ).with_override_speed(1.3).with_override_rps(0.25),
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_support_4_extra_2
                    ).with_override_speed(1.3).with_override_rps(0.25),
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_support_4_extra_3
                    ).with_override_speed(1.3).with_override_rps(0.25),
            ),
            AutoCheckpoint(2),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_4
                    ).with_override_smoothing_radius(kPath.smoothing_radius_auto_sweep
                    ).with_override_speed(kPath.auto_path_speed * 0.8
                    ).with_override_rps(0.3),
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_5
                    ).with_override_speed(kPath.auto_path_speed * 0.8
                    ).with_override_rps(0.3),
                DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_6
                    ).with_override_speed(kPath.bump_speed),
                DriveToASpot(self.drivetrain, target_pose = (kPoses.middle_sweep_7, kPoses.middle_sweep_depot_left_7)),
            ),
            self.shoot_command_builder(5),
        ),
        "just_depot" : SequentialCommandGroup(
            drive_commands.initial_pose(self.drivetrain, kPoses.initial_pose_center),
            AutoCheckpoint(0),
            IntakeRollerForwardInstant(self.roller_subsystem),
            SequentialCommandGroup(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_depot_left_8
                        ).with_override_speed(2),
                    DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_depot_left_9
                        ).with_override_speed(kPath.depot_intake_speed),
                    DriveToASpot(self.drivetrain, target_pose = kPoses.middle_sweep_depot_left_10),
                ),
                self.shoot_command_builder(10)
            ),
        ),
        }
        
    def make_teleop_paths(self):
        self.teleop_paths = {
        "right_trench" : ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_right_trench).with_closest_180().with_override_max_acceleration(kPath.trench_path_max_acceleration),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_close_right_trench).with_closest_180(),
            ),
            ConditionalCommand(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.neutral_close_right_trench)).with_closest_180().with_override_max_acceleration(kPath.trench_path_max_acceleration),
                    DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.alliance_right_trench)).with_closest_180(),
                ),
                ConditionalCommand(
                    DriveToASpotSequence(
                        DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.opposing_right_trench)).with_closest_180().with_override_max_acceleration(kPath.trench_path_max_acceleration),
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
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_left_trench).with_closest_180().with_override_max_acceleration(kPath.trench_path_max_acceleration),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_close_left_trench).with_closest_180(),
            ),
            ConditionalCommand(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.neutral_close_left_trench)).with_closest_180().with_override_max_acceleration(kPath.trench_path_max_acceleration),
                    DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.alliance_left_trench)).with_closest_180(),
                ),
                ConditionalCommand(
                    DriveToASpotSequence(
                        DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.opposing_left_trench)).with_closest_180().with_override_max_acceleration(kPath.trench_path_max_acceleration),
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
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_right_trench).with_closest_180().with_override_max_acceleration(kPath.trench_path_max_acceleration),
                DriveToASpot(self.drivetrain, target_pose = kPoses.opposing_right_trench).with_closest_180(),
            ),
            ConditionalCommand(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_far_right_trench).with_closest_180().with_override_max_acceleration(kPath.trench_path_max_acceleration),
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
                DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_left_trench).with_closest_180().with_override_max_acceleration(kPath.trench_path_max_acceleration),
                DriveToASpot(self.drivetrain, target_pose = kPoses.opposing_left_trench).with_closest_180(),
            ),
            ConditionalCommand(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_far_left_trench).with_closest_180().with_override_max_acceleration(kPath.trench_path_max_acceleration),
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
        }
    
    def get_auto_paths(self):
        return self.auto_paths

    def get_teleop_paths(self):
        return self.teleop_paths
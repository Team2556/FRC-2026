from commands2 import cmd, ConditionalCommand, WaitCommand, SequentialCommandGroup, ParallelRaceGroup
from wpimath.geometry import Pose2d, Rotation2d

from commands.path_commands.drive_to_a_spot import DriveToASpot
from commands.path_commands.drive_to_a_spot_sequence import DriveToASpotSequence
from commands.auto_align.path_with_align import DriveWithAlign
from commands.auto_align.align_with_controller import ConditionalAlignAndShoot
from commands.drive.drive_commands import InitialPose, AutoDrive
from commands.intake.intake_commands import IntakeCommandManualForwardAuto, IntakeCommandManualReverseAuto, IntakeRollerForward
from commands.shooter.shooter_commands import EnableShooter, DisableShooter

from util.robot_zone_checker import RobotZoneChecker
from util.flip_util import FlipUtil

from constants.key_poses import kPoses
from constants.field import kHub

from subsystems.drivetrain.drivetrain import SwerveDriveTrain
from subsystems.shooter.shooter_hood import ShooterHood
from subsystems.shooter.dual_shooter import DualMotorShooter
from subsystems.trasnfer.transfer_subsystem import TransferSubsystem
from subsystems.intake.intake import IntakeSubsystem
from subsystems.led.LED_controller import CANdleLEDController

from math import pi

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
        
        self.make_path_commands()
        self.make_autos()
    
    def opposite_pose_rotation(self, pose : Pose2d):
        return Pose2d(
            pose.X(),
            pose.Y(),
            Rotation2d(pose.rotation().radians() + pi)
        )
        
    def make_autos(self):
        speed_intaking_middle = 1
        
        return {
        "simple_right" : SequentialCommandGroup(
            InitialPose(self.drivetrain, pose=kPoses.simple_right0),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.simple_right1),
                DriveToASpot(self.drivetrain, target_pose = kPoses.simple_right2),
            ),
            ParallelRaceGroup(
                ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, 
                                         self.hood_subsystem, self.led_subsystem),
                AutoDrive(self.drivetrain),
                WaitCommand(5)
            )
        ),
        "neutral_grab_right" : SequentialCommandGroup(
            InitialPose(self.drivetrain, pose=kPoses.neutral_grab_right0),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_grab_right1
                             ).with_parallel_command(IntakeCommandManualForwardAuto(self.intake_subsystem)),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_grab_right2),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_grab_right3).with_override_speed(speed_intaking_middle),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_grab_right4
                             ).with_parallel_command(EnableShooter(self.shooter_subsystem)),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_grab_right5),
                DriveWithAlign(kHub.POS, self.drivetrain, target_pose = kPoses.neutral_grab_right6),
            ),
            ParallelRaceGroup(
                ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, 
                                         self.hood_subsystem, self.led_subsystem),
                AutoDrive(self.drivetrain),
                WaitCommand(6)
            )
        ),
        "neutral_grab_left" : SequentialCommandGroup(
            InitialPose(self.drivetrain, pose=kPoses.neutral_grab_left0),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_grab_left1
                             ).with_parallel_command(IntakeCommandManualForwardAuto(self.intake_subsystem)),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_grab_left2),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_grab_left3).with_override_speed(speed_intaking_middle),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_grab_left4
                             ).with_parallel_command(EnableShooter(self.shooter_subsystem)),
                DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_grab_left5),
                DriveWithAlign(kHub.POS, self.drivetrain, target_pose = kPoses.neutral_grab_left6),
            ),
            ParallelRaceGroup(
                ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, 
                                         self.hood_subsystem, self.led_subsystem),
                AutoDrive(self.drivetrain),
                WaitCommand(6)
            )
        ),
        "double_grab_right" : SequentialCommandGroup(
            InitialPose(self.drivetrain, pose=kPoses.double_grab_right0),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_grab_right1
                             ).with_parallel_command(IntakeCommandManualForwardAuto(self.intake_subsystem)),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_grab_right2),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_grab_right3).with_override_speed(speed_intaking_middle),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_grab_right4
                             ).with_parallel_command(EnableShooter(self.shooter_subsystem)),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_grab_right5),
                DriveWithAlign(kHub.POS, self.drivetrain, target_pose = kPoses.double_grab_right6),
            ),
            ParallelRaceGroup(
                ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, 
                                         self.hood_subsystem, self.led_subsystem),
                AutoDrive(self.drivetrain),
                WaitCommand(6)
            ),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_grab_right7),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_grab_right8
                             ).with_parallel_command(IntakeCommandManualForwardAuto(self.intake_subsystem)),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_grab_right9),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_grab_right10).with_override_speed(speed_intaking_middle),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_grab_right11
                             ).with_parallel_command(EnableShooter(self.shooter_subsystem)),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_grab_right12),
                DriveWithAlign(kHub.POS, self.drivetrain, target_pose = kPoses.double_grab_right13),
            ),
            ParallelRaceGroup(
                ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, 
                                         self.hood_subsystem, self.led_subsystem),
                AutoDrive(self.drivetrain),
                WaitCommand(6)
            ),
        ),
        "double_grab_left" : SequentialCommandGroup(
            InitialPose(self.drivetrain, pose=kPoses.double_grab_left0),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_grab_left1
                             ).with_parallel_command(IntakeCommandManualForwardAuto(self.intake_subsystem)),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_grab_left2),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_grab_left3).with_override_speed(speed_intaking_middle),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_grab_left4
                             ).with_parallel_command(EnableShooter(self.shooter_subsystem)),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_grab_left5),
                DriveWithAlign(kHub.POS, self.drivetrain, target_pose = kPoses.double_grab_left6),
            ),
            ParallelRaceGroup(
                ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, 
                                         self.hood_subsystem, self.led_subsystem),
                AutoDrive(self.drivetrain),
                WaitCommand(6)
            ),
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_grab_left7),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_grab_left8
                             ).with_parallel_command(IntakeCommandManualForwardAuto(self.intake_subsystem)),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_grab_left9),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_grab_left10).with_override_speed(speed_intaking_middle),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_grab_left11
                             ).with_parallel_command(EnableShooter(self.shooter_subsystem)),
                DriveToASpot(self.drivetrain, target_pose = kPoses.double_grab_left12),
                DriveWithAlign(kHub.POS, self.drivetrain, target_pose = kPoses.double_grab_left13),
            ),
            ParallelRaceGroup(
                ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, 
                                         self.hood_subsystem, self.led_subsystem),
                AutoDrive(self.drivetrain),
                WaitCommand(6)
            ),
            
        ),
        "depot_grab" : SequentialCommandGroup(
            InitialPose(self.drivetrain, pose=kPoses.depot_grab0),
            DriveWithAlign(kHub.POS, self.drivetrain, target_pose = kPoses.depot_grab1
                           ).with_parallel_command(IntakeRollerForward(self.intake_subsystem)),
            ParallelRaceGroup(
                ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, 
                                         self.hood_subsystem, self.led_subsystem),
                AutoDrive(self.drivetrain),
                WaitCommand(4)
            ),
            DriveToASpot(self.drivetrain, target_pose = kPoses.depot_grab2
                         ).with_parallel_command(IntakeCommandManualForwardAuto(self.intake_subsystem)
                         ).with_precise_values(),
            ParallelRaceGroup(
                DriveToASpot(self.drivetrain, target_pose = kPoses.depot_grab3).with_precise_values(),
                WaitCommand(5)
            ),
            DriveWithAlign(kHub.POS, self.drivetrain, target_pose = kPoses.depot_grab4),
            ParallelRaceGroup(
                ConditionalAlignAndShoot(self.drivetrain, self.shooter_subsystem, self.transfer_subsystem, 
                                         self.hood_subsystem, self.led_subsystem),
                AutoDrive(self.drivetrain),
                WaitCommand(4)
            ),
        ),
        }
        
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
                DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.alliance_zone_right_trench)),
                DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.neutral_zone_right_trench)).with_goal_end_velocity(0),
            ),
            ConditionalCommand(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_right_trench),
                    DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_right_trench).with_goal_end_velocity(0),
                ),
                cmd.none(),
                lambda : RobotZoneChecker.is_in_neutral_zone(self.drivetrain.get_state().pose) 
            ),
            lambda : RobotZoneChecker.is_in_alliance_zone(self.drivetrain.get_state().pose)   
        )
        
        self.right_bump = ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.alliance_zone_right_bump)),
                DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.neutral_zone_right_bump)).with_goal_end_velocity(0),
            ),
            ConditionalCommand(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_right_bump),
                    DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_right_bump).with_goal_end_velocity(0),
                ),
                cmd.none(),
                lambda : RobotZoneChecker.is_in_neutral_zone(self.drivetrain.get_state().pose) 
            ),
            lambda : RobotZoneChecker.is_in_alliance_zone(self.drivetrain.get_state().pose)   
        )
        
        self.left_trench = ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.alliance_zone_left_trench)),
                DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.neutral_zone_left_trench)).with_goal_end_velocity(0),
            ),
            ConditionalCommand(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_left_trench),
                    DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_left_trench).with_goal_end_velocity(0),
                ),
                cmd.none(),
                lambda : RobotZoneChecker.is_in_neutral_zone(self.drivetrain.get_state().pose) 
            ),
            lambda : RobotZoneChecker.is_in_alliance_zone(self.drivetrain.get_state().pose)   
        )
        
        self.left_bump = ConditionalCommand(
            DriveToASpotSequence(
                DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.alliance_zone_left_bump)),
                DriveToASpot(self.drivetrain, target_pose = self.opposite_pose_rotation(kPoses.neutral_zone_left_bump)).with_goal_end_velocity(0),
            ),
            ConditionalCommand(
                DriveToASpotSequence(
                    DriveToASpot(self.drivetrain, target_pose = kPoses.neutral_zone_left_bump),
                    DriveToASpot(self.drivetrain, target_pose = kPoses.alliance_zone_left_bump).with_goal_end_velocity(0),
                ),
                cmd.none(),
                lambda : RobotZoneChecker.is_in_neutral_zone(self.drivetrain.get_state().pose) 
            ),
            lambda : RobotZoneChecker.is_in_alliance_zone(self.drivetrain.get_state().pose)   
        )
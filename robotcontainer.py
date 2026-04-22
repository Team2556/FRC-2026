#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from wpilib import SmartDashboard

from commands2 import ParallelCommandGroup, cmd, InterruptionBehavior

from util.custom_controller import XboxController
from util.send_fms_data import SendFMSData
from util.auto_chooser import AutoBuilder
from util.flip_util import FlipUtil

from subsystems.drivetrain import drivetrain
from subsystems.vision import rpi_vision
from subsystems.vision import multi_limelight
from subsystems.intake.intake_pivot import IntakePivot
from subsystems.intake.intake_roller import IntakeRoller
from subsystems.trasnfer.transfer_subsystem import TransferSubsystem
from subsystems.shooter.shooter_hood import ShooterHood
from subsystems.led.LED_controller import CANdleLEDController
from subsystems.shooter.dual_shooter import DualMotorShooter

from commands.auto_align import align_with_controller
from commands.drive import drive_commands
from commands.vision import vision_odometry
from constants.path import custom_path_commands, key_poses
from commands.transfer.run_transfer_motors import RunTransferCommand, ReverseTransferCommand, SpindexOnlyCommand
from commands.intake.pivot_commands import (
    IntakePivotDefaultCommand,
    IntakePivotForward,
    IntakePivotReverse,
)
from commands.intake.roller_commands import (
    IntakeRollerDefaultCommand,
    IntakeRollerForward,
    IntakeRollerBackward,
    IntakeRollerOscillate
)
from commands.shooter import shooter_commands, hood_commands

from constants.vision import kCamera

# from magnolia_vision import (
#     magnolia_limelight_helpers,
#     magnolia_limelight_commands,
#     magnolia_limelight_constants,
#     magnolia_limelight_subsystem
# )


class RobotContainer:
    def __init__(self) -> None:
        self._controller_1 = XboxController(port=0).with_deadband(0).with_power(1).with_mult(0)
        self._controller_2 = XboxController(port=1).with_deadband(0).with_power(1).with_mult(0)

        self._drivetrain = drivetrain.SwerveDriveTrain()

        self.intake_pivot = IntakePivot()
        self.intake_roller = IntakeRoller()
        self.transfer_subsystem = TransferSubsystem()
        self.shooter_subsystem = DualMotorShooter()
        self.hood_subsystem = ShooterHood()
        # self.climb_subsystem = ClimbSubsystem()


        self.rpi_vision = rpi_vision.RpiVision()
        self.mono_vision = multi_limelight.Vision(kCamera.BACK_LL, kCamera.SHOOTER_LL)
        # self.magnolia_vision_subsystem = magnolia_limelight_subsystem.Vision(magnolia_limelight_constants.kCamera.SHOOTER_LL, magnolia_limelight_constants.kCamera.BACK_LL)
        # self.LED_controller = CANdleLEDController()

        self.custom_path_commands = custom_path_commands.CustomPathCommands(
            self._drivetrain,
            pivot_subsystem=self.intake_pivot,
            roller_subsystem=self.intake_roller,
            transfer_subsystem=self.transfer_subsystem,
            shooter_subsystem=self.shooter_subsystem,
            hood_subsystem=self.hood_subsystem,
        )

        self.auto_chooser = AutoBuilder(
            self.custom_path_commands.get_auto_paths()
        )

        self.time_manager = SendFMSData()

        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:

        # -------------------------------------------------------------------
        # Default commands
        # -------------------------------------------------------------------

        # self.magnolia_vision_subsystem.setDefaultCommand(
        #     magnolia_limelight_commands.UpdateOdometry(self.magnolia_vision_subsystem, self._drivetrain)
        # )
        self.mono_vision.setDefaultCommand(vision_odometry.UpdateOdometry(self.mono_vision, self._drivetrain))
        self.shooter_subsystem.setDefaultCommand(
            shooter_commands.DisableShooter(self.shooter_subsystem)
        )
        self.intake_pivot.setDefaultCommand(
            IntakePivotDefaultCommand(self.intake_pivot, self._drivetrain)
        )
        self.intake_roller.setDefaultCommand(
            IntakeRollerDefaultCommand(self.intake_roller, self.intake_pivot, self._drivetrain)
        )
        self.hood_subsystem.setDefaultCommand(
            hood_commands.UpdateHoodPositionVariable(
                self.hood_subsystem, self._drivetrain
            )
        )

        # -------------------------------------------------------------------
        # CONTROLLER 1
        # -------------------------------------------------------------------

        self._drivetrain.setDefaultCommand(
            drive_commands.joystick_drive(
                self._drivetrain,
                lambda: -self._controller_1.getLeftY(),
                lambda: -self._controller_1.getLeftX(),
                lambda: -self._controller_1.getRightX(),
            )
        )

        self._controller_1.rightTrigger().whileTrue(
            ParallelCommandGroup(
                cmd.runEnd(
                    lambda: self._drivetrain.set_modifiers(
                        drivetrain.SwerveDriveTrain.SLOW_ROTATE
                    ),
                    lambda: self._drivetrain.reset_modifiers(),
                ),
                align_with_controller.ConditionalAlignAndShoot(
                    self._drivetrain,
                    self.shooter_subsystem,
                    self.transfer_subsystem,
                    self.hood_subsystem,
                ),
                IntakeRollerOscillate(self.intake_roller, self._drivetrain)
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        ).debounce(0.2)

        self._controller_1.leftTrigger().whileTrue(
            cmd.runEnd(
                lambda: self._drivetrain.set_modifiers(
                    drivetrain.SwerveDriveTrain.FAST
                ),
                lambda: self._drivetrain.reset_modifiers(),
            )
        )
        
        # All the path to pose commands for doing bump/trench
        self._controller_1.x().and_(self._controller_1.leftBumper().not_()).and_(self._controller_1.rightBumper().not_()).whileTrue(
            self.custom_path_commands.teleop_paths["left_trench"]
        )
        self._controller_1.a().and_(self._controller_1.leftBumper().not_()).and_(self._controller_1.rightBumper().not_()).whileTrue(
            self.custom_path_commands.teleop_paths["left_bump"]
        )
        self._controller_1.y().and_(self._controller_1.leftBumper().not_()).and_(self._controller_1.rightBumper().not_()).whileTrue(
            self.custom_path_commands.teleop_paths["right_bump"]
        )
        self._controller_1.b().and_(self._controller_1.leftBumper().not_()).and_(self._controller_1.rightBumper().not_()).whileTrue(
            self.custom_path_commands.teleop_paths["right_trench"]
        )
        # Go to opposing zone paths
        self._controller_1.x().and_(self._controller_1.povUp()).and_(self._controller_1.rightBumper().not_()).whileTrue(
            self.custom_path_commands.teleop_paths["opposing_left_trench"]
        )
        self._controller_1.a().and_(self._controller_1.povUp()).and_(self._controller_1.rightBumper().not_()).whileTrue(
            self.custom_path_commands.teleop_paths["opposing_left_bump"]
        )
        self._controller_1.y().and_(self._controller_1.povUp()).and_(self._controller_1.rightBumper().not_()).whileTrue(
            self.custom_path_commands.teleop_paths["opposing_right_bump"]
        )
        self._controller_1.b().and_(self._controller_1.povUp()).and_(self._controller_1.rightBumper().not_()).whileTrue(
            self.custom_path_commands.teleop_paths["opposing_right_trench"]
        )
        # Extake spot paths
        self._controller_1.x().and_(self._controller_1.leftBumper().not_()).and_(self._controller_1.rightBumper()).whileTrue(
            self.custom_path_commands.teleop_paths["extake_left_trench"]
        )
        self._controller_1.b().and_(self._controller_1.leftBumper().not_()).and_(self._controller_1.rightBumper()).whileTrue(
            self.custom_path_commands.teleop_paths["extake_right_trench"]
        )
        
        from commands.path_commands import drive_to_a_spot
        self._controller_1.povUp().whileTrue(
            drive_to_a_spot.DriveToASpot(self._drivetrain).with_lock_values()
        )

        # -------------------------------------------------------------------
        # CONTROLLER 2 
        # -------------------------------------------------------------------

        self._controller_2.a().whileTrue(ReverseTransferCommand(self.transfer_subsystem))
        self._controller_2.x().whileTrue(SpindexOnlyCommand(self.transfer_subsystem))
        
        self._controller_2.b().whileTrue(RunTransferCommand(self.transfer_subsystem))

        self._controller_2.y().whileTrue(
            shooter_commands.EnableShooter(self.shooter_subsystem)
        )

        self._controller_2.leftTrigger().whileTrue(
            IntakeRollerForward(self.intake_roller)
        )

        self._controller_2.rightTrigger().whileTrue(
            IntakeRollerBackward(self.intake_roller)
        )

        self._controller_2.leftBumper().onTrue(
            IntakePivotReverse(self.intake_pivot),
        )

        self._controller_2.rightBumper().onTrue(
            IntakePivotForward(self.intake_pivot),
        )

        self._controller_2.povUp().onTrue(
            hood_commands.ResetShooterHood(self.hood_subsystem)
        )

        self._controller_2.povLeft().whileTrue(
            IntakeRollerBackward(self.intake_roller)
        )

    def getAutonomousCommand(self):
        IntakePivotForward(self.intake_pivot).schedule()
        IntakeRollerForward(self.intake_roller).schedule()

        initial_pose = self.auto_chooser.get_initial_pose()
        if initial_pose:
            if key_poses.kPath.MIRROR_REVERSE_PATHS:
                initial_pose = FlipUtil.mirrorPose(initial_pose)
            drive_commands.initial_pose(self._drivetrain, initial_pose).schedule()
        
        return self.auto_chooser.choose_auto()
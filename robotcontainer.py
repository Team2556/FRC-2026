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
from subsystems.vision import multi_limelight
from subsystems.intake.intake import IntakeSubsystem
from subsystems.trasnfer.transfer_subsystem import TransferSubsystem
from subsystems.shooter.shooter_hood import ShooterHood
from subsystems.led.LED_controller import CANdleLEDController
from subsystems.shooter.dual_shooter import DualMotorShooter

from commands.auto_align import align_with_controller
from commands.drive import drive_commands
from commands.vision import vision_odometry
from constants.path import custom_path_commands, key_poses
from commands.transfer.run_transfer_motors import RunTransferCommand
from commands.intake.intake_commands import (
    IntakePivotForward,
    IntakePivotReverse,
    IntakeDefaultCommand,
    IntakeRollerForward,
    IntakeRollerBackward,
)
from commands.shooter import shooter_commands, hood_commands

from constants.vision import kCamera


class RobotContainer:
    def __init__(self) -> None:
        self._controller_1 = XboxController(port=0).with_deadband(0).with_power(1).with_mult(0)
        self._controller_2 = XboxController(port=1).with_deadband(0).with_power(1).with_mult(0)

        self._drivetrain = drivetrain.SwerveDriveTrain()

        self.intake_subsystem = IntakeSubsystem()
        self.transfer_subsystem = TransferSubsystem()
        self.shooter_subsystem = DualMotorShooter()
        self.hood_subsystem = ShooterHood()
        # self.climb_subsystem = ClimbSubsystem()

        self.mono_vision = multi_limelight.Vision(kCamera.BACK_LL, kCamera.SHOOTER_LL)
        self.LED_controller = CANdleLEDController()

        self.custom_path_commands = custom_path_commands.CustomPathCommands(
            self._drivetrain,
            intake_subsystem=self.intake_subsystem,
            transfer_subsystem=self.transfer_subsystem,
            shooter_subsystem=self.shooter_subsystem,
            hood_subsystem=self.hood_subsystem,
        )

        self.auto_chooser = AutoBuilder(
            self.custom_path_commands.get_auto_paths()
        )
        SmartDashboard.putData("Update ALL (auto) Values", cmd.runOnce(self.update_auto))

        self.time_manager = SendFMSData()

        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:

        # -------------------------------------------------------------------
        # Default commands
        # -------------------------------------------------------------------

        self.mono_vision.setDefaultCommand(
            vision_odometry.UpdateOdometry(self.mono_vision, self._drivetrain)
        )
        self.shooter_subsystem.setDefaultCommand(
            shooter_commands.DisableShooter(self.shooter_subsystem)
        )
        self.intake_subsystem.setDefaultCommand(
            IntakeDefaultCommand(self.intake_subsystem, self._drivetrain)
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
                    self.intake_subsystem,
                ),
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        ).debounce(0.2)

        self._controller_1.leftTrigger().whileTrue(
            cmd.runEnd(
                lambda: self._drivetrain.set_modifiers(
                    drivetrain.SwerveDriveTrain.SLOW
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
        self._controller_1.x().and_(self._controller_1.leftBumper()).and_(self._controller_1.rightBumper().not_()).whileTrue(
            self.custom_path_commands.teleop_paths["opposing_left_trench"]
        )
        self._controller_1.a().and_(self._controller_1.leftBumper()).and_(self._controller_1.rightBumper().not_()).whileTrue(
            self.custom_path_commands.teleop_paths["opposing_left_bump"]
        )
        self._controller_1.y().and_(self._controller_1.leftBumper()).and_(self._controller_1.rightBumper().not_()).whileTrue(
            self.custom_path_commands.teleop_paths["opposing_right_bump"]
        )
        self._controller_1.b().and_(self._controller_1.leftBumper()).and_(self._controller_1.rightBumper().not_()).whileTrue(
            self.custom_path_commands.teleop_paths["opposing_right_trench"]
        )
        # Extake spot paths
        self._controller_1.x().and_(self._controller_1.leftBumper().not_()).and_(self._controller_1.rightBumper()).whileTrue(
            self.custom_path_commands.teleop_paths["extake_left_trench"]
        )
        self._controller_1.b().and_(self._controller_1.leftBumper().not_()).and_(self._controller_1.rightBumper()).whileTrue(
            self.custom_path_commands.teleop_paths["extake_right_trench"]
        )
        
        # Testing button for precision
        self._controller_1.povLeft().whileTrue(
            self.custom_path_commands.teleop_paths["back_left_corner"]
        )

        # -------------------------------------------------------------------
        # CONTROLLER 2
        # -------------------------------------------------------------------

        self._controller_2.b().whileTrue(RunTransferCommand(self.transfer_subsystem))

        self._controller_2.y().whileTrue(
            shooter_commands.EnableShooter(self.shooter_subsystem)
        )

        self._controller_2.leftTrigger().whileTrue(
            IntakeRollerForward(self.intake_subsystem)
        )

        self._controller_2.rightTrigger().whileTrue(
            IntakeRollerBackward(self.intake_subsystem)
        )

        self._controller_2.leftBumper().onTrue(
            IntakePivotReverse(self.intake_subsystem),
        )

        self._controller_2.rightBumper().onTrue(
            IntakePivotForward(self.intake_subsystem),
        )

        self._controller_2.povUp().onTrue(
            hood_commands.ResetShooterHood(self.hood_subsystem)
        )

        self._controller_2.povLeft().whileTrue(
            IntakeRollerBackward(self.intake_subsystem)
        )

    def update_auto(self):
        self.auto = self.auto_chooser.choose_auto()

    def getAutonomousCommand(self):
        IntakePivotForward(self.intake_subsystem).schedule()
        IntakeRollerForward(self.intake_subsystem).schedule()

        initial_pose = self.auto_chooser.get_initial_pose()
        if key_poses.kPath.MIRROR_REVERSE_PATHS:
            initial_pose = FlipUtil.mirrorPose(initial_pose)
        if initial_pose:
            drive_commands.initial_pose(self._drivetrain, initial_pose).schedule()
        
        self.update_auto()
        return self.auto
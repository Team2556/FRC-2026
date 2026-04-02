#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
from wpilib import SmartDashboard

from util.custom_controller import XboxController
from util.send_fms_data import SendFMSData
from util.auto_chooser import AutoBuilder

from subsystems.drivetrain import drivetrain
from subsystems.vision import multi_limelight
from subsystems.intake.intake import IntakeSubsystem
from subsystems.trasnfer.transfer_subsystem import TransferSubsystem
from subsystems.shooter.shooter_hood import ShooterHood, HoodStates
from subsystems.led.LED_controller import CANdleLEDController
from subsystems.shooter.dual_shooter import DualMotorShooter
from commands2 import ParallelCommandGroup, cmd, InstantCommand, InterruptionBehavior
from commands2.button import Trigger

from commands.auto_align import align_with_controller
from commands.auto_align.alignio import AlignIntakeToVelocity
from commands.drive import drive_commands
from commands.vision import vision_odometry
from constants.path import custom_path_commands
from commands.transfer.run_transfer_motors import RunTransferCommand
from commands.intake.intake_commands import (
    IntakePivotForward,
    IntakePivotReverse,
    IntakeDefaultCommand,
    IntakeRollerForward,
    IntakeRollerBackward,
    IntakeRollerOscillate,
)
from commands.shooter import shooter_commands, hood_commands

from constants.vision import kCamera
from constants.drive import kDriveConfig
from constants.led import kLED
from constants.path.key_poses import kPoses

from subsystems.drivetrain import drivetrain
from subsystems.vision import multi_limelight
from subsystems.intake.intake import IntakeSubsystem
from subsystems.climb.climb_subsystem import ClimbSubsystem
from subsystems.trasnfer.transfer_subsystem import TransferSubsystem
from subsystems.shooter.shooter_hood import ShooterHood, HoodStates
from subsystems.led.LED_controller import CANdleLEDController
from subsystems.shooter.dual_shooter import DualMotorShooter

from util.custom_controller import XboxController
from util.send_fms_data import SendFMSData
from util.auto_chooser import AutoBuilder
from util.robot_zone_checker import RobotZoneChecker


class RobotContainer:
    def __init__(self) -> None:
        self._controller_1 = XboxController(port=0).with_deadband(0.2).with_power(3)
        self._controller_2 = XboxController(port=1).with_deadband(0.2).with_power(3)

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
            # climb_subsyetem=self.climb_subsystem,
        )
        
        self.auto_chooser = AutoBuilder(
            self.custom_path_commands.get_auto_paths()
        )
        SmartDashboard.putData("Update ALL (auto) Values", cmd.runOnce(self.update_auto))

        self.time_manager = SendFMSData()
        
        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:

        # Order: Default commands, controller 1, controller 2
        # Controller Button Order: Joysticks, Triggers/Bumpers, Letter Buttons, D-pad, Other Buttons

        self.mono_vision.setDefaultCommand(
            vision_odometry.UpdateOdometry(self.mono_vision, self._drivetrain)
        )
        self.shooter_subsystem.setDefaultCommand(
            shooter_commands.DisableShooter(self.shooter_subsystem)
        )
        self.intake_subsystem.setDefaultCommand(
            IntakeDefaultCommand(self.intake_subsystem, self._drivetrain)
        )

        # CONTROLLER 1
        self._drivetrain.setDefaultCommand(
            drive_commands.ControllerDrive(self._drivetrain, self._controller_1)
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

        # Currently not nessary, a little buggy too
        # self._controller_1.rightBumper().whileTrue(
        #     AlignIntakeToVelocity(self._drivetrain)
        # )

        # CONTROLLER 2

        # Retracts hood in danger zone (bumps/trench); interrupts default command _FCC_
        self.hood_subsystem.setDefaultCommand(
            hood_commands.UpdateHoodPositionVariable(
                self.hood_subsystem, self._drivetrain
            )
        )

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
        # Kinda optional maybe initial stuff
        # hood_commands.ResetShooterHood(self.hood_subsystem).schedule()
        IntakePivotForward(self.intake_subsystem).schedule()
        IntakeRollerForward(self.intake_subsystem).schedule()
        
        # Do Auto command stuff
        initial_pose = self.auto_chooser.get_initial_pose()
        if initial_pose:
            drive_commands.InitialPose(self._drivetrain, initial_pose).schedule()
        self.update_auto()
        return self.auto

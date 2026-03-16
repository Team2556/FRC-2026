#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from commands.auto_align import align_with_controller
from commands.drive import drive_commands
from util.custom_controller import XboxController
from util.send_fms_data import SendFMSData
from util.auto_chooser import AutoChooser

from commands.vision import vision_odometry
from commands.path_commands import custom_path_commands, go_back_with_path
from commands.transfer.run_transfer_motors import RunTransferCommand
from commands.intake.intake_commands import (
    IntakeCommandDeploy,
    IntakeCommandUndeploy,
    IntakeForceRetract,
)
from commands.climb.climb import ClimbDown, ClimbUp
from commands.shooter import shooter_commands, hood_commands

from constants.vision import kCamera
from constants.drive import kDriveConfig
from constants.led import kLED

from subsystems.drivetrain import drivetrain
from subsystems.vision import mono_limelight
from subsystems.intake.intake import IntakeSubsystem
from subsystems.climb.climb_subsystem import ClimbSubsystem
from subsystems.trasnfer.transfer_subsystem import TransferSubsystem

from subsystems.shooter.shooter_hood import ShooterHood
from subsystems.led.LED_controller import CANdleLEDController
from subsystems.shooter.dual_shooter import DualMotorShooter

from commands2 import ParallelCommandGroup, RunCommand, cmd, InstantCommand
from commands2.button import Trigger

from util.robot_zone_checker import RobotZoneChecker
from util.flip_util import FlipUtil
from constants.field import kHub, kPassSpots


class RobotContainer:
    def __init__(self) -> None:
        self._controller_1 = (
            XboxController(port=0).with_deadband(0.3).with_power(5).with_mult(0.6)
        )
        self._controller_2 = (
            XboxController(port=1).with_deadband(0.3).with_power(5).with_mult(0.6)
        )

        self._drivetrain = drivetrain.SwerveDriveTrain()

        self.intake_subsystem = IntakeSubsystem()
        self.transfer_subsystem = TransferSubsystem()
        self.shooter_subsystem = DualMotorShooter()
        self.hood_subsystem = ShooterHood()
        # self.climb_subsystem = ClimbSubsystem()

        self.mono_vision = mono_limelight.Vision(kCamera.BACK_LL, kCamera.SHOOTER_LL)
        self.LED_controller = CANdleLEDController()

        self.custom_path_commands = custom_path_commands.CustomPathCommands(
            self._drivetrain,
            intake_subsystem=self.intake_subsystem,
            transfer_subsystem=self.transfer_subsystem,
            shooter_subsystem=self.shooter_subsystem,
            hood_subsystem=self.hood_subsystem,
            led_controller=self.LED_controller,
            # climb_subsyetem=self.climb_subsystem,
        )
        
        self.auto_chooser = AutoChooser(self.custom_path_commands.get_autos())
        self.auto_chooser.make_dropdown()
        
        self.time_manager = SendFMSData()

        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        self.mono_vision.setDefaultCommand(
            vision_odometry.UpdateOdometry(self.mono_vision, self._drivetrain)
        )
        self.shooter_subsystem.setDefaultCommand(
            shooter_commands.DisableShooter(self.shooter_subsystem)
        )

        # CONTROLLER 1
        self._drivetrain.setDefaultCommand(
            drive_commands.ControllerDrive(self._drivetrain, self._controller_1)
        )

        # .negate() guards prevent firing when both bumpers are held (that's intake) _FCC_
        self._controller_1.rightBumper().and_(
            self._controller_1.leftBumper().negate()
        ).whileTrue(
            ParallelCommandGroup(
                RunTransferCommand(self.transfer_subsystem, self.shooter_subsystem),
                shooter_commands.EnableShooter(self.shooter_subsystem),
            )
        )

        self._controller_1.rightTrigger().whileTrue(
            align_with_controller.ConditionalAlignAndShoot(
                self._drivetrain,
                self.shooter_subsystem,
                self.transfer_subsystem,
                self.hood_subsystem,
                self.LED_controller,
            )
        )

        self._controller_1.leftBumper().and_(
            self._controller_1.rightBumper().negate()
        ).whileTrue(go_back_with_path.GoBackWithPath(self._drivetrain))

        self._controller_1.leftTrigger().whileTrue(
            cmd.runEnd(
                lambda: self._drivetrain.change_speed_mult(
                    kDriveConfig.SLOW_SPEED_MULT, kDriveConfig.SLOW_ROTATION_MULT
                ),
                lambda: self._drivetrain.change_speed_mult(),
            )
        )

        self._controller_1.x().whileTrue(self.custom_path_commands.left_trench)
        self._controller_1.a().whileTrue(self.custom_path_commands.left_bump)
        self._controller_1.y().whileTrue(self.custom_path_commands.right_bump)
        self._controller_1.b().whileTrue(self.custom_path_commands.right_trench)

        # CONTROLLER 2

        # =========================
        #        TESTING ONLY
        # self.hood_subsystem.setDefaultCommand(
        #     hood_commands.ManualShooterHood(
        #         self.hood_subsystem,
        #         self._controller_2,
        #         self._get_hood_pose_and_target,
        #     )
        # )
        # =========================
        
        # Retracts hood in danger zone (bumps/trench); interrupts default command _FCC_
        Trigger(self._drivetrain.should_stop_shooting).whileTrue(
            RunCommand(
                lambda: self.hood_subsystem.toggle_force_hide(True), self.hood_subsystem
            )
        ).whileFalse(
            RunCommand(
                lambda: self.hood_subsystem.toggle_force_hide(False),
                self.hood_subsystem,
            )
        )
        # =========================

        self._controller_2.b().whileTrue(
            RunTransferCommand(self.transfer_subsystem, self.shooter_subsystem)
        )

        self._controller_2.y().whileTrue(
            shooter_commands.EnableShooter(self.shooter_subsystem)
        )

        # self._controller_2.povUp().onTrue(ClimbUp(self.climb_subsystem))

        # self._controller_2.povDown().onTrue(ClimbDown(self.climb_subsystem))

        self._controller_2.rightTrigger().onTrue(
            IntakeCommandDeploy(self.intake_subsystem)
        )
        self._controller_2.rightTrigger().onFalse(
            IntakeCommandUndeploy(self.intake_subsystem)
        )

        self._controller_2.povDown().onTrue(IntakeForceRetract(self.intake_subsystem))

        # Dev only: Back + Start force-pushes all dashboard PID values to motors
        (self._controller_1.back().and_(self._controller_1.start())).onTrue(
            InstantCommand(self._force_apply_all_pids)
        )

    def _get_hood_pose_and_target(self):
        # Picks hub or pass spot based on field zone; keeps drivetrain out of hood commands _FCC_
        pose = self._drivetrain.get_state().pose
        if RobotZoneChecker.is_in_left_neutral_zone(pose):
            return pose, FlipUtil.fieldPose(kPassSpots.PASS_SPOT_LEFT)
        if RobotZoneChecker.is_in_right_neutral_zone(pose):
            return pose, FlipUtil.fieldPose(kPassSpots.PASS_SPOT_RIGHT)
        return pose, FlipUtil.fieldPose(kHub.POS)

    def _force_apply_all_pids(self):
        self.intake_subsystem.pivot_editable_pid.force_apply()
        self.intake_subsystem.roller_editable_pid.force_apply()
        self.transfer_subsystem.spindex_editable_pid.force_apply()
        self.transfer_subsystem.up_transfer_editable_pid.force_apply()
        self.hood_subsystem.editable_pid.force_apply()
        self.shooter_subsystem.editable_PID.force_apply()

    def getAutonomousCommand(self):
        return self.auto_chooser.choose_auto()

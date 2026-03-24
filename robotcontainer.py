#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from commands2 import (
    ParallelCommandGroup,
    cmd,
    InstantCommand
)
from commands2.button import Trigger

from commands.auto_align import align_with_controller
from commands.drive import drive_commands
from commands.vision import vision_odometry
from commands.path_commands import custom_path_commands, go_back_with_path, go_to_shooting_spot, drive_to_a_spot
from commands.transfer.run_transfer_motors import RunTransferCommand, ReverseTransferCommand, SpindexOnlyCommand
from commands.intake.intake_commands import (
    IntakeCommandManualForward,
    IntakeCommandManualReverse,
    IntakeDefaultCommand,
    IntakeRollerForward,
    IntakeRollerBackward,
)
from commands.climb.climb import ClimbDown, ClimbUp
from commands.shooter import shooter_commands, hood_commands
from commands.drive.drive_commands import (
    OverrideRotation,
    SetOdometryBackLeft,
    SetOdometryBackRight,
)

from constants.vision import kCamera
from constants.drive import kDriveConfig
from constants.led import kLED
from constants.key_poses import kPoses

from subsystems.drivetrain import drivetrain
from subsystems.vision import mono_limelight
from subsystems.intake.intake import IntakeSubsystem
from subsystems.climb.climb_subsystem import ClimbSubsystem
from subsystems.trasnfer.transfer_subsystem import TransferSubsystem
from subsystems.shooter.shooter_hood import ShooterHood, HoodStates
from subsystems.led.LED_controller import CANdleLEDController
from subsystems.shooter.dual_shooter import DualMotorShooter

from util.custom_controller import XboxController
from util.send_fms_data import SendFMSData
from util.auto_chooser import AutoChooser
from util.robot_zone_checker import RobotZoneChecker
from util.tune_with_controller import TuneShooterSpeed, TuneAlignAngle, TuneHoodAngle

class RobotContainer:
    def __init__(self) -> None:
        self._controller_1 = (
            XboxController(port=0).with_deadband(0.2).with_power(3)
        )
        self._controller_2 = (
            XboxController(port=1).with_deadband(0.2).with_power(3)
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
            # climb_subsyetem=self.climb_subsystem,
        )
        self.time_manager = SendFMSData()
        self.auto_chooser = AutoChooser(self.custom_path_commands.make_autos())
        self.auto_chooser.make_dropdown()

        # In-game tuner stuff:
        self.tune_hood_angle = TuneHoodAngle(self._controller_2)
        # self.tune_align_angle = TuneAlignAngle(self._controller_2)

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
            IntakeDefaultCommand(self.intake_subsystem)
        )

        # CONTROLLER 1
        self._drivetrain.setDefaultCommand(
            drive_commands.ControllerDrive(self._drivetrain, self._controller_1)
        )

        self._controller_1.rightTrigger().whileTrue(
            align_with_controller.ConditionalAlignAndShoot(
                self._drivetrain,
                self.shooter_subsystem,
                self.transfer_subsystem,
                self.hood_subsystem,
            )
        )

        # self._controller_1.leftBumper().whileTrue(go_back_with_path.GoBackWithPath(self._drivetrain))
        
        self._controller_1.leftBumper().whileTrue(
            ParallelCommandGroup(
                go_to_shooting_spot.GoToShootingSpot(self._drivetrain),
                shooter_commands.EnableShooter(self.shooter_subsystem)
            )
        )

        self._controller_1.leftTrigger().whileTrue(
            cmd.runEnd(
                lambda: self._drivetrain.change_speed_mult(
                    kDriveConfig.SLOW_SPEED_MULT
                ),
                lambda: self._drivetrain.change_speed_mult(),
            )
        )

        self._controller_1.x().and_(self._controller_1.rightBumper().not_()).whileTrue(
            ParallelCommandGroup(
                self.custom_path_commands.left_trench,
                IntakeCommandManualForward(self.intake_subsystem),
            )
        )
        
        self._controller_1.x().and_(self._controller_1.rightBumper()).whileTrue(
            drive_to_a_spot.DriveToASpot(self._drivetrain, target_pose=kPoses.left_trench_feed).with_precise_values()
        )
        
        self._controller_1.a().whileTrue(
            self.custom_path_commands.left_bump
        )
        
        self._controller_1.y().whileTrue(
            self.custom_path_commands.right_bump
        )
        
        self._controller_1.b().and_(self._controller_1.rightBumper().not_()).whileTrue(
            ParallelCommandGroup(
                self.custom_path_commands.right_trench,
                IntakeCommandManualForward(self.intake_subsystem),
            )
        )
        
        self._controller_1.b().and_(self._controller_1.rightBumper()).whileTrue(
            drive_to_a_spot.DriveToASpot(self._drivetrain, target_pose=kPoses.right_trench_feed).with_precise_values()
        )
        
        self._controller_1.povUp().onTrue(
            OverrideRotation(self._drivetrain)
        )
        self._controller_1.povLeft().onTrue(
            SetOdometryBackLeft(self._drivetrain)
        )
        self._controller_1.povRight().onTrue(
            SetOdometryBackRight(self._drivetrain)
        )

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

        self._controller_2.x().whileTrue(SpindexOnlyCommand(self.transfer_subsystem))

        self._controller_2.a().whileTrue(
            ReverseTransferCommand(self.transfer_subsystem)
        )

        self._controller_2.leftTrigger().whileTrue(
            IntakeRollerForward(self.intake_subsystem)
        )

        self._controller_2.rightTrigger().whileTrue(
            IntakeRollerBackward(self.intake_subsystem)
        )

        self._controller_2.leftBumper().onTrue(
            IntakeCommandManualReverse(self.intake_subsystem),
        )
        
        self._controller_2.rightBumper().onTrue(
            IntakeCommandManualForward(self.intake_subsystem),
        )

        self._controller_2.povUp().onTrue(
            hood_commands.ResetShooterHood(self.hood_subsystem)
        )

        self._controller_2.povLeft().whileTrue(
            IntakeRollerBackward(self.intake_subsystem)
        )

        self._controller_2.povDown().whileTrue(
            hood_commands.SetShooterHoodState(
                self.hood_subsystem, HoodStates.OUTER_RING
            )
        )

        self._controller_2.leftStick().onTrue(cmd.runOnce(lambda: self.tune_hood_angle.reset()))
        # self._controller_2.rightStick().onTrue(cmd.runOnce(lambda: self.tune_align_angle.reset()))
        
        # self._controller_2.povUp().onTrue(ClimbUp(self.climb_subsystem))

        # self._controller_2.povDown().onTrue(ClimbDown(self.climb_subsystem))

        # Dev only: Back + Start force-pushes all dashboard PID values to motors
        (self._controller_1.back().and_(self._controller_1.start())).onTrue(
            InstantCommand(self._force_apply_all_pids)
        )

    def _force_apply_all_pids(self):
        self.intake_subsystem.pivot_editable_pid.force_apply()
        self.intake_subsystem.roller_editable_pid.force_apply()
        self.transfer_subsystem.spindex_editable_pid.force_apply()
        self.transfer_subsystem.up_transfer_editable_pid.force_apply()
        self.hood_subsystem.editable_pid.force_apply()
        self.shooter_subsystem.editable_PID.force_apply()

    def getAutonomousCommand(self):
        hood_commands.ResetShooterHood(self.hood_subsystem).schedule()
        # TODO this second command kinda isn't required
        IntakeRollerForward(self.intake_subsystem).schedule()
        return self.auto_chooser.choose_auto()
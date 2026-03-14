#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from commands.auto_align import align_with_controller
from util.custom_controller import XboxController
from util.send_fms_data import SendFMSData

from commands import drive_commands, vision_odometry
from commands.path_commands import custom_path_commands, go_back_with_path
from commands.run_transfer_motors import RunTransferCommand
from commands.intake_commands import IntakeCommandDeploy, IntakeCommandUndeploy
from commands.climb import ClimbDown, ClimbUp
from commands.shooter import shooter_commands, hood_commands

from constants.vision import kCamera
from constants.drive import kDriveConfig
from constants.led import kLED

from subsystems.drivetrain import drivetrain
from subsystems.vision import mono_limelight
from subsystems.intake import IntakeSubsystem
from subsystems.climb_subsystem import ClimbSubsystem
from subsystems.transfer_subsystem import TransferSubsystem

from subsystems.shooter.shooter_hood import ShooterHood
from subsystems.led.LED_controller import CANdleLEDController
from subsystems.shooter.dual_shooter import DualMotorShooter

from commands2 import ParallelCommandGroup, cmd


class RobotContainer:
    def __init__(self) -> None:
        self._controller_1 = XboxController(port=0).with_deadband(0.1)
        self._controller_2 = XboxController(port=1).with_deadband(0.1)

        self._drivetrain = drivetrain.SwerveDriveTrain()

        self.intake_subsystem = IntakeSubsystem()
        self.transfer_subsystem = TransferSubsystem()
        self.shooter_subsystem = DualMotorShooter()
        self.hood_subsystem = ShooterHood()
        self.climb_subsystem = ClimbSubsystem()

        self.mono_vision = mono_limelight.Vision(kCamera.llFront.NAME)
        self.LED_controller = CANdleLEDController(kLED.CAN_ID)

        self.custom_path_commands = custom_path_commands.CustomPathCommands(
            self._drivetrain,
            shooter_subsystem=self.shooter_subsystem,
            climb_subsyetem=self.climb_subsystem,
        )
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

        self._controller_1.rightBumper().whileTrue(
            ParallelCommandGroup(
                RunTransferCommand(self.transfer_subsystem),
                shooter_commands.EnableShooter(self.shooter_subsystem),
            )
        )

        self._controller_1.rightTrigger().whileTrue(
            align_with_controller.ConditionalAlignAndShoot(
                self._drivetrain,
                self._controller_1,
                self.shooter_subsystem,
                self.transfer_subsystem,
                self.hood_subsystem,
                self.LED_controller,
            )
        )

        self._controller_1.leftBumper().whileTrue(
            go_back_with_path.GoBackWithPath(self._drivetrain)
        )

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
        self.hood_subsystem.setDefaultCommand(
            hood_commands.ManualShooterHood(self.hood_subsystem, self._controller_2)
        )
        # =========================
        
        self._controller_2.b().whileTrue(RunTransferCommand(self.transfer_subsystem))

        self._controller_2.y().whileTrue(
            shooter_commands.EnableShooter(self.shooter_subsystem)
        )

        self._controller_2.povUp().onTrue(ClimbUp(self.climb_subsystem))

        self._controller_2.povDown().onTrue(ClimbDown(self.climb_subsystem))

        self._controller_2.rightTrigger().onTrue(
            IntakeCommandDeploy(self.intake_subsystem)
        )
        self._controller_2.rightTrigger().onFalse(
            IntakeCommandUndeploy(self.intake_subsystem)
        )

    def getAutonomousCommand(self):
        return self.custom_path_commands.test_auto

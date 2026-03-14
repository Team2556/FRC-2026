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
from commands.spin_motor import SpinMotor
from commands.intake_commands import IntakeCommandDeploy, IntakeCommandUndeploy
from commands.climb import ClimbDown, ClimbUp
from commands.reset_shooter_hood import ResetShooterHood

from constants.vision import kCamera
from constants.indexer import kSpindexer, kTrasnfer
from constants.shooter import kShooterMotor
from constants.intake import kIntakeSpinner

# from pathplannerlib.auto import NamedCommands
from constants.intake import kIntakeMotor
from constants.climb import kClimb
from constants.drive import kDriveConfig
from constants.led import kLED

from subsystems.drivetrain import drivetrain
from subsystems.vision import mono_limelight
from subsystems.controlled_motor import ControlledTalonMotor
from subsystems.intake import IntakeSubsystem

from commands2.button import CommandXboxController
from subsystems.shooter.shooter_hood import ShooterHood
from subsystems.led.LED_controller import CANdleLEDController

from subsystems.climb_subsystem import ClimbSubsystem
# from subsystems.intake import IntakeSubsystem

from commands2 import ParallelCommandGroup, cmd

class RobotContainer:
    def __init__(self) -> None:
        
        self._controller_1 = (
            XboxController(port=0).with_deadband(0.1).with_smoothing(0.1)
        )
        self._controller_2 = (
            XboxController(port=1).with_deadband(0.1).with_smoothing(0.1)
        )

        self._drivetrain = drivetrain.SwerveDriveTrain()
        self.mono_vision = mono_limelight.Vision(kCamera.llFront.NAME)
        
        self.intake_subsystem = IntakeSubsystem()
        self.LED_controller = CANdleLEDController(kLED.CAN_ID)

        self.spindex_motor = ControlledTalonMotor(
            "Spindex",
            kSpindexer.CAN_ID,
            kSpindexer._CONFIG,
            kSpindexer.TARGET_RPM,
        )
        self.transfer_motor = ControlledTalonMotor(
            "Transfer 1",
            kTrasnfer.motor_1.CAN_ID,
            kTrasnfer.motor_1._CONFIG,
            kTrasnfer.motor_1.TARGET_RPM,
        )
        self.shooter_motor = ControlledTalonMotor(
            "Shooter",
            kShooterMotor.CAN_ID,
            kShooterMotor._CONFIG,
            kShooterMotor.TARGET_RPM,
            enable_smartdashboard=True,
            coast_when_neutral=True
        )
        
        self.climb_subsystem = ClimbSubsystem()
        
        self.hood_motor = ShooterHood()
        
        self.custom_path_commands = custom_path_commands.CustomPathCommands(
            self._drivetrain,
            hood_subsystem = self.hood_motor,
            shooter_subsystem = self.shooter_motor,
            climb_subsyetem = self.climb_subsystem
        )
        
        self.time_manager = SendFMSData()

        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        
        self.mono_vision.setDefaultCommand(
            vision_odometry.UpdateOdometry(self.mono_vision, self._drivetrain)
        )
        
        # CONTROLLER 1
        self._drivetrain.setDefaultCommand(
            drive_commands.ControllerDrive(self._drivetrain, self._controller_1)
        )

        self._controller_1.rightBumper().whileTrue(
            ParallelCommandGroup(
                SpinMotor(self.transfer_motor),
                SpinMotor(self.spindex_motor),
                SpinMotor(self.shooter_motor),
            )
        )
        
        self._controller_1.rightTrigger().whileTrue(
            align_with_controller.ConditionalAlignAndShoot(
                self._drivetrain, 
                self._controller_1, 
                self.shooter_motor, 
                self.spindex_motor,
                self.transfer_motor,
                self.hood_motor,
                self.LED_controller
            )
        )
        
        self._controller_1.leftBumper().whileTrue(
            go_back_with_path.GoBackWithPath(self._drivetrain)
        )
        
        self._controller_1.leftTrigger().whileTrue(
            cmd.runEnd(
                lambda: self._drivetrain.change_speed_mult(kDriveConfig.SLOW_SPEED_MULT, kDriveConfig.SLOW_ROTATION_MULT),
                lambda: self._drivetrain.change_speed_mult()
            )
        )
    
        self._controller_1.x().whileTrue(self.custom_path_commands.left_trench)
        self._controller_1.a().whileTrue(self.custom_path_commands.left_bump)
        self._controller_1.y().whileTrue(self.custom_path_commands.right_bump)
        self._controller_1.b().whileTrue(self.custom_path_commands.right_trench)
        
        # CONTROLLER 2
        self._controller_2.b().onTrue(
            ParallelCommandGroup(
                SpinMotor(self.spindex_motor),
                SpinMotor(self.transfer_motor),
            )
        )
        
        self._controller_2.y().onTrue(
            ParallelCommandGroup(
                SpinMotor(self.shooter_motor),
            )
        )
        
        cmd.run(lambda: self.hood_motor.increment(self._controller_2.getRightX()))
        
        self._controller_2.povUp().onTrue(
            ClimbUp(self.climb_subsystem)
        )
        
        self._controller_2.povDown().onTrue(
            ClimbDown(self.climb_subsystem)
        )
        
        self._controller_2.rightTrigger().onTrue(
            IntakeCommandDeploy(self.intake_subsystem)
        )
        self._controller_2.rightTrigger().onFalse(
            IntakeCommandUndeploy(self.intake_subsystem)
        )
        
    def getAutonomousCommand(self):
        return self.custom_path_commands.test_auto
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from commands.auto_align import align_with_controller
from util.custom_controller import XboxController

from commands import drive_commands, vision_odometry
from commands.path_commands import custom_path_commands, go_back_with_path
from commands.spin_motor import SpinMotor

from constants.vision import kCamera
from constants.indexer import kSpindexer, kTrasnfer
from constants.shooter import kShooterMotor
from constants.intake import kIntakeMotor

from subsystems.drivetrain import drivetrain
from subsystems.vision import mono_limelight
from subsystems.controlled_motor import ControlledTalonMotor
from subsystems.shooter.shooter_hood import ShooterHood

# from subsystems.intake import IntakeSubsystem

from commands2 import ParallelCommandGroup

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

        self.spindex_motor = ControlledTalonMotor(
            "Spindex",
            kSpindexer.CAN_ID,
            kSpindexer._CONFIG,
            kSpindexer.TARGET_RPM,
        )
        self.transfer_motor1 = ControlledTalonMotor(
            "Transfer 1",
            kTrasnfer.motor_1.CAN_ID,
            kTrasnfer.motor_1._CONFIG,
            kTrasnfer.motor_1.TARGET_RPM,
        )
        self.transfer_motor2 = ControlledTalonMotor(
            "Transfer 2",
            kTrasnfer.motor_2.CAN_ID,
            kTrasnfer.motor_2._CONFIG,
            kTrasnfer.motor_2.TARGET_RPM,
        )
        self.intake_motor = ControlledTalonMotor(
            "Intake Motor",
            kIntakeMotor.CAN_ID,
            kIntakeMotor._CONFIG,
            kIntakeMotor.TARGET_RPM,
            enable_smartdashboard=True,
        )
        self.shooter_motor = ControlledTalonMotor(
            "Shooter",
            kShooterMotor.CAN_ID,
            kShooterMotor._CONFIG,
            kShooterMotor.TARGET_RPM,
            enable_smartdashboard=True
        )
        
        self.hood_motor = ShooterHood()
        
        self.custom_path_commands = custom_path_commands.CustomPathCommands(
            self._drivetrain,
            hood_subsystem = self.hood_motor,
            shooter_subsystem = self.shooter_motor,
            # climb_subsyetem = code=good
        )

        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        
        self._drivetrain.setDefaultCommand(
            drive_commands.ControllerDrive(self._drivetrain, self._controller_1)
        )

        self._controller_1.rightTrigger().whileTrue(
            ParallelCommandGroup(
                SpinMotor(self.transfer_motor1),
                SpinMotor(self.transfer_motor2),
                SpinMotor(self.spindex_motor),
            )
        )

        self._controller_1.rightBumper().whileTrue(SpinMotor(self.spindex_motor))

        self._controller_1.leftTrigger().whileTrue(SpinMotor(self.shooter_motor))

        self._controller_1.b().whileTrue(
            ParallelCommandGroup(
                align_with_controller.HubAlign(self._drivetrain, self._controller_1, self.shooter_motor, None),
                SpinMotor(self.shooter_motor),
            )
        )
        self._controller_1.a().whileTrue(
            align_with_controller.HubAlign(self._drivetrain, self._controller_1, self.shooter_motor, None),
        )

        self.mono_vision.setDefaultCommand(
            vision_odometry.UpdateOdometry(self.mono_vision, self._drivetrain)
        )
        
        self._controller_1.x().whileTrue(
            go_back_with_path.GoBackWithPath(self._drivetrain)
        )
        
        self._controller_1.y().whileTrue(
            go_back_with_path.GoBackWithPath(self._drivetrain, use_bump = True)
        )
        
        self._controller_2.povLeft().whileTrue(self.custom_path_commands.left_trench_advance)
        self._controller_2.povDown().whileTrue(self.custom_path_commands.left_bump_advance)
        self._controller_2.povUp().whileTrue(self.custom_path_commands.right_bump_advance)
        self._controller_2.povRight().whileTrue(self.custom_path_commands.right_trench_advance)
        
        self._controller_2.x().whileTrue(self.custom_path_commands.left_trench_retreat)
        self._controller_2.a().whileTrue(self.custom_path_commands.left_bump_retreat)
        self._controller_2.y().whileTrue(self.custom_path_commands.right_bump_retreat)
        self._controller_2.b().whileTrue(self.custom_path_commands.right_trench_retreat)
        
        self._controller_2.rightTrigger().whileTrue(
            SpinMotor(self.intake_motor)
        )
        
        self._controller_1.povUp().whileTrue(
            self.custom_path_commands.back_up_to_outpost
        )

    def getAutonomousCommand(self):
        return self.custom_path_commands.test_auto
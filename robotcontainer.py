#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from util.custom_controller import XboxController

from commands import auto_align, drive_commands, vision_odometry
from commands.path_commands import go_back_with_path, drive_to_a_spot, drive_to_a_spot_sequence
from commands.spin_motor import SpinMotor

from constants.vision import kCamera
from constants.indexer import kSpindexer, kTrasnfer
from constants.key_poses import kPoses
from constants.shooter import kShooterMotor
from constants.intake import kIntakeMotor

# from pathplannerlib.auto import NamedCommands

from subsystems.drivetrain import drivetrain
from subsystems.vision import mono_limelight

from subsystems.controlled_motor import ControlledTalonMotor
from commands2.button import CommandXboxController

# from subsystems.intake import IntakeSubsystem

from commands2 import button, ParallelCommandGroup, SequentialCommandGroup, WaitCommand

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
                auto_align.HubAlign(self._drivetrain, self._controller_1, self.shooter_motor, None),
                SpinMotor(self.shooter_motor),
            )
        )
        self._controller_1.a().whileTrue(
            auto_align.HubAlign(self._drivetrain, self._controller_1, self.shooter_motor, None),
        )

        self.mono_vision.setDefaultCommand(
            vision_odometry.UpdateOdometry(self.mono_vision, self._drivetrain)
        )
        
        self._controller_1.x().whileTrue(
            go_back_with_path.GoBackWithPath(self._drivetrain)
        )
        
        self._controller_2.rightTrigger().whileTrue(
            SpinMotor(self.intake_motor)
        )

    def getAutonomousCommand(self):
        pass
        
        start_shooting_point_command = drive_to_a_spot.DriveToASpot(
            self._drivetrain,
            kPoses.start_shooting_point
        ).with_reflected_red_alliance_pose()
        
        bottom_climb_test_command = drive_to_a_spot.DriveToASpot(
            self._drivetrain,
            kPoses.bottom_climb_test
        ).with_reflected_red_alliance_pose().with_precise_values()
        
        autonomous_command = SequentialCommandGroup(
            # Drive to a spot
            start_shooting_point_command,
            # Do some shooting
            WaitCommand(2),
            # Drive to the climber
            bottom_climb_test_command,
            # Do some climbing
            WaitCommand(2)
        )
        
        return autonomous_command

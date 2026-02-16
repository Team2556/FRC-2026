#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from util.custom_controller import XboxController

from commands import auto_align, drive_commands, spin_motor, vision_odometry

from constants.vision import kCamera
from constants.indexer import kSpindexer, kTrasnfer

# from pathplannerlib.auto import NamedCommands

from subsystems.drivetrain import drivetrain
from subsystems.vision import mono_limelight

from subsystems.controlled_motor import ControlledTalonMotor

from commands2 import button, ParallelCommandGroup


class RobotContainer:
    def __init__(self) -> None:
        self._controller_1 = (
            XboxController(port=0).with_deadband(0.05).with_smoothing(1)
        )

        self._drivetrain = drivetrain.SwerveDriveTrain()
        self.mono_vision = mono_limelight.Vision(kCamera.llFront.NAME)

        self.spindex_motor = ControlledTalonMotor(
            "Spindex",
            kSpindexer.CAN_ID,
            kSpindexer._CONFIG,
            kSpindexer.TARGET_RPM,
            enable_smartdashboard=True,
        )
        self.transfer_motor1 = ControlledTalonMotor(
            "Transfer 1",
            kTrasnfer.motor_1.CAN_ID,
            kTrasnfer.motor_1._CONFIG,
            kTrasnfer.motor_1.TARGET_RPM,
            enable_smartdashboard=True,
        )
        self.transfer_motor2 = ControlledTalonMotor(
            "Transfer 2",
            kTrasnfer.motor_2.CAN_ID,
            kTrasnfer.motor_2._CONFIG,
            kTrasnfer.motor_2.TARGET_RPM,
            enable_smartdashboard=True,
        )

        # self.shooter_motor = ControlledTalonMotor(
        #     "Shooter", 24, 0.1, 0.15, 0, -37.000000, enable_smartdashboard=True
        # )

        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        self._drivetrain.setDefaultCommand(
            drive_commands.ControllerDrive(self._drivetrain, self._controller_1)
        )

        self._controller_1.rightTrigger().whileTrue(
            ParallelCommandGroup(
                spin_motor.SpinMotor(self.transfer_motor1),
                spin_motor.SpinMotor(self.transfer_motor2),
                spin_motor.SpinMotor(self.spindex_motor),
            )
        )

        self._controller_1.rightBumper().whileTrue(
            spin_motor.SpinMotor(self.spindex_motor)
        )

        # self._controller_1.leftTrigger().whileTrue(
        #     spin_motor.SpinMotor(self.shooter_motor)
        # )

        self._controller_1.b().whileTrue(
            ParallelCommandGroup(
                auto_align.HubAlign(self._drivetrain, self._controller_1),
                # spin_motor.SpinMotor(self.shooter_motor),
            )
        )

        self.mono_vision.setDefaultCommand(
            vision_odometry.UpdateOdometry(self.mono_vision, self._drivetrain)
        )

    def getAutonomousCommand(self):
        return None

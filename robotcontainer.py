#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from util.custom_controller import XboxController

from commands import auto_align, drive_commands, spin_motor, vision_odometry

from constants.vision import kCamera

# from pathplannerlib.auto import NamedCommands

from subsystems.drivetrain import drivetrain
from subsystems.vision.visionsubsystem import VisionSubsystem
from subsystems.vision.visioniolimelight import VisionSubsystemIOLimelight
from subsystems.vision import mono_limelight

from subsystems import controlled_motor

from commands2 import button, ParallelCommandGroup


class RobotContainer:
    def __init__(self) -> None:
        self._controller_1 = (
            XboxController(port=0).with_deadband(0.05).with_smoothing(1)
        )

        self._drivetrain = drivetrain.SwerveDriveTrain()
        self.mono_vision = mono_limelight.Vision(kCamera.llFront.NAME)
        # self._vision = VisionSubsystem(
        #     self._drivetrain._add_vision_measurements,
        #     [
        #         VisionSubsystemIOLimelight(
        #             kCamera.llFront.NAME,
        #             kCamera.llFront.ROBOT_TO_CAMERA_TRANSFORM,
        #             self._drivetrain.get_robot_rotation,
        #         ),
        #         VisionSubsystemIOLimelight(
        #             kCamera.llRight.NAME,
        #             kCamera.llRight.ROBOT_TO_CAMERA_TRANSFORM,
        #             self._drivetrain.get_robot_rotation,
        #         ),
        #     ],
        # )

        self.spindex_motor = controlled_motor.ControlledTalonMotor(
            "Spindex", 27, 0.1, 5.0, 0, 100
        )
        self.transfer_motor1 = controlled_motor.ControlledTalonMotor(
            "Transfer 1", 20, 0.1, 5, 0, -100
        )
        self.transfer_motor2 = controlled_motor.ControlledTalonMotor(
            "Transfer 2", 21, 0.1, 5, 0, -100
        )
        self.shooter_motor = controlled_motor.ControlledTalonMotor(
            "Shooter", 24, 0.1, 0.15, 0, -37.000000
        )

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

        self._controller_1.leftTrigger().whileTrue(
            spin_motor.SpinMotor(self.shooter_motor)
        )

        auto_align_drive = auto_align.HubAlign(self._drivetrain, self._controller_1)
        self._controller_1.b().whileTrue(
            ParallelCommandGroup(
                auto_align_drive, spin_motor.SpinMotor(self.shooter_motor)
            )
        )

        self.mono_vision.setDefaultCommand(
            vision_odometry.UpdateOdometry(self.mono_vision, self._drivetrain)
        )

    def getAutonomousCommand(self):
        return None

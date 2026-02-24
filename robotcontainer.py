#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from util.custom_controller import XboxController

from commands import auto_align, drive_commands, spin_motor, vision_odometry, intake_commands
from commands.path_commands import go_back_with_path, drive_to_a_spot, drive_to_a_spot_sequence

from constants.vision import kCamera
from constants.indexer import kSpindexer, kTrasnfer
from constants.key_poses import kPoses
from constants.intake import kIntakeSpinner

# from pathplannerlib.auto import NamedCommands

from subsystems.drivetrain import drivetrain
from subsystems.vision import mono_limelight

from subsystems.controlled_motor import ControlledTalonMotor

from subsystems.intake import IntakeSubsystem

from commands2 import button, ParallelCommandGroup, SequentialCommandGroup, WaitCommand

class RobotContainer:
    def __init__(self) -> None:
        self._controller_1 = (
            XboxController(port=0).with_deadband(0.05).with_smoothing(0.1)
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
        
        self.intake_spinny = ControlledTalonMotor(
            "Intake Spinny",
            kIntakeSpinner.CAN_ID,
            kIntakeSpinner._CONFIG,
            kIntakeSpinner.TARGET_RPM,
            enable_smartdashboard=True,
        )
        
        # VERY TEMPORARY THING
        import phoenix6
        cfg = phoenix6.configs.TalonFXConfiguration()
        self.shooter = ControlledTalonMotor(
            "Shooter",
            24,
            cfg,
            -4000,
            enable_smartdashboard=True,
        )
        
        # self.intake_subsystem = IntakeSubsystem()

        # self.shooter_motor = ControlledTalonMotor(
        #     "Shooter", 24, 0.1, 0.15, 0, -37.000000, enable_smartdashboard=True
        # )

        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        self._controller_1.leftStick().onTrue(
            self._drivetrain.runOnce(lambda: self._drivetrain._drivetrain.seed_field_centric())
        )
        
        self._drivetrain.setDefaultCommand(
            drive_commands.ControllerDrive(self._drivetrain, self._controller_1)
        )

        self._controller_1.rightTrigger().whileTrue(
            ParallelCommandGroup(
                spin_motor.SpinMotor(self.transfer_motor1),
                spin_motor.SpinMotor(self.transfer_motor2),
                spin_motor.SpinMotor(self.spindex_motor),
                spin_motor.SpinMotor(self.shooter),
            )
        )

        self._controller_1.rightBumper().whileTrue(
            spin_motor.SpinMotor(self.spindex_motor)
        )
        
        # uncomment this when merging pls
        # self._controller_1.b().whileTrue(
        #     ParallelCommandGroup(
        #         auto_align.HubAlign(self._drivetrain, self._controller_1),
        #         spin_motor.SpinMotor(self.shooter_motor),
        #     )
        # )

        self.mono_vision.setDefaultCommand(
            vision_odometry.UpdateOdometry(self.mono_vision, self._drivetrain)
        )
        
        self._controller_1.x().whileTrue(
            go_back_with_path.GoBackWithPath(self._drivetrain)
        )
        
        # self._controller_1.y().whileTrue(
        #     intake_commands.IntakeCommand(self.intake_subsystem)
        # )
        
        self._controller_1.y().whileTrue(
            spin_motor.SpinMotor(self.intake_spinny)
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

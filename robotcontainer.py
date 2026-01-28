#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from _utils import custom_controller

from subsystems import (controlled_motor, limelight_camera, drivetrain)
from commands import (spin_motor, auto_align, drive_commands)

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self._controller_1 = custom_controller.XboxController(0).with_deadband(0.05).with_smoothing(.5)
        
        self._drivetrain = drivetrain.SwerveDriveTrain()
        # self._example_subsystem = controlled_motor.ControlledMotor()
        # self._front_camera = limelight_camera.LimelightCamera('limelight')
        
        self.configureButtonBindings()
        
    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        
        # run_motor = spin_motor.SpinMotor(self._example_subsystem)
        # self._controller.b().whileTrue(run_motor)
        
        drive_with_controller = drive_commands.ControllerDrive(self._drivetrain, self._controller_1)
        self._drivetrain.setDefaultCommand(drive_with_controller)
        
        reset_field_centric = drive_commands.ResetFieldCentric(self._drivetrain)
        self._controller_1.rightBumper().onTrue(reset_field_centric)
        
        # auto_alignment = auto_align.MobileAlign(self._front_camera, self._drivetrain, self._controller_1)
        # self._controller_1.a().whileTrue(auto_alignment)


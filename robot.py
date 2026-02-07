#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
from wpimath import units
import commands2
import typing

from robotcontainer import RobotContainer

from phoenix6 import SignalLogger

from _utils import limelight_helpers


class MyRobot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[commands2.Command] = None

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        
        self._field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self._field)

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        SignalLogger.stop()
        self.container = RobotContainer()
        

    def robotPeriodic(self) -> None:
        """This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
        that you want ran during disabled, autonomous, teleoperated and test.

        This runs after the mode specific periodic functions, but before LiveWindow and
        SmartDashboard integrated updating."""

        # Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        # commands, running already-scheduled commands, removing finished or interrupted commands,
        # and running subsystem periodic() methods.  This must be called from the robot's periodic
        # block in order for anything in the Command-based framework to work.
        commands2.CommandScheduler.getInstance().run()

        # omegaRPS = self.container._drivetrain._drivetrain.get_rotation3d
        drive_state = self.container._drivetrain._drivetrain.get_state()
        # headingDeg = drive_state.pose.rotation().degrees()
        # omegaRPS = units.radiansToRotations(drive_state.speeds.omega)
        
        # limelight_helpers.set_robot_orientation("limelight", headingDeg, 0, 0, 0, 0, 0)
        
        # llMeasurement = limelight_helpers.get_bot_pose_estimate(
        #     "limelight", "botpose_wpiblue", False
        # )
        # # print(llMeasurement)
        # if llMeasurement != None and llMeasurement.tagCount > 0 and abs(omegaRPS) < 2:
        #     self.container._drivetrain._add_vision_measurements(
        #         llMeasurement.pose, llMeasurement.timestampSeconds
        #     )
        
        # drive_state = self.container._drivetrain._drivetrain.get_state()
        self._field.setRobotPose(drive_state.pose)

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        pass

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        pass

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        pass

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        pass

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        pass

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()

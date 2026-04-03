#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import typing

import wpilib
import commands2
import phoenix6
import phoenix6.unmanaged
from phoenix6.signal_logger import SignalLogger
from ntcore import NetworkTableInstance

from robotcontainer import RobotContainer


class Gravedigger(commands2.TimedCommandRobot):

    autonomousCommand: typing.Optional[commands2.Command] = None

    def robotInit(self) -> None:
        phoenix6.unmanaged.set_phoenix_diagnostics_start_time(-1)
        SignalLogger.enable_auto_logging(False)
        wpilib.LiveWindow.disableAllTelemetry()


        wpilib.DataLogManager.start()
        wpilib.DriverStation.startDataLog(wpilib.DataLogManager.getLog())

        deploy_config = wpilib.deployinfo.getDeployData()
        if deploy_config is not None:
            wpilib.DataLogManager.log(f"Deploy Host: {deploy_config.get('deploy-host', '')}")
            wpilib.DataLogManager.log(f"Deploy User: {deploy_config.get('deploy-user', '')}")
            wpilib.DataLogManager.log(f"Deploy Date: {deploy_config.get('deploy-date', '')}")
            wpilib.DataLogManager.log(f"Git Hash:    {deploy_config.get('git-hash', '')}")
            wpilib.DataLogManager.log(f"Git Branch:  {deploy_config.get('git-branch', '')}")

        self.container = RobotContainer()
        wpilib.CameraServer.launch()

        nt = NetworkTableInstance.getDefault().getTable("Timing")
        self._loop_time_pub = nt.getDoubleTopic("Loop Time (ms)").publish()
        self._overrun_count_pub = nt.getDoubleTopic("Overrun Count").publish()
        self._overrun_count = 0
        self._loop_timer = wpilib.Timer()
        self._loop_timer.start()

    def robotPeriodic(self) -> None:
        start = self._loop_timer.get()
        self.container.auto_chooser.update()
        commands2.CommandScheduler.getInstance().run()
        elapsed_ms = (self._loop_timer.get() - start) * 1000.0
        self._loop_time_pub.set(elapsed_ms)
        if elapsed_ms > 20.0:
            self._overrun_count += 1
            self._overrun_count_pub.set(self._overrun_count)

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        self.autonomousCommand = self.container.getAutonomousCommand()
        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

    def teleopPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self) -> None:
        pass

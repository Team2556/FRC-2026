#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import os
import typing

import wpilib
import commands2
import phoenix6
import phoenix6.unmanaged
from phoenix6.signal_logger import SignalLogger

from pykit.wpilog.wpilogwriter import WPILOGWriter
from pykit.wpilog.wpilogreader import WPILOGReader
from pykit.networktables.nt4Publisher import NT4Publisher
from pykit.loggedrobot import LoggedRobot
from pykit.logger import Logger
from pykit.inputs.loggablepowerdistribution import LoggedPowerDistribution

import constants
from constants.canbus import kCANId
from robotcontainer import RobotContainer


class Gravedigger(LoggedRobot):

    autonomousCommand: typing.Optional[commands2.Command] = None

    def __init__(self):
        super().__init__()
        phoenix6.unmanaged.set_phoenix_diagnostics_start_time(-1)

        Logger.recordMetadata("Robot", type(self).__name__)

        match constants.kRobotMode:
            case constants.RobotModes.REAL:
                # Log deploy info from the last `python -m robotpy deploy`
                deploy_config = wpilib.deployinfo.getDeployData()
                if deploy_config is not None:
                    Logger.recordMetadata("Deploy Host",       deploy_config.get("deploy-host", ""))
                    Logger.recordMetadata("Deploy User",       deploy_config.get("deploy-user", ""))
                    Logger.recordMetadata("Deploy Date",       deploy_config.get("deploy-date", ""))
                    Logger.recordMetadata("Git Hash",          deploy_config.get("git-hash", ""))
                    Logger.recordMetadata("Git Branch",        deploy_config.get("git-branch", ""))
                    Logger.recordMetadata("Git Description",   deploy_config.get("git-desc", ""))
                Logger.addDataReciever(NT4Publisher(True))
                # Prefer USB drive (/U) for logs; fall back to pyLogs/
                usb_mount = "/U"
                if os.path.ismount(usb_mount) or os.path.exists(usb_mount):
                    os.makedirs(os.path.join(usb_mount, "logs"), exist_ok=True)
                    Logger.addDataReciever(WPILOGWriter())
                else:
                    fallback = os.path.join(wpilib.getDeployDirectory(), "..", "pyLogs")
                    os.makedirs(fallback, exist_ok=True)
                    Logger.addDataReciever(WPILOGWriter(filename=None, path=fallback))

            case constants.RobotModes.SIMULATION:
                Logger.addDataReciever(WPILOGWriter())
                Logger.addDataReciever(NT4Publisher(True))

            case constants.RobotModes.TESTING:
                # Skip WPILOGWriter — PyKit's os.rename() raises
                # FileExistsError on Windows when multiple pyfrc tests
                # share the same second-level timestamp.
                Logger.addDataReciever(NT4Publisher(True))

            case constants.RobotModes.REPLAY:
                self.useTiming = False   # replay as fast as possible
                log_path = os.path.abspath(os.environ["LOG_PATH"])
                print(f"[PyKit] Starting replay from {log_path}")
                Logger.setReplaySource(WPILOGReader(log_path))
                Logger.addDataReciever(WPILOGWriter(log_path[:-7] + "_sim.wpilog"))

        # Disable PyKit's PDH logging — CAN ID mismatch causes errors every 20ms.
        # Monkey-patch getInstance so no PowerDistribution object is ever created.
        class _NullPDH:
            def saveToTable(self, table):
                pass
        LoggedPowerDistribution.getInstance = classmethod(lambda cls: _NullPDH())
        Logger.start()
        self.container = RobotContainer()
        wpilib.CameraServer.launch()

    def robotInit(self) -> None:
        SignalLogger.enable_auto_logging(False)
        wpilib.LiveWindow.disableAllTelemetry()
        # Extend watchdog period — commands2 default (0.02 s) is too tight
        # for a full robot init; 1 s gives headroom without hiding real hangs.
        commands2.CommandScheduler.getInstance().setPeriod(1)

        # Log every command's active/inactive state to AdvantageScope
        _command_count: dict[str, int] = {}

        def _log_command(command: commands2.Command, active: bool) -> None:
            name = command.getName()
            _command_count[name] = _command_count.get(name, 0) + (1 if active else -1)
            Logger.recordOutput(f"Commands/{name}", _command_count[name] > 0)

        commands2.CommandScheduler.getInstance().onCommandInitialize(
            lambda c: _log_command(c, True)
        )
        commands2.CommandScheduler.getInstance().onCommandFinish(
            lambda c: _log_command(c, False)
        )
        commands2.CommandScheduler.getInstance().onCommandInterrupt(
            lambda c: _log_command(c, False)
        )

    def robotPeriodic(self) -> None:
        self.container.auto_chooser.update()
        commands2.CommandScheduler.getInstance().run()

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

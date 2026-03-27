"""
constants/__init__.py

Robot-wide constants and operating-mode detection.

Physical constants must have their units specified.
Default units:
    Length: meters
    Angle:  radians

Axes Convention (right hand rule):
    Translation:  +X forward, +Y left, +Z up
    Rotation:     +X CCW from front, +Y CCW from left, +Z CCW from top
"""

from enum import Enum, auto
import os
import sys

from phoenix6 import CANBus
from wpilib import RobotBase


kRobotUpdatePeriod: float = 1 / 50  # seconds  (50 Hz main loop)
kRobotUpdateFrequency: float = 1 / kRobotUpdatePeriod  # Hz

kRioCANBus = CANBus()  # default "rio" CAN bus


class RobotModes(Enum):
    REAL = auto()  # running on real robot hardware
    SIMULATION = auto()  # WPILib simulator  (python -m robotpy sim)
    REPLAY = auto()  # log-replay        (set LOG_PATH env var)
    TESTING = auto()  # pyfrc / pytest    (python -m pytest)


kSimMode: RobotModes = (
    RobotModes.REPLAY
    if "LOG_PATH" in os.environ and os.environ["LOG_PATH"] != ""
    else RobotModes.SIMULATION
)

kRobotMode: RobotModes = RobotModes.REAL if RobotBase.isReal() else kSimMode

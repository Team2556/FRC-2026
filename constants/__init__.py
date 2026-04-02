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

from phoenix6 import CANBus


kRobotUpdatePeriod: float = 1 / 50  # seconds  (50 Hz main loop)
kRobotUpdateFrequency: float = 1 / kRobotUpdatePeriod  # Hz

kRioCANBus = CANBus()  # default "rio" CAN bus

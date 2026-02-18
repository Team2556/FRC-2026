"""
Holds all of the numeric values set for different subsystems and commands.
This is not for tuning but for known values, such as a position of a field element.
"""

from wpimath.geometry import Translation2d
from util.alliance_constant import AllianceConstant


class kHub:
    POS = AllianceConstant(Translation2d(12.0, 4.0), Translation2d(4.0, 4.0))

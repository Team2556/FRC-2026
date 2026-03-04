"""
Holds all of the numeric values set for different subsystems and commands.
This is not for tuning but for known values, such as a position of a field element.
"""

from wpimath.geometry import Translation2d, Pose2d
from util.alliance_constant import AllianceConstant


class kHub:
    POS = Pose2d(4.6, 4.0, 0)

class kField:
    WIDTH = 8.07
    LENGTH = 16.54

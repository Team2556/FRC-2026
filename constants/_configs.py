"""
Holds all of the numeric values set for different subsystems and commands.
This is not for tuning but for known values, such as a position of a field element.
"""

from wpimath.geometry import Translation2d


class kShooter:
    SHOOTER_OFFSET = Translation2d(-0.029566, -0.212725)  # Meters
    SHOOTER_DIRECTION = 0


class kField:
    RED_HUB_POS = Translation2d(12.0, 4.0)

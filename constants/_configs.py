"""
Holds all of the numeric values set for different subsystems and commands.
This is not for tuning but for known values, such as a position of a field element.
"""

from wpimath.geometry import Translation2d


class kShooter:
    SHOOTER_OFFSET = Translation2d(0.0, 0.0)  # Meters
    SHOOTER_DIRECTION = -90.0


class kField:
    RED_HUB_POS = Translation2d(12.0, 4.0)

class kLimelights:
    FRONT = "limelight-front"
    RIGHT = "limelight-right"

from wpimath.units import rotationsToRadians

class kAutoAlign:
    class ROTATION_PID:
        p = 5.0 / 180
        i = 0
        d = 0

    DIRECTION_TUNING = 0.0
    SHOOTER_ACCURACY = 2
    ROBOT_VELOCITY_MULT = 0.5
    FLIGHT_TIME_SCALAR: float = 1.25


class kDriveConfig:
    MAX_SPEED = 1.0
    """Speed at 12v"""
    MAX_ANGULAR_RATE = rotationsToRadians(0.75)

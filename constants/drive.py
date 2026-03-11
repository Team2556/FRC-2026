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
    
    # Keep in mind this degree requirement is when the transfer motors start; it should be a bit more than normal
    # because it takes some time for fuels to start going up from transfer and them actually shooting at an angle
    # But ALSO keep in mind that there might be fuel just before the flywheel for example when quickly switching
    # between alliance/neutral zones
    REQUIRED_SHOOT_ACCURACY_DEGREES = 5

class kDriveConfig:
    MAX_SPEED = 1.0
    """Speed at 12v"""
    MAX_ANGULAR_RATE = rotationsToRadians(0.75)

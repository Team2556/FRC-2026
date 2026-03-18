from wpimath.units import rotationsToRadians

class kAutoAlign:
    class ROTATION_PID:
        p = 0.006
        i = 0.00005
        d = 0.00008

    DIRECTION_TUNING = 0.0
    SHOOTER_ACCURACY = 2
    ROBOT_VELOCITY_MULT = 0.5
    FLIGHT_TIME_SCALAR: float = 1
    ANGLE_TUNER = 0
    
    # Keep in mind this degree requirement is when the transfer motors start; it should be a bit more than normal
    # because it takes some time for fuels to start going up from transfer and them actually shooting at an angle
    # But ALSO keep in mind that there might be fuel just before the flywheel for example when quickly switching
    # between alliance/neutral zones
    REQUIRED_SHOOT_ACCURACY_DEGREES = 10
    AUTO_ALIGN_MAX_ANGULAR_RATE = rotationsToRadians(1.5)

class kDriveConfig:
    # Specifically used for retracting hood and stopping shooter when transitioning between zones through bump/trench
    LOOKAHEAD_SECONDS = 0.25
    
    SLOW_SPEED_MULT = 0.3
    
    SPEED_MULT = 1.0
    ROTATION_MULT = 1.0
    """Speed at 12v"""
    MAX_ANGULAR_RATE = rotationsToRadians(1.5) # This is not used for auto align rotation
    


from wpimath.units import rotationsToRadians

from subsystems.drivetrain.swerve_tuner import TunerConstants


class kAutoAlign:
    class ROTATION_PID:
        p = 0.006
        i = 0.0001
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

    ALIGN_TUNER_OFFSET = 0


class kDriveConfig:
    # Specifically used for retracting hood and stopping shooter when transitioning between zones through bump/trench
    LOOKAHEAD_SECONDS = 0.25

    SLOW_SPEED_MULT = 0.3

    # Derived from module positions — distance from robot center to farthest module
    DRIVE_BASE_RADIUS = max(
        (TunerConstants._front_left_x_pos ** 2  + TunerConstants._front_left_y_pos ** 2)  ** 0.5,
        (TunerConstants._front_right_x_pos ** 2 + TunerConstants._front_right_y_pos ** 2) ** 0.5,
        (TunerConstants._back_left_x_pos ** 2   + TunerConstants._back_left_y_pos ** 2)   ** 0.5,
        (TunerConstants._back_right_x_pos ** 2  + TunerConstants._back_right_y_pos ** 2)  ** 0.5,
    )

    # Physical max angular rate derived from max linear speed and drive-base geometry
    # This replaces the old rotationsToRadians(1.3) which was ~8.17 rad/s — way too fast
    MAX_ANGULAR_RATE = TunerConstants.speed_at_12_volts / DRIVE_BASE_RADIUS
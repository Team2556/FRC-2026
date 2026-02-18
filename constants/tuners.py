from wpimath.controller import PIDController

class kAutoAlign:
    ROTATIONAL_PID = PIDController(5.0 / 180.0, 0, 0)  # Convert the P into rotations
    DIRECTION_TUNING = 0.0
    SHOOTER_ACCURACY = 2
    ROBOT_VELOCITY_MULT = 0.5
    CORRECTION_MULT = 0.1

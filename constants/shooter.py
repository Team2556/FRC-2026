from phoenix6.configs import TalonFXConfiguration, TalonFXSConfiguration
from phoenix6.signals import InvertedValue, MotorArrangementValue
from wpimath.geometry import Transform2d, Rotation2d, Translation2d

from constants.math import kMath

class kHoodMotor:
    _CONFIG = TalonFXSConfiguration()
    _CONFIG.commutation.motor_arrangement = MotorArrangementValue.MINION_JST
    _CONFIG.slot0.k_p = 1.9
    _CONFIG.slot0.k_i = 0.1
    _CONFIG.slot0.k_d = 0.1

    _CONFIG.motion_magic.motion_magic_cruise_velocity = 10   # rotations/sec
    _CONFIG.motion_magic.motion_magic_acceleration = 20      # rotations/sec²
    _CONFIG.motion_magic.motion_magic_jerk = 100             # rotations/sec³

    # Rotor position value/second when Driver 2 is manually moving it
    INCREMENT_AMOUNT = 0.1

    RESET_HOME_SPEED = 0.03
    
    # Put more hood constants here pls
    
class kShooterMotor:
    _CONFIG = TalonFXConfiguration()
    _CONFIG.slot0.k_p = 0.1
    _CONFIG.slot0.k_i = 0.15
    _CONFIG.slot0.k_d = 0
    _CONFIG.slot1.k_p = 0.1
    _CONFIG.slot1.k_i = 10
    _CONFIG.slot1.k_d = 0
    _CONFIG.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
    
    IDLE_RPM = -1000
    TARGET_RPM = -3500
    
    REACH_TARGET_VELOCITY_ERROR = 20
    '''Defines an interval for when the shooter motor "REACHES" a goal RPM (in RPM)'''

class kShooterData:
    '''
    Measurements used for interpolation'''
    # Distance | Time
    SHOT_TIME = [
        (2, 0.2),
        (3, 0.3),
    ]
    # Distance (meters) | Hood angle (degrees)
    # Anchors: ~25° at 2m, ~35° at 4m — tune on robot
    SHOT_ANGLES = [
        (1.5, 20.0),
        (2.0, 25.0),
        (2.5, 28.0),
        (3.0, 31.0),
        (3.5, 33.5),
        (4.0, 35.0),
        (4.5, 36.5),
        (5.0, 37.5),
    ]
    

class kShooterConfig:
    SHOOTER_OFFSET = Translation2d(10.432 * kMath.MetersPerInch, 0)  # Meters
    SHOOTER_DIRECTION = -90  # 180 for Reverse
    WHEEL_RADIUS = 4 * kMath.MetersPerInch

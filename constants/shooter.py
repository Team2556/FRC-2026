import math
from phoenix6.configs import TalonFXConfiguration, TalonFXSConfiguration
from phoenix6.signals import InvertedValue, MotorArrangementValue
from wpimath.geometry import Transform2d, Rotation2d, Translation2d

from util.math import kMath


class kHoodMotor:
    _CONFIG = TalonFXSConfiguration()
    _CONFIG.commutation.motor_arrangement = MotorArrangementValue.MINION_JST
    _CONFIG.slot0.k_p = 1.9
    _CONFIG.slot0.k_i = 0.1
    _CONFIG.slot0.k_d = 0.1
    _CONFIG.motion_magic.motion_magic_cruise_velocity = 240
    _CONFIG.motion_magic.motion_magic_acceleration = 480
    _CONFIG.motion_magic.motion_magic_jerk = 2400

    RESET_HOME_SPEED = -0.3

    # Gear ratio: 15.1 motor revolutions = 35 degrees of hood travel
    GEAR_RATIO = 15.1 / 35.0  # revolutions per degree
    DEGREES_PER_REVOLUTION = 1 / GEAR_RATIO

    # Positions in degrees — converted to revolutions where needed
    HOME_ANGLE_DEG = 15.0
    MAX_ANGLE_DEG = 52.0
    REACH_TARGET_ANGLE_ERROR = 2.0  # degrees
    
    INNER_RING_ANGLE = 13.9
    OUTER_RING_ANGLE = 21.2

    # Dashboard override defaults
    OVERRIDE_ENABLED = False
    OVERRIDE_ANGLE_DEG = 25.0
    
    OPPOSING_ANGLE_DEG = 50.0
    
    TUNER_OFFSET = 0
    
    @staticmethod
    def to_revs(degrees: float) -> float:
        return degrees * kHoodMotor.GEAR_RATIO

    @staticmethod
    def to_deg(revs: float) -> float:
        return revs * kHoodMotor.DEGREES_PER_REVOLUTION


class kShooterMotor:
    _CONFIG = TalonFXConfiguration()
    
    _CONFIG.slot0.k_p = 0.5
    _CONFIG.slot0.k_i = 0.0
    _CONFIG.slot0.k_d = 0.0
    _CONFIG.slot0.k_v = 0.1125
    _CONFIG.slot0.k_s = 0.18

    _CONFIG.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

    TARGET_RPM = -2900
    REACH_TARGET_VELOCITY_ERROR = 50
    """Defines an interval for when the shooter motor "REACHES" a goal RPM (in RPM)"""


class kShooterData:
    """Measured shot data used for distance interpolation.

    Both tables share the same distance column (meters to hub).
    Values outside the measured range are clamped to the nearest endpoint
    by numpy.interp.
    """

    # (distance_m, hood_angle_deg)
    SHOT_ANGLES = [
        (0.95, 19.0),
        (1.51, 22.0),
        (2.00, 27.0),
        (2.53, 30.0),
        (3.06, 34.5),
        (3.53, 35.0),
        (4.03, 36.5),
        (4.50, 37.5),
        (5.03, 40.0),
        (7.43, 45.0),
    ]

    # (distance_m, shooter_rpm)  — negative because motor is inverted
    SHOT_RPMS = [
        (0.95, -2500),
        (1.51, -2600),
        (2.00, -2600),
        (2.53, -2800),
        (3.06, -2900),
        (3.53, -3000),
        (4.03, -3150),
        (4.50, -3300),
        (5.03, -3500),
        (7.43, -4300),
        (10, -6000),
    ]


class kShooterConfig:
    # Negative X = toward back of robot where shooter sits _FCC_
    SHOOTER_OFFSET = Translation2d(-10.432 * kMath.MetersPerInch, 0)  # Meters
    SHOOTER_DIRECTION = -90  # 180 for Reverse
    WHEEL_RADIUS = 4 * kMath.MetersPerInch

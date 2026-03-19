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
    HOME_ANGLE_DEG = 5.0
    MAX_ANGLE_DEG = 35.0
    REACH_TARGET_ANGLE_ERROR = 2.0  # degrees

    # Dashboard override defaults
    OVERRIDE_ENABLED = False
    OVERRIDE_ANGLE_DEG = 25.0
    
    OPPOSING_ANGLE_DEG = 35.0

    @staticmethod
    def to_revs(degrees: float) -> float:
        return degrees * kHoodMotor.GEAR_RATIO

    @staticmethod
    def to_deg(revs: float) -> float:
        return revs * kHoodMotor.DEGREES_PER_REVOLUTION


class kShooterMotor:
    _CONFIG = TalonFXConfiguration()
    
    _CONFIG.slot0.k_p = 0.2
    _CONFIG.slot0.k_i = 0.2
    _CONFIG.slot0.k_d = 0.0
    
    _CONFIG.slot1.k_p = 0.5
    _CONFIG.slot1.k_i = 1.15
    _CONFIG.slot1.k_d = 0.0085
    
    _CONFIG.slot2.k_p = 0.1
    _CONFIG.slot2.k_i = 5   
    _CONFIG.slot2.k_d = 0.017
    
    _CONFIG.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

    IDLE_RPM = -1000
    TARGET_RPM = -3700
    TUNED_RPM = 0
    
    CURRENT_TARGET_RPM = TARGET_RPM
    TARGET_RPM_FAR = -6000

    REACH_TARGET_VELOCITY_ERROR = 20
    """Defines an interval for when the shooter motor "REACHES" a goal RPM (in RPM)"""


class kShooterData:
    """
    Measurements used for interpolation"""

    # Distance | Time
    SHOT_TIME = [ (2.66, 1.2), (2.80, 1.3), (3.60, 0.92), (4.66, 1)]
    # Distance (meters) | Hood angle (degrees)
    # Anchors: ~25° at 2m, ~35° at 4m — tune on robot
    SHOT_ANGLES = [ (2.66, 15), (2.80, 15.5), (3.60, 18.85), (4.66, 22.0)]


class kShooterConfig:
    # Negative X = toward back of robot where shooter sits _FCC_
    SHOOTER_OFFSET = Translation2d(-10.432 * kMath.MetersPerInch, 0)  # Meters
    SHOOTER_DIRECTION = -90  # 180 for Reverse
    WHEEL_RADIUS = 4 * kMath.MetersPerInch

from phoenix6.configs import TalonFXConfiguration
from wpimath.geometry import Transform2d, Rotation2d

from constants.math import kMath

class kShooterMotor:
    CAN_ID = 24
    _CONFIG = TalonFXConfiguration()
    _CONFIG.slot0.k_p = 0.1
    _CONFIG.slot0.k_i = 0.15
    _CONFIG.slot0.k_d = 0
    TARGET_RPM = -2500

class kShooterConfig:
    SHOOTER_OFFSET = Transform2d(-0.029566, -0.212725, Rotation2d())  # Meters
    SHOOTER_DIRECTION = 0 # 180 for Reverse
    WHEEL_RADIUS = 4 * kMath.MetersPerInch
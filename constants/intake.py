from phoenix6.configs import TalonFXConfiguration, TalonFXSConfiguration
from phoenix6.signals import MotorArrangementValue


class kIntakePivot:
    _CONFIG = TalonFXSConfiguration()
    _CONFIG.commutation.motor_arrangement = MotorArrangementValue.MINION_JST
    _CONFIG.slot0.k_p = .1
    _CONFIG.slot0.k_i = 0.5
    _CONFIG.slot0.k_d = 0.15
    _CONFIG.slot0.k_v = 1.5
    _CONFIG.slot0.k_s = 0.1

    _CONFIG.slot1.k_p = 0.1
    _CONFIG.slot1.k_i = 0.01
    _CONFIG.slot1.k_d = 0
    _CONFIG.slot1.k_v = 1.5

    _CONFIG.motion_magic.motion_magic_cruise_velocity = 25   # rotations/sec
    _CONFIG.motion_magic.motion_magic_acceleration = 20     # rotations/sec²
    _CONFIG.motion_magic.motion_magic_jerk = 20             # rotations/sec³

    DEPLOYED_POSITION = 10.0

    # Which robot-relative axis the intake faces.
    # 1.0  = robot front (+X)   -1.0 = robot rear (-X)
    INTAKE_DIRECTION = 1.0


class kIntakeRoller:
    _CONFIG = TalonFXConfiguration()
    _CONFIG.slot0.k_p = 2
    _CONFIG.slot0.k_i = 0.3
    _CONFIG.slot0.k_d = 0
    _CONFIG.slot0.k_v = 0.12
    _CONFIG.slot0.k_s = 0.1

    TARGET_RPM = -4750

    # Minimum robot speed toward the intake side (m/s) before the default
    # command auto-spins the rollers.
    AUTO_INTAKE_SPEED_THRESHOLD = 0.05

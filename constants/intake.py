from phoenix6.configs import TalonFXConfiguration, Slot0Configs, TalonFXSConfiguration
from phoenix6.signals import MotorArrangementValue


class kIntakePivot:
    _CONFIG = TalonFXSConfiguration()
    _CONFIG.commutation.motor_arrangement = MotorArrangementValue.MINION_JST
    _CONFIG.slot0.k_p = 15.75
    _CONFIG.slot0.k_i = 2.25
    _CONFIG.slot0.k_d = 0

    _CONFIG.slot1.k_p = 0.1
    _CONFIG.slot1.k_i = 0.01
    _CONFIG.slot1.k_d = 0

    _CONFIG.motion_magic.motion_magic_cruise_velocity = 152  # rotations/sec
    _CONFIG.motion_magic.motion_magic_acceleration = 507     # rotations/sec²
    _CONFIG.motion_magic.motion_magic_jerk = 1013            # rotations/sec³

    DEPLOYED_POSITION = 10.0


class kIntakeRoller:
    _CONFIG = TalonFXConfiguration()
    _CONFIG.slot0.k_p = 5
    _CONFIG.slot0.k_i = 5
    _CONFIG.slot0.k_d = 0

    TARGET_RPM = -2475

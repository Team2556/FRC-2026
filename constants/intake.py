from phoenix6.configs import TalonFXConfiguration, Slot0Configs, TalonFXSConfiguration
from phoenix6.signals import MotorArrangementValue


class kIntakeDeployer:
    _CONFIG = TalonFXSConfiguration()
    _CONFIG.commutation.motor_arrangement = MotorArrangementValue.MINION_JST
    _CONFIG.slot0.k_p = 10.5
    _CONFIG.slot0.k_i = 1.5
    _CONFIG.slot0.k_d = 0

    _CONFIG.slot1.k_p = 0.1
    _CONFIG.slot1.k_i = 0.01
    _CONFIG.slot1.k_d = 0

    _CONFIG.motion_magic.motion_magic_cruise_velocity = 20   # rotations/sec
    _CONFIG.motion_magic.motion_magic_acceleration = 60      # rotations/sec²
    _CONFIG.motion_magic.motion_magic_jerk = 100             # rotations/sec³

    DEPLOYED_POSITION = 10.0


class kIntakeSpinner:
    _CONFIG = TalonFXConfiguration()
    _CONFIG.slot0.k_p = 5
    _CONFIG.slot0.k_i = 5
    _CONFIG.slot0.k_d = 0

    TARGET_RPM = -1500

from phoenix6.configs import TalonFXConfiguration


class kShooterMotor:
    CAN_ID = 24
    _CONFIG = TalonFXConfiguration()
    _CONFIG.slot0.k_p = 0.1
    _CONFIG.slot0.k_i = 0.15
    _CONFIG.slot0.k_d = 0
    TARGET_RPM = -2500
    
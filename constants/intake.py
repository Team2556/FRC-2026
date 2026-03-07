from phoenix6.configs import TalonFXConfiguration, Slot0Configs


class kIntakeDeployer:
    _CONFIG = TalonFXConfiguration()
    _CONFIG.slot0.k_p = 0.4
    _CONFIG.slot0.k_i = 0
    _CONFIG.slot0.k_d = 0

    _CONFIG.slot1.k_p = 0.1
    _CONFIG.slot1.k_i = 0
    _CONFIG.slot1.k_d = 0

    LEFT_CAN_ID = 0
    RIGHT_CAN_ID = 0
    # right follows left using Follower in intake.py subsystems line 27
    
    STATE = "undeployed"
    INITIAL_POSITION = 0
    DEPLOYED_POSITION = 0.5


class kIntakeSpinner:
    _CONFIG = TalonFXConfiguration()
    _CONFIG.slot0.k_p = 0
    _CONFIG.slot0.k_i = 6
    _CONFIG.slot0.k_d = 0

    CAN_ID = 0
    TARGET_RPS = 1
    TARGET_RPM = -3600

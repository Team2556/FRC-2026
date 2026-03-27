from phoenix6.configs import TalonFXConfiguration


class kClimb:
    _CONFIG = TalonFXConfiguration()
    _CONFIG.slot0.k_p = 0.2
    _CONFIG.slot0.k_i = 0
    _CONFIG.slot0.k_d = 0

    POSITION_DOWN = 0
    POSITION_UP = 1
    
   
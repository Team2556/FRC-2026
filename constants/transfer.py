from phoenix6.configs import TalonFXConfiguration, Slot0Configs


class kSpindexer:
    _CONFIG = TalonFXConfiguration()
    _CONFIG.slot0.k_p = 0.1
    _CONFIG.slot0.k_i = 5
    _CONFIG.slot0.k_d = 0

    TARGET_RPM = 6000

class kTrasnfer:
    _CONFIG = TalonFXConfiguration()
    _CONFIG.slot0.k_p = 0.1
    _CONFIG.slot0.k_i = 5
    _CONFIG.slot0.k_d = 0

    TARGET_RPM = -6000

from phoenix6.configs import TalonFXConfiguration, Slot0Configs


class kSpindexer:
    _CONFIG = TalonFXConfiguration()
    _CONFIG.slot0.k_p = 0.1
    _CONFIG.slot0.k_i = 5
    _CONFIG.slot0.k_d = 0

    CAN_ID = 27
    TARGET_RPM = 6000

class kTrasnfer:
    class motor_1:
        _CONFIG = TalonFXConfiguration()
        _CONFIG.slot0.k_p = 0.1
        _CONFIG.slot0.k_i = 5

        CAN_ID = 20
        TARGET_RPM = -6000

    class motor_2:
        _CONFIG = TalonFXConfiguration()
        _CONFIG.slot0.k_p = 0.1
        _CONFIG.slot0.k_i = 5

        CAN_ID = 21
        TARGET_RPM = -6000

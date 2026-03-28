from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals import InvertedValue


class kSpindexer:
    _CONFIG = TalonFXConfiguration()
    _CONFIG.slot0.k_p = 0.1
    _CONFIG.slot0.k_i = 5
    _CONFIG.slot0.k_d = 0
    
    _CONFIG.current_limits.supply_current_limit = 40
    _CONFIG.current_limits.supply_current_lower_limit = 15

    TARGET_RPM = 6000

class kTransfer:
    _CONFIG = TalonFXConfiguration()
    _CONFIG.slot0.k_p = 1
    _CONFIG.slot0.k_i = 0.5
    _CONFIG.slot0.k_d = 0
    _CONFIG.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

    TARGET_RPM = 1400

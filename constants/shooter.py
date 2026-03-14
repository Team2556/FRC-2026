from phoenix6.configs import TalonFXConfiguration
from wpimath.geometry import Transform2d, Rotation2d, Translation2d

from constants.math import kMath

class kHoodMotor:
    _CONFIG = TalonFXConfiguration()
    _CONFIG.slot0.k_p = 0.2
    _CONFIG.slot0.k_i = 0
    _CONFIG.slot0.k_d = 0
    
    CAN_ID = 0
    
    # Rotor position value/second when Driver 2 is manually moving it
    INCREMENT_AMOUNT = 0.1
    
    # Put more hood constants here pls
    
class kShooterMotor:
    _CONFIG = TalonFXConfiguration()
    _CONFIG.slot0.k_p = 0.1
    _CONFIG.slot0.k_i = 0.15
    _CONFIG.slot0.k_d = 0
    _CONFIG.slot1.k_p = 0.1
    _CONFIG.slot1.k_i = 10
    _CONFIG.slot1.k_p = 0
    
    IDLE_RPM = -1000
    TARGET_RPM = -2500
    
    REACH_TARGET_VELOCITY_ERROR = 20
    '''Defines an interval for when the shooter motor "REACHES" a goal RPM (in RPM)'''

class kShooterData:
    '''
    Measurements used for interpolation'''
    # Distance | Time
    SHOT_TIME = [
        (2, 0.7)
        (3, 1),
    ]
    # Distance | Position Voltage
    SHOT_ANGLES = [
        (2, 1)
    ]
    

class kShooterConfig:
    SHOOTER_OFFSET = Translation2d(-0.029566, -0.212725)  # Meters
    SHOOTER_DIRECTION = 0  # 180 for Reverse
    WHEEL_RADIUS = 4 * kMath.MetersPerInch

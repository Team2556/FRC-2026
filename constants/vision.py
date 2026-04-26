from util.math import kMath


class kCamera:
    LOCATION_PUBLISHER_KEY = "camera/location"
    SHOOTER_LL = "limelight-shooter"
    BACK_LL = "limelight-back"


class kOdometry:
    USE_MEGATAG2 = True 
    
    MAX_RPS = 2
    MAX_TAG_DIST = 7.0
    MAX_TAG_AMBIGUITY = 0.9

    MT1_MIN_APRILTAGS = 2
    MT1_XY_COEFF    = 0.3
    MT1_THETA_COEFF = 0.5
    
    MT1_RESET_MAX_AMBIGUITY = 0.2
    MT1_RESET_MAX_TAG_DIST  = 2.0
    MT1_RESET_XY_STD    = 0.1
    MT1_RESET_THETA_STD = 0.1 

    MT2_MIN_APRILTAGS = 1
    MT2_XY_COEFF      = 0.4
    MT2_THETA_STD_DEV = 9999999.0
    MT2_MAX_POSE_ERROR = 1

    MAX_TILT_ERROR = 5
    IGNORE_TILT = True

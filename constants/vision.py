from wpimath.geometry import Rotation2d, Transform3d

from .math import kMath


class kCamera:
    LOCATION_PUBLISHER_KEY = "camera/location"
    SHOOTER_LL = "limelight-shooter"
    BACK_LL = "limelight-back"


class kOdometry:
    MAX_VISION_AMBIGUITY = 0.3
    MAX_VISION_Z_ERROR = 0.75
    XY_SD_COEFF = 0.02
    THETA_SD_COEFF = 0.06

    MAX_RPS = 2  # m/s
    USE_MEGATAG_2 = False

    MIN_APRILTAGS = 2
    MAX_TILT_ERROR = 1

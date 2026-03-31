from util.math import kMath


class kCamera:
    LOCATION_PUBLISHER_KEY = "camera/location"
    SHOOTER_LL = "limelight-shooter"
    BACK_LL = "limelight-back"


class kOdometry:
    MAX_RPS = 2  # rotations/s — discard measurements above this angular rate

    USE_MEGATAG2 = True  # False = MegaTag1, True = MegaTag2 (also NT-tunable at runtime)

    # Maximum average tag distance to accept a measurement (meters).
    # Pose error grows quickly beyond this — reject rather than pollute the estimator.
    MAX_TAG_DIST = 7.0

    # Per-tag ambiguity threshold (0–1) for normal soft updates.
    # Any estimate containing a tag above this is rejected entirely.
    MAX_TAG_AMBIGUITY = 0.9

    # Stricter thresholds for the MT1 hard pose reset.
    # Only a very clean multi-tag view at close range triggers a full reset.
    MT1_RESET_MAX_AMBIGUITY = 0.2   # very unambiguous tags only
    MT1_RESET_MAX_TAG_DIST  = 4.0   # meters — close tags only

    # MT1 needs ≥2 tags for reliable yaw; MT2 works with 1
    MT2_MIN_APRILTAGS = 1
    MT1_MIN_APRILTAGS = 2

    # Per-measurement std devs are scaled by (COEFF * avg_tag_dist / tag_count).
    # Closer cameras with more tags get tighter (more trusted) updates.
    MT2_XY_COEFF    = 0.4
    MT2_THETA_STD_DEV = 9_999_999.0  # MT2 never corrects yaw — gyro owns it

    MT1_XY_COEFF    = 0.3
    MT1_THETA_COEFF = 0.5

    MAX_TILT_ERROR = 5
    IGNORE_TILT = True

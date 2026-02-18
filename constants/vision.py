from os import path
import wpilib
from wpimath.geometry import Pose3d, Rotation2d, Rotation3d, Transform3d
from robotpy_apriltag import AprilTagFieldLayout

from .math import kMath


class kLimelight:
    TARGET_INVALID_VALUE = 0.0
    TARGET_VALID_VALUE = 1.0
    MIN_HORIZONTAL_FOV = Rotation2d.fromDegrees(-29.8)
    MAX_HORIZONTAL_FOV = Rotation2d.fromDegrees(29.8)
    MIN_VERTICAL_FOV = Rotation2d.fromDegrees(-22.85)
    MAX_VERTICAL_FOV = Rotation2d.fromDegrees(22.85)
    NETWORK_TABLENAME = "limelight"
    TARGET_VALID_KEY = "tv"
    TARGET_HORIZONTAL_ANGLE_KEY = "tx"
    TARGET_VERTICAL_ANGLE_KEY = "ty"
    LED_MODE_KEY = "ledMode"
    TRACKER_MODULE_NAME = "limelight"

    FOV_HORIZONTAL = 75.9  # degrees
    FOVVERTICAL = 47.4  # degrees


class kCamera:
    LOCATION_PUBLISHER_KEY = "camera/location"

    class llFront:
        NAME = "limelight-front"
        ROBOT_TO_CAMERA_TRANSFORM = Transform3d(
            Pose3d(),
            Pose3d(
                0,
                0.2413,
                0.33655,
                Rotation3d.fromDegrees(0, 15, 0),
            ),
        )

    class llRight:
        NAME = "limelight-right"
        ROBOT_TO_CAMERA_TRANSFORM = Transform3d(
            Pose3d(),
            Pose3d(
                0.30575,
                0.2286,
                0.1143,
                Rotation3d.fromDegrees(0, 14.76, -90),
            ),
        )


class kOdometry:
    MAX_VISION_AMBIGUITY = 0.3
    MAX_VISION_Z_ERROR = 0.75
    XY_SD_COEFF = 0.02
    THETA_SD_COEFF = 0.06

    MAX_RPS = 2  # m/s
    USE_MEGATAG_2 = False


class kAprilTag:
    FIELD_LAYOUT = AprilTagFieldLayout(
        path.join(
            wpilib.getDeployDirectory(), "apriltags", "2026-rebuilt-andymark.json"
        )
    )

    POSITIONS = {
        1: Pose3d(
            (kMath.M_INCH * 657.37),
            (kMath.M_INCH * 25.80),
            (kMath.M_INCH * 58.50),
            Rotation3d(0.0, 0.0, 126 * kMath.RAD_DEG),
        ),
        2: Pose3d(
            (kMath.M_INCH * 657.37),
            (kMath.M_INCH * 291.20),
            (kMath.M_INCH * 58.50),
            Rotation3d(0.0, 0.0, 234 * kMath.RAD_DEG),
        ),
        3: Pose3d(
            (kMath.M_INCH * 455.15),
            (kMath.M_INCH * 317.15),
            (kMath.M_INCH * 51.25),
            Rotation3d(0.0, 0.0, 270 * kMath.RAD_DEG),
        ),
        4: Pose3d(
            (kMath.M_INCH * 365.20),
            (kMath.M_INCH * 241.64),
            (kMath.M_INCH * 73.54),
            Rotation3d(0.0, 30 * kMath.RAD_DEG, 0.0),
        ),
        5: Pose3d(
            (kMath.M_INCH * 365.20),
            (kMath.M_INCH * 75.39),
            (kMath.M_INCH * 73.54),
            Rotation3d(0.0, 30 * kMath.RAD_DEG, 0.0),
        ),
        6: Pose3d(
            (kMath.M_INCH * 530.49),
            (kMath.M_INCH * 130.17),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 300 * kMath.RAD_DEG),
        ),
        7: Pose3d(
            (kMath.M_INCH * 546.87),
            (kMath.M_INCH * 158.50),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 0.0),
        ),
        8: Pose3d(
            (kMath.M_INCH * 530.49),
            (kMath.M_INCH * 186.83),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 60 * kMath.RAD_DEG),
        ),
        9: Pose3d(
            (kMath.M_INCH * 497.77),
            (kMath.M_INCH * 186.83),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 120 * kMath.RAD_DEG),
        ),
        10: Pose3d(
            (kMath.M_INCH * 481.39),
            (kMath.M_INCH * 158.50),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 180 * kMath.RAD_DEG),
        ),
        11: Pose3d(
            (kMath.M_INCH * 497.77),
            (kMath.M_INCH * 130.17),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 240 * kMath.RAD_DEG),
        ),
        12: Pose3d(
            (kMath.M_INCH * 33.51),
            (kMath.M_INCH * 25.80),
            (kMath.M_INCH * 58.50),
            Rotation3d(0.0, 0.0, 54 * kMath.RAD_DEG),
        ),
        13: Pose3d(
            (kMath.M_INCH * 33.51),
            (kMath.M_INCH * 291.20),
            (kMath.M_INCH * 58.50),
            Rotation3d(0.0, 0.0, 306 * kMath.RAD_DEG),
        ),
        14: Pose3d(
            (kMath.M_INCH * 325.68),
            (kMath.M_INCH * 241.64),
            (kMath.M_INCH * 73.54),
            Rotation3d(0.0, 30 * kMath.RAD_DEG, 180 * kMath.RAD_DEG),
        ),
        15: Pose3d(
            (kMath.M_INCH * 325.68),
            (kMath.M_INCH * 75.39),
            (kMath.M_INCH * 73.54),
            Rotation3d(0.0, 30 * kMath.RAD_DEG, 180 * kMath.RAD_DEG),
        ),
        16: Pose3d(
            (kMath.M_INCH * 235.73),
            (kMath.M_INCH * -0.15),
            (kMath.M_INCH * 51.25),
            Rotation3d(0.0, 0.0, 90 * kMath.RAD_DEG),
        ),
        17: Pose3d(
            (kMath.M_INCH * 160.39),
            (kMath.M_INCH * 130.17),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 240 * kMath.RAD_DEG),
        ),
        18: Pose3d(
            (kMath.M_INCH * 144.00),
            (kMath.M_INCH * 158.50),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 180 * kMath.RAD_DEG),
        ),
        19: Pose3d(
            (kMath.M_INCH * 160.39),
            (kMath.M_INCH * 186.83),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 120 * kMath.RAD_DEG),
        ),
        20: Pose3d(
            (kMath.M_INCH * 193.10),
            (kMath.M_INCH * 186.83),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 60 * kMath.RAD_DEG),
        ),
        21: Pose3d(
            (kMath.M_INCH * 209.49),
            (kMath.M_INCH * 158.50),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 0.0),
        ),
        22: Pose3d(
            (kMath.M_INCH * 193.10),
            (kMath.M_INCH * 130.17),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 300 * kMath.RAD_DEG),
        ),
    }

    POSITIONS_ANDYMARK = {
        1: Pose3d(
            (kMath.M_INCH * 656.98),
            (kMath.M_INCH * 24.73),
            (kMath.M_INCH * 58.50),
            Rotation3d(0.0, 0.0, 126 * kMath.RAD_DEG),
        ),
        2: Pose3d(
            (kMath.M_INCH * 656.98),
            (kMath.M_INCH * 291.90),
            (kMath.M_INCH * 58.50),
            Rotation3d(0.0, 0.0, 234 * kMath.RAD_DEG),
        ),
        3: Pose3d(
            (kMath.M_INCH * 452.40),
            (kMath.M_INCH * 316.21),
            (kMath.M_INCH * 51.25),
            Rotation3d(0.0, 0.0, 270 * kMath.RAD_DEG),
        ),
        4: Pose3d(
            (kMath.M_INCH * 365.20),
            (kMath.M_INCH * 241.44),
            (kMath.M_INCH * 73.54),
            Rotation3d(0.0, 30 * kMath.RAD_DEG, 0.0),
        ),
        5: Pose3d(
            (kMath.M_INCH * 365.20),
            (kMath.M_INCH * 75.19),
            (kMath.M_INCH * 73.54),
            Rotation3d(0.0, 30 * kMath.RAD_DEG, 0.0),
        ),
        6: Pose3d(
            (kMath.M_INCH * 530.49),
            (kMath.M_INCH * 129.97),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 300 * kMath.RAD_DEG),
        ),
        7: Pose3d(
            (kMath.M_INCH * 546.87),
            (kMath.M_INCH * 158.30),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 0.0),
        ),
        8: Pose3d(
            (kMath.M_INCH * 530.49),
            (kMath.M_INCH * 186.63),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 60 * kMath.RAD_DEG),
        ),
        9: Pose3d(
            (kMath.M_INCH * 497.77),
            (kMath.M_INCH * 186.63),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 120 * kMath.RAD_DEG),
        ),
        10: Pose3d(
            (kMath.M_INCH * 481.39),
            (kMath.M_INCH * 158.30),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 180 * kMath.RAD_DEG),
        ),
        11: Pose3d(
            (kMath.M_INCH * 497.77),
            (kMath.M_INCH * 129.97),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 240 * kMath.RAD_DEG),
        ),
        12: Pose3d(
            (kMath.M_INCH * 33.91),
            (kMath.M_INCH * 24.73),
            (kMath.M_INCH * 58.50),
            Rotation3d(0.0, 0.0, 54 * kMath.RAD_DEG),
        ),
        13: Pose3d(
            (kMath.M_INCH * 33.91),
            (kMath.M_INCH * 291.90),
            (kMath.M_INCH * 58.50),
            Rotation3d(0.0, 0.0, 306 * kMath.RAD_DEG),
        ),
        14: Pose3d(
            (kMath.M_INCH * 325.68),
            (kMath.M_INCH * 241.44),
            (kMath.M_INCH * 73.54),
            Rotation3d(0.0, 30 * kMath.RAD_DEG, 180 * kMath.RAD_DEG),
        ),
        15: Pose3d(
            (kMath.M_INCH * 325.68),
            (kMath.M_INCH * 75.19),
            (kMath.M_INCH * 73.54),
            Rotation3d(0.0, 30 * kMath.RAD_DEG, 180 * kMath.RAD_DEG),
        ),
        16: Pose3d(
            (kMath.M_INCH * 238.49),
            (kMath.M_INCH * 0.42),
            (kMath.M_INCH * 51.25),
            Rotation3d(0.0, 0.0, 90 * kMath.RAD_DEG),
        ),
        17: Pose3d(
            (kMath.M_INCH * 160.39),
            (kMath.M_INCH * 129.97),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 240 * kMath.RAD_DEG),
        ),
        18: Pose3d(
            (kMath.M_INCH * 144.00),
            (kMath.M_INCH * 158.30),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 180 * kMath.RAD_DEG),
        ),
        19: Pose3d(
            (kMath.M_INCH * 160.39),
            (kMath.M_INCH * 186.63),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 120 * kMath.RAD_DEG),
        ),
        20: Pose3d(
            (kMath.M_INCH * 193.10),
            (kMath.M_INCH * 186.63),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 60 * kMath.RAD_DEG),
        ),
        21: Pose3d(
            (kMath.M_INCH * 209.49),
            (kMath.M_INCH * 158.30),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 0.0),
        ),
        22: Pose3d(
            (kMath.M_INCH * 193.10),
            (kMath.M_INCH * 129.97),
            (kMath.M_INCH * 12.13),
            Rotation3d(0.0, 0.0, 300 * kMath.RAD_DEG),
        ),
    }

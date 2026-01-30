from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath import units
from ntcore import DoubleArrayEntry, NetworkTableInstance, NetworkTable

class PoseEstimate:
    def __init__(
        self,
        pose=None,
        timestamp_seconds=0.0,
        latency=0.0,
        tag_count=0,
        tag_span=0.0,
        avg_tag_dist=0.0,
        avg_tag_area=0.0,
        raw_fiducials=None,
        is_megatag2=False,
    ):
        self.pose = pose if pose is not None else Pose2d()
        self.timestampSeconds = timestamp_seconds
        self.latency = latency
        self.tagCount = tag_count
        self.tagSpan = tag_span
        self.avgTagDist = avg_tag_dist
        self.avgTagArea = avg_tag_area
        self.rawFiducials = raw_fiducials if raw_fiducials is not None else []
        self.isMegaTag2 = is_megatag2


class RawFiducial:
    def __init__(
        self,
        fid: int = 0,
        txnc: float = 0.0,
        tync: float = 0.0,
        ta: float = 0.0,
        dist_to_camera: float = 0.0,
        dist_to_robot: float = 0.0,
        ambiguity: float = 0.0,
    ):
        self.id = fid
        self.txnc = txnc
        self.tync = tync
        self.ta = ta
        self.distToCamera = dist_to_camera
        self.distToRobot = dist_to_robot
        self.ambiguity = ambiguity


def sanitize_name(name: str | None) -> str:
    if name is None or name == "":
        return "limelight"
    return name


def get_limelight_nt_table(table_name: str) -> NetworkTable:
    return NetworkTableInstance.getDefault().getTable(sanitize_name(table_name))


def get_limelight_double_array_entry(
    table_name: str, entry_name: str
) -> DoubleArrayEntry:
    key = f"{table_name}/{entry_name}"
    table = get_limelight_nt_table(table_name)
    return table.getDoubleArrayTopic(entry_name).getEntry([])


def to_pose2d(in_data):
    if len(in_data) < 6:
        # Bad LL 2D Pose Data
        return Pose2d()

    tran2d = Translation2d(in_data[0], in_data[1])
    r2d = Rotation2d(units.degreesToRadians(in_data[5]))
    return Pose2d(tran2d, r2d)


def extract_array_entry(in_data, position):
    return in_data[position] if position < len(in_data) else 0.0


def get_bot_pose_estimate(limelight_name: str, entry_name: str, is_megatag2: bool):
    pose_entry = get_limelight_double_array_entry(limelight_name, entry_name)

    ts_value = pose_entry.getAtomic()
    pose_array = ts_value.value
    timestamp = ts_value.time

    if len(pose_array) == 0:
        # Handle the case where no data is available
        return None  # or some default PoseEstimate

    pose = to_pose2d(pose_array)
    latency = extract_array_entry(pose_array, 6)
    tag_count = int(extract_array_entry(pose_array, 7))
    tag_span = extract_array_entry(pose_array, 8)
    tag_dist = extract_array_entry(pose_array, 9)
    tag_area = extract_array_entry(pose_array, 10)

    # Convert server timestamp from microseconds to seconds and adjust for latency
    adjusted_timestamp = (timestamp / 1_000_000.0) - (latency / 1000.0)

    raw_fiducials = [None] * tag_count
    vals_per_fiducial = 7
    expected_total_vals = 11 + vals_per_fiducial * tag_count

    if len(pose_array) == expected_total_vals:
        for i in range(tag_count):
            base_index = 11 + (i * vals_per_fiducial)
            fid = int(pose_array[base_index])
            txnc = pose_array[base_index + 1]
            tync = pose_array[base_index + 2]
            ta = pose_array[base_index + 3]
            dist_to_camera = pose_array[base_index + 4]
            dist_to_robot = pose_array[base_index + 5]
            ambiguity = pose_array[base_index + 6]

            raw_fiducials[i] = RawFiducial(
                fid, txnc, tync, ta, dist_to_camera, dist_to_robot, ambiguity
            )
    # else: don't populate fiducials

    return PoseEstimate(
        pose,
        adjusted_timestamp,
        latency,
        tag_count,
        tag_span,
        tag_dist,
        tag_area,
        raw_fiducials,
        is_megatag2,
    )

def set_robot_orientation(limelight_name, yaw, yaw_rate,
                          pitch, pitch_rate,
                          roll, roll_rate):
    set_robot_orientation_internal(
        limelight_name,
        yaw, yaw_rate,
        pitch, pitch_rate,
        roll, roll_rate,
        False
    )

def set_robot_orientation_internal(limelight_name, yaw, yaw_rate,
                                   pitch, pitch_rate,
                                   roll, roll_rate, flush):
    entries = [
        yaw,
        yaw_rate,
        pitch,
        pitch_rate,
        roll,
        roll_rate
    ]

    set_limelight_nt_double_array(
        limelight_name,
        "robot_orientation_set",
        entries
    )

    if flush:
        flush()

def set_limelight_nt_double_array(table_name, entry_name, val):
    get_limelight_nt_table_entry(table_name, entry_name).setDoubleArray(val)

def get_limelight_nt_table_entry(table_name, entry_name):
    return get_limelight_nt_table(table_name).getEntry(entry_name)

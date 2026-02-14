from dataclasses import dataclass, field
from enum import Enum
from typing import List
from wpimath.geometry import Pose2d, Pose3d, Transform3d
from wpiutil.wpistruct import make_wpistruct


class ObservationType(Enum):
    MEGATAG_1 = 0
    MEGATAG_2 = 1
    PHOTONVISION = 2


@make_wpistruct(name="visionobservation")
@dataclass
class VisionSubsystemPoseObservation:
    timestamp: float = 0
    pose: Pose3d = field(default_factory=Pose3d)
    ambiguity: float = 0
    tagCount: int = 0
    averageTagDistance: float = 0
    observationType: int = ObservationType.PHOTONVISION.value

class VisionSubsystemIO:

    @dataclass
    class VisionSubsystemIOInputs:
        connected: bool = False
        poseObservations: list[VisionSubsystemPoseObservation] = field(
            default_factory=list
        )
        tagIds: List[int] = field(default_factory=list)

    def updateInputs(self, inputs: VisionSubsystemIOInputs):
        pass

    def updateCameraPosition(self, transform: Transform3d) -> None:
        raise NotImplementedError("Must be implemented by subclass")

class VisionObservation:
    """
    Represents a vision measurement for the robot pose estimator.
    """

    def __init__(self, visionPose: Pose2d, timestamp: float, std: list[float]) -> None:
        assert len(std) == 3
        self.visionPose = visionPose
        self.timestamp = timestamp
        self.std = std
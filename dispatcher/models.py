from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional

class TrajectoryStatus(str, Enum):
    IDLE = "IDLE"
    QUEUED = "QUEUED"
    RUNNING = "RUNNING"
    COMPLETED = "COMPLETED"
    ABORTED = "ABORTED"
    CANCELLED = "CANCELLED"

@dataclass
class KinematicLimits:
    min_pos: float
    max_pos: float
    max_vel: float
    max_acc: float

@dataclass
class JointTrajectoryPoint:
    time_from_start: float
    positions: List[float]
    velocities: Optional[List[float]] = None
    accelerations: Optional[List[float]] = None

@dataclass
class TrajectoryGoal:
    goal_id: str
    joint_names: List[str]
    points: List[JointTrajectoryPoint]

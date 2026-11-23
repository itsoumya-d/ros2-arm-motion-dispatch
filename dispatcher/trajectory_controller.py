from typing import List, Optional, Tuple, Dict
from .models import TrajectoryGoal, TrajectoryStatus, KinematicLimits, JointTrajectoryPoint

class TrajectoryController:
    def __init__(self, limits: Dict[str, KinematicLimits]):
        self.limits = limits
        self.status = TrajectoryStatus.IDLE
        self.current_goal: Optional[TrajectoryGoal] = None
        self.current_time: float = 0.0
        self.current_positions: List[float] = []

    def validate_goal(self, goal: TrajectoryGoal) -> Tuple[bool, Optional[str]]:
        if not goal.points:
            return False, "Trajectory contains no waypoints"
        
        num_joints = len(goal.joint_names)
        for name in goal.joint_names:
            if name not in self.limits:
                return False, f"Unknown joint {name} not in configured limits"

        last_time = -1e-6
        for idx, pt in enumerate(goal.points):
            if pt.time_from_start <= last_time:
                return False, f"Non-monotonic timestamp at waypoint {idx}"
            last_time = pt.time_from_start

            if len(pt.positions) != num_joints:
                return False, f"Position dimension mismatch at waypoint {idx}"

            for j_idx, j_name in enumerate(goal.joint_names):
                pos = pt.positions[j_idx]
                lim = self.limits[j_name]
                if not (lim.min_pos <= pos <= lim.max_pos):
                    return False, f"Position limit exceeded for {j_name} at waypoint {idx}: {pos}"

                if pt.velocities and len(pt.velocities) == num_joints:
                    vel = abs(pt.velocities[j_idx])
                    if vel > lim.max_vel:
                        return False, f"Velocity limit exceeded for {j_name} at waypoint {idx}: {vel}"

                if pt.accelerations and len(pt.accelerations) == num_joints:
                    acc = abs(pt.accelerations[j_idx])
                    if acc > lim.max_acc:
                        return False, f"Acceleration limit exceeded for {j_name} at waypoint {idx}: {acc}"

        return True, None

    def dispatch_goal(self, goal: TrajectoryGoal) -> Tuple[bool, Optional[str]]:
        if self.status == TrajectoryStatus.RUNNING:
            return False, "Cannot dispatch goal while another trajectory is executing"

        ok, err = self.validate_goal(goal)
        if not ok:
            self.status = TrajectoryStatus.ABORTED
            return False, err

        self.current_goal = goal
        self.status = TrajectoryStatus.RUNNING
        self.current_time = 0.0
        self.current_positions = list(goal.points[0].positions)
        return True, None

    def step(self, dt: float) -> Tuple[TrajectoryStatus, float, List[float]]:
        if self.status != TrajectoryStatus.RUNNING or not self.current_goal:
            return self.status, 0.0, self.current_positions

        self.current_time += dt
        total_time = self.current_goal.points[-1].time_from_start

        if self.current_time >= total_time:
            self.current_positions = list(self.current_goal.points[-1].positions)
            self.status = TrajectoryStatus.COMPLETED
            return self.status, 1.0, self.current_positions

        # Interpolate between surrounding waypoints
        pts = self.current_goal.points
        for i in range(len(pts) - 1):
            t0, t1 = pts[i].time_from_start, pts[i + 1].time_from_start
            if t0 <= self.current_time <= t1:
                ratio = (self.current_time - t0) / (t1 - t0) if t1 > t0 else 1.0
                interp = [
                    p0 + ratio * (p1 - p0)
                    for p0, p1 in zip(pts[i].positions, pts[i + 1].positions)
                ]
                self.current_positions = interp
                progress = self.current_time / total_time if total_time > 0 else 1.0
                return self.status, progress, self.current_positions

        return self.status, 1.0, self.current_positions

    def cancel_goal(self) -> bool:
        if self.status == TrajectoryStatus.RUNNING:
            self.status = TrajectoryStatus.CANCELLED
            return True
        return False

    def emergency_stop(self) -> None:
        self.status = TrajectoryStatus.ABORTED

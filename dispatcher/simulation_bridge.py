import math
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
from dispatcher.models import KinematicLimits, JointTrajectoryPoint, TrajectoryGoal, TrajectoryStatus
from dispatcher.trajectory_controller import TrajectoryController

@dataclass
class ObstacleZone:
    name: str
    min_bounds: List[float]  # [x_min, y_min, z_min]
    max_bounds: List[float]  # [x_max, y_max, z_max]

    def contains_point(self, point: List[float]) -> bool:
        if len(point) < 3:
            return False
        return all(self.min_bounds[i] <= point[i] <= self.max_bounds[i] for i in range(3))

@dataclass
class SimulatedHumanoidState:
    joint_positions: Dict[str, float]
    joint_velocities: Dict[str, float]
    simulation_time: float = 0.0
    contacts: List[str] = field(default_factory=list)

class AgenticSimulationBridge:
    """Bridges high-level agentic motion commands with ROS 2 trajectory execution

    and physics-based simulation (Isaac Sim / MuJoCo).
    """

    def __init__(
        self,
        controller: TrajectoryController,
        obstacles: Optional[List[ObstacleZone]] = None,
        link_lengths: Optional[List[float]] = None,
    ):
        self.controller = controller
        self.obstacles = obstacles or []
        self.link_lengths = link_lengths or [0.5, 0.5]  # 2-link planar arm approximation for testing
        self.sim_state = SimulatedHumanoidState(
            joint_positions={j: 0.0 for j in controller.limits},
            joint_velocities={j: 0.0 for j in controller.limits},
            simulation_time=0.0
        )

    def forward_kinematics_2d(self, theta1: float, theta2: float) -> Tuple[float, float, float]:
        """Compute end-effector 3D coordinate (z=0) for 2-link planar arm."""
        l1, l2 = self.link_lengths[0], self.link_lengths[1]
        x = l1 * math.cos(theta1) + l2 * math.cos(theta1 + theta2)
        y = l1 * math.sin(theta1) + l2 * math.sin(theta1 + theta2)
        return (x, y, 0.0)

    def is_trajectory_safe(self, goal: TrajectoryGoal) -> Tuple[bool, Optional[str]]:
        """Validate that planned trajectory waypoints do not breach collision obstacles."""
        for pt in goal.points:
            if len(pt.positions) >= 2:
                ee_pos = list(self.forward_kinematics_2d(pt.positions[0], pt.positions[1]))
                for obs in self.obstacles:
                    if obs.contains_point(ee_pos):
                        return False, f"Collision detected with obstacle '{obs.name}' at point {ee_pos}"
        return True, None

    def plan_agentic_task(
        self,
        task_id: str,
        target_joint_positions: Dict[str, float],
        duration: float = 2.0,
        num_waypoints: int = 5,
    ) -> Tuple[bool, Optional[TrajectoryGoal], Optional[str]]:
        """Synthesize a smooth multi-waypoint trajectory from agentic task parameters."""
        if duration <= 0:
            return False, None, "Task duration must be positive"
        if num_waypoints < 2:
            return False, None, "At least 2 waypoints required"

        joint_names = list(self.controller.limits.keys())
        current_positions = [self.sim_state.joint_positions.get(j, 0.0) for j in joint_names]
        target_positions = [target_joint_positions.get(j, current_positions[idx]) for idx, j in enumerate(joint_names)]

        points: List[JointTrajectoryPoint] = []
        for i in range(num_waypoints):
            s = i / (num_waypoints - 1)
            # Quintic smoothstep s_smooth = 10*s^3 - 15*s^4 + 6*s^5 for zero boundary vel/acc
            s_smooth = 10 * (s ** 3) - 15 * (s ** 4) + 6 * (s ** 5)
            interpolated_pos = [
                current_positions[j] + s_smooth * (target_positions[j] - current_positions[j])
                for j in range(len(joint_names))
            ]
            t = s * duration
            points.append(JointTrajectoryPoint(time_from_start=t, positions=interpolated_pos))

        goal = TrajectoryGoal(goal_id=task_id, joint_names=joint_names, points=points)

        safe, err = self.is_trajectory_safe(goal)
        if not safe:
            return False, None, err

        return True, goal, None

    def execute_task(self, goal: TrajectoryGoal) -> Tuple[bool, Optional[str]]:
        """Dispatch goal to controller and sync simulation state."""
        return self.controller.dispatch_goal(goal)

    def step_simulation(self, dt: float) -> SimulatedHumanoidState:
        """Step physics simulation forward by dt and update joint telemetry."""
        status, progress, positions = self.controller.step(dt)
        self.sim_state.simulation_time += dt

        if positions and len(positions) == len(self.controller.limits):
            joint_names = list(self.controller.limits.keys())
            for idx, j in enumerate(joint_names):
                prev_pos = self.sim_state.joint_positions.get(j, 0.0)
                new_pos = positions[idx]
                vel = (new_pos - prev_pos) / dt if dt > 0 else 0.0
                self.sim_state.joint_positions[j] = new_pos
                self.sim_state.joint_velocities[j] = vel

        return self.sim_state

import pytest
from dispatcher.models import KinematicLimits, TrajectoryStatus
from dispatcher.trajectory_controller import TrajectoryController
from dispatcher.simulation_bridge import ObstacleZone, AgenticSimulationBridge

@pytest.fixture
def simulation_setup():
    limits = {
        "joint_1": KinematicLimits(min_pos=-3.14, max_pos=3.14, max_vel=2.0, max_acc=4.0),
        "joint_2": KinematicLimits(min_pos=-1.57, max_pos=1.57, max_vel=1.5, max_acc=3.0),
    }
    controller = TrajectoryController(limits)
    # Define an obstacle zone in the upper work quadrant: x in [0.4, 0.9], y in [0.4, 0.8]
    obstacle = ObstacleZone(
        name="fixture_barrier",
        min_bounds=[0.4, 0.4, -0.1],
        max_bounds=[0.9, 0.8, 0.1],
    )
    bridge = AgenticSimulationBridge(
        controller=controller,
        obstacles=[obstacle],
        link_lengths=[0.5, 0.5]
    )
    return bridge

def test_smooth_agentic_task_planning_success(simulation_setup):
    bridge = simulation_setup
    # Target joint angles that move into the lower quadrant (negative y), clear of obstacle
    targets = {"joint_1": -0.6, "joint_2": 0.2}
    ok, goal, err = bridge.plan_agentic_task(
        task_id="task-manipulation-01",
        target_joint_positions=targets,
        duration=2.0,
        num_waypoints=5
    )

    assert ok is True
    assert goal is not None
    assert err is None
    assert len(goal.points) == 5
    assert goal.points[0].positions[0] == 0.0
    assert pytest.approx(goal.points[-1].positions[0], rel=1e-3) == -0.6

def test_collision_detection_rejects_hazardous_trajectory(simulation_setup):
    bridge = simulation_setup
    # Joint angles that sweep through upper quadrant (0.8, 0.4) will cross fixture_barrier
    targets = {"joint_1": 0.8, "joint_2": 0.4}
    ok, goal, err = bridge.plan_agentic_task(
        task_id="task-collision-test",
        target_joint_positions=targets,
        duration=2.0,
        num_waypoints=5
    )

    assert ok is False
    assert goal is None
    assert "Collision detected with obstacle 'fixture_barrier'" in err

def test_simulation_step_updates_joint_telemetry(simulation_setup):
    bridge = simulation_setup
    targets = {"joint_1": -0.5, "joint_2": 0.3}
    ok, goal, _ = bridge.plan_agentic_task(
        task_id="task-telemetry",
        target_joint_positions=targets,
        duration=2.0,
        num_waypoints=5
    )
    assert ok is True

    exec_ok, _ = bridge.execute_task(goal)
    assert exec_ok is True

    # Step simulation forward
    state = bridge.step_simulation(dt=1.0)
    assert state.simulation_time == 1.0
    assert state.joint_positions["joint_1"] < 0.0
    assert state.joint_velocities["joint_1"] < 0.0

    # Step to completion
    state = bridge.step_simulation(dt=1.0)
    assert state.simulation_time == 2.0
    assert bridge.controller.status == TrajectoryStatus.COMPLETED

def test_invalid_task_parameters_rejected(simulation_setup):
    bridge = simulation_setup
    ok, _, err = bridge.plan_agentic_task("task-bad-dur", {"joint_1": 0.5}, duration=-1.0)
    assert ok is False
    assert "duration must be positive" in err

    ok, _, err = bridge.plan_agentic_task("task-bad-wp", {"joint_1": 0.5}, duration=1.0, num_waypoints=1)
    assert ok is False
    assert "At least 2 waypoints required" in err

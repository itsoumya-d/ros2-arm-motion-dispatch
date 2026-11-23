import pytest
from dispatcher.models import KinematicLimits, JointTrajectoryPoint, TrajectoryGoal, TrajectoryStatus
from dispatcher.trajectory_controller import TrajectoryController

@pytest.fixture
def arm_controller():
    limits = {
        "joint_1": KinematicLimits(min_pos=-3.14, max_pos=3.14, max_vel=2.0, max_acc=4.0),
        "joint_2": KinematicLimits(min_pos=-1.57, max_pos=1.57, max_vel=1.5, max_acc=3.0),
    }
    return TrajectoryController(limits)

def test_successful_trajectory_dispatch_and_execution(arm_controller):
    goal = TrajectoryGoal(
        goal_id="traj-001",
        joint_names=["joint_1", "joint_2"],
        points=[
            JointTrajectoryPoint(time_from_start=0.0, positions=[0.0, 0.0], velocities=[0.0, 0.0]),
            JointTrajectoryPoint(time_from_start=1.0, positions=[0.5, 0.3], velocities=[0.5, 0.3]),
            JointTrajectoryPoint(time_from_start=2.0, positions=[1.0, 0.6], velocities=[0.0, 0.0]),
        ]
    )

    ok, err = arm_controller.dispatch_goal(goal)
    assert ok is True
    assert err is None
    assert arm_controller.status == TrajectoryStatus.RUNNING

    status, progress, positions = arm_controller.step(1.0)
    assert status == TrajectoryStatus.RUNNING
    assert pytest.approx(progress, rel=1e-2) == 0.5
    assert pytest.approx(positions[0], rel=1e-2) == 0.5

    status, progress, positions = arm_controller.step(1.0)
    assert status == TrajectoryStatus.COMPLETED
    assert progress == 1.0
    assert pytest.approx(positions[0], rel=1e-2) == 1.0

def test_validation_fails_on_out_of_bounds_joint_position(arm_controller):
    goal = TrajectoryGoal(
        goal_id="traj-fail-pos",
        joint_names=["joint_1", "joint_2"],
        points=[
            JointTrajectoryPoint(time_from_start=0.0, positions=[0.0, 0.0]),
            JointTrajectoryPoint(time_from_start=1.0, positions=[4.5, 0.0]),  # Exceeds max_pos 3.14
        ]
    )

    ok, err = arm_controller.dispatch_goal(goal)
    assert ok is False
    assert "Position limit exceeded for joint_1" in err
    assert arm_controller.status == TrajectoryStatus.ABORTED

def test_validation_fails_on_velocity_breach(arm_controller):
    goal = TrajectoryGoal(
        goal_id="traj-fail-vel",
        joint_names=["joint_1", "joint_2"],
        points=[
            JointTrajectoryPoint(time_from_start=0.0, positions=[0.0, 0.0], velocities=[0.0, 0.0]),
            JointTrajectoryPoint(time_from_start=1.0, positions=[0.5, 0.2], velocities=[3.5, 0.1]),  # max_vel is 2.0
        ]
    )

    ok, err = arm_controller.dispatch_goal(goal)
    assert ok is False
    assert "Velocity limit exceeded for joint_1" in err

def test_non_monotonic_timestamp_rejected(arm_controller):
    goal = TrajectoryGoal(
        goal_id="traj-non-mono",
        joint_names=["joint_1", "joint_2"],
        points=[
            JointTrajectoryPoint(time_from_start=1.0, positions=[0.0, 0.0]),
            JointTrajectoryPoint(time_from_start=0.5, positions=[0.5, 0.2]),
        ]
    )

    ok, err = arm_controller.dispatch_goal(goal)
    assert ok is False
    assert "Non-monotonic timestamp" in err

def test_cancellation_mid_execution(arm_controller):
    goal = TrajectoryGoal(
        goal_id="traj-cancel",
        joint_names=["joint_1", "joint_2"],
        points=[
            JointTrajectoryPoint(time_from_start=0.0, positions=[0.0, 0.0]),
            JointTrajectoryPoint(time_from_start=2.0, positions=[1.0, 0.5]),
        ]
    )

    arm_controller.dispatch_goal(goal)
    arm_controller.step(0.5)
    assert arm_controller.status == TrajectoryStatus.RUNNING

    cancelled = arm_controller.cancel_goal()
    assert cancelled is True
    assert arm_controller.status == TrajectoryStatus.CANCELLED

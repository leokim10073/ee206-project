"""A /sim/reset that fails halfway gives the arm back to the controllers it took it from."""

from types import SimpleNamespace

from std_srvs.srv import Trigger

from maze_sim_bringup.sim_tools_node import SimTools


def fake_sim_tools(reset_world_ok=True, ball_ok=True):
    """SimTools without ROS: service calls answered by fakes, controller switches recorded."""
    tools = SimTools.__new__(SimTools)
    tools.list_cli, tools.reset_cli, tools.ball_cli = "list", "reset", "ball"
    tools.switches = []
    answers = {
        "list": SimpleNamespace(controller=[
            SimpleNamespace(name="scaled_joint_trajectory_controller", state="active"),
            SimpleNamespace(name="forward_velocity_controller", state="inactive"),
        ]),
        "reset": SimpleNamespace(success=reset_world_ok, message="no keyframe"),
        "ball": SimpleNamespace(success=ball_ok, message="no ball"),
    }
    tools._call = lambda client, _request: answers[client]
    tools._switch = lambda activate, deactivate: tools.switches.append((list(activate), list(deactivate)))
    tools._sleep_sim = lambda _s: None
    tools._command_initial_joints = lambda _active: None
    tools._board_pose = lambda: SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0, z=0.0), orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0)))
    tools.cfg = SimpleNamespace(ball=SimpleNamespace(start=(0.0, 0.0), radius=0.007))
    tools.gripper_pub = tools.reset_done = SimpleNamespace(publish=lambda _m: None)
    return tools


def test_failed_reset_world_reactivates_the_arm():
    tools = fake_sim_tools(reset_world_ok=False)
    response = tools._reset(Trigger.Response())
    assert not response.success and "reset_world failed" in response.message
    assert tools.switches == [([], ["scaled_joint_trajectory_controller"]), (["scaled_joint_trajectory_controller"], [])]


def test_failure_after_reactivation_switches_once():
    tools = fake_sim_tools(ball_ok=False)
    response = tools._reset(Trigger.Response())
    assert not response.success and "set_free_joint_state failed" in response.message
    assert tools.switches == [([], ["scaled_joint_trajectory_controller"]), (["scaled_joint_trajectory_controller"], [])]


def test_successful_reset():
    tools = fake_sim_tools()
    assert tools._reset(Trigger.Response()).success
    assert tools.switches == [([], ["scaled_joint_trajectory_controller"]), (["scaled_joint_trajectory_controller"], [])]

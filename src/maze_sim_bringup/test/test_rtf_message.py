"""The real-time-factor log tells a teammate on a slow machine what it means for their code."""

from maze_sim_bringup.sim_tools_node import rtf_message


def test_real_time_is_info():
    level, text = rtf_message(1.0, target=1.0)
    assert level == "info" and text == "real-time factor: 1.00"


def test_slow_machine_warns_about_wall_clock_nodes():
    level, text = rtf_message(0.6, target=1.0)
    assert level == "warn"
    assert text.startswith("real-time factor: 0.60")
    assert "wall clock" in text  # lab code on the wall clock moves the arm less per second than on the robot

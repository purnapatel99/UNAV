"""
Launch file: SITL test pipeline

Starts:
  1. Micro-XRCE-DDS-Agent  (bridges PX4 uORB ↔ ROS2 DDS)
  2. MPC controller         (C++, 50 Hz, auto-arms + goes offboard)
  3. Orchestrator           (Python, mission brain)

Run test mission separately after everything is up:
  ros2 run unav_test test_mission

Prerequisites (run manually in separate terminals):
  Terminal 1: ~/Workspace/run_sitl_cota.sh   # spawns at COTA (30.1316°N, 97.6386°W)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


XRCE_AGENT = os.path.expanduser(
    "~/Workspace/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent"
)

# Central motion limits — mirrors PX4 MPC_XY_VEL_MAX, MPC_Z_VEL_MAX_UP/DN.
# Update this file when you change PX4 params.
FLIGHT_LIMITS_YAML = os.path.join(
    get_package_share_directory("unav_mpc"),
    "config",
    "flight_limits.yaml",
)


def generate_launch_description():
    xrce_agent = ExecuteProcess(
        cmd=[XRCE_AGENT, "udp4", "-p", "8888"],
        name="xrce_agent",
        output="screen",
    )

    # Give agent 2 s to start before launching ROS2 nodes
    mpc_controller = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="unav_mpc",
                executable="mpc_controller",
                name="mpc_controller",
                output="screen",
                parameters=[
                    FLIGHT_LIMITS_YAML,
                    {
                        "control_hz":   50.0,
                        "mpc_horizon":  50,
                        # Tracking weights — moderate so PX4 feedback doesn't double-correct
                        "q_pos":         5.0,
                        "q_vel":         2.0,
                        "qf_pos":        5.0,
                        "qf_vel":        2.0,
                        # Input cost — higher r_acc softens the initial acceleration snap
                        "r_acc":         0.5,
                        # Jerk cost — higher rd_jerk smooths the step from u_prev=0 at start
                        "rd_jerk":       8.0,
                    },
                ],
            )
        ],
    )

    orchestrator = TimerAction(
        period=2.5,
        actions=[
            Node(
                package="unav_planner",
                executable="orchestrator",
                name="orchestrator",
                output="screen",
                parameters=[FLIGHT_LIMITS_YAML],
            )
        ],
    )

    return LaunchDescription([
        xrce_agent,
        mpc_controller,
        orchestrator,
    ])

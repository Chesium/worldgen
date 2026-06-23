#!/usr/bin/env python3
"""Launch a single Nav2 benchmark trial: Gazebo + TB3 + Nav2 + benchmark runner.

Brings up Nav2's stock TB3 simulation (Gazebo world, robot spawn, bridge, Nav2
stack) for a generated world/map with a given parameter profile, then runs
``benchmark_runner.py`` to navigate from the world's fixed start to its fixed
goal. When the runner exits, the whole launch is shut down so the orchestrator
can move on to the next trial.

Required launch arguments:
    world         augmented world SDF (see augment_world.py)
    map           occupancy map yaml
    params_file   Nav2 parameter file (see make_profiles.py)
    nav_task      nav_task.json with start/goal poses
    result_out    where the runner writes the trial result JSON

Optional: timeout, wall_timeout, collision_threshold, headless, use_rviz,
world_id, profile, planner, controller, inflation.
"""
from __future__ import annotations

import json
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_RUNNER = os.path.join(_THIS_DIR, "..", "scripts", "benchmark_runner.py")


def _declare() -> list[DeclareLaunchArgument]:
    specs = {
        "world": None,
        "map": None,
        "params_file": None,
        "nav_task": None,
        "result_out": "/tmp/trial_result.json",
        "timeout": "120.0",
        "wall_timeout": "400.0",
        "collision_threshold": "0.16",
        "goal_tolerance": "0.35",
        "headless": "True",
        "use_rviz": "False",
        "use_composition": "False",
        "world_id": "",
        "profile": "",
        "planner": "",
        "controller": "",
        "inflation": "",
    }
    declarations = []
    for name, default in specs.items():
        kwargs = {} if default is None else {"default_value": default}
        declarations.append(DeclareLaunchArgument(name, **kwargs))
    return declarations


def _setup(context, *args, **kwargs):
    def cfg(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    nav_task_path = cfg("nav_task")
    with open(nav_task_path, encoding="utf-8") as handle:
        task = json.load(handle)
    start = task["start"]

    bringup_dir = get_package_share_directory("nav2_bringup")
    tb3_sim_launch = os.path.join(bringup_dir, "launch", "tb3_simulation_launch.py")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_sim_launch),
        launch_arguments={
            "world": cfg("world"),
            "map": cfg("map"),
            "params_file": cfg("params_file"),
            "headless": cfg("headless"),
            "use_rviz": cfg("use_rviz"),
            "use_composition": cfg("use_composition"),
            "use_sim_time": "True",
            "autostart": "True",
            "x_pose": str(start["x"]),
            "y_pose": str(start["y"]),
            "yaw": str(start.get("yaw", 0.0)),
        }.items(),
    )

    runner = ExecuteProcess(
        cmd=[
            sys.executable,
            os.path.abspath(_RUNNER),
            "--nav-task", nav_task_path,
            "--output", cfg("result_out"),
            "--timeout", cfg("timeout"),
            "--wall-timeout", cfg("wall_timeout"),
            "--collision-threshold", cfg("collision_threshold"),
            "--goal-tolerance", cfg("goal_tolerance"),
            "--world-id", cfg("world_id"),
            "--profile", cfg("profile"),
            "--planner", cfg("planner"),
            "--controller", cfg("controller"),
            "--inflation", cfg("inflation"),
        ],
        output="screen",
        name="benchmark_runner",
    )

    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(target_action=runner, on_exit=[EmitEvent(event=Shutdown())])
    )

    return [simulation, runner, shutdown_on_exit]


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    for declaration in _declare():
        ld.add_action(declaration)
    ld.add_action(OpaqueFunction(function=_setup))
    return ld

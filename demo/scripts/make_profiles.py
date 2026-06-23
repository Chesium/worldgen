#!/usr/bin/env python3
"""Generate Nav2 parameter files for a benchmark sweep.

Each *profile* is a combination of (global planner) x (local controller) x
(costmap inflation radius). Profiles are produced by loading Nav2's stock
``nav2_params.yaml`` and swapping the ``planner_server.GridBased`` plugin block,
the ``controller_server.FollowPath`` plugin block, and the ``inflation_radius``
on both the local and global costmaps.

The script is stdlib + PyYAML only so it runs in the uv project venv.

Usage:
    python make_profiles.py --out demo/nav2_profiles --set curated
    python make_profiles.py --out demo/nav2_profiles --set full
"""
from __future__ import annotations

import argparse
import copy
import json
import os
from pathlib import Path

import yaml

# ---------------------------------------------------------------------------
# Planner (planner_server.GridBased) blocks
# ---------------------------------------------------------------------------
PLANNERS: dict[str, dict] = {
    "navfn": {
        "plugin": "nav2_navfn_planner::NavfnPlanner",
        "tolerance": 0.5,
        "use_astar": False,
        "allow_unknown": True,
    },
    "smac2d": {
        "plugin": "nav2_smac_planner::SmacPlanner2D",
        "tolerance": 0.125,
        "downsample_costmap": False,
        "downsampling_factor": 1,
        "allow_unknown": True,
        "max_iterations": 1000000,
        "max_on_approach_iterations": 1000,
        "max_planning_time": 2.0,
        "cost_travel_multiplier": 2.0,
        "use_final_approach_orientation": False,
        "smoother": {
            "max_iterations": 1000,
            "w_smooth": 0.3,
            "w_data": 0.2,
            "tolerance": 1.0e-10,
        },
    },
    "theta": {
        "plugin": "nav2_theta_star_planner::ThetaStarPlanner",
        "how_many_corners": 8,
        "w_euc_cost": 1.0,
        "w_traversal_cost": 2.0,
        "allow_unknown": True,
    },
}

# ---------------------------------------------------------------------------
# Controller (controller_server.FollowPath) blocks
# ---------------------------------------------------------------------------
_MPPI = {
    "plugin": "nav2_mppi_controller::MPPIController",
    "time_steps": 56,
    "model_dt": 0.05,
    "batch_size": 1000,
    "ax_max": 3.0,
    "ax_min": -3.0,
    "ay_max": 3.0,
    "ay_min": -3.0,
    "az_max": 3.5,
    "vx_std": 0.2,
    "vy_std": 0.2,
    "wz_std": 0.4,
    "vx_max": 0.26,
    "vx_min": -0.26,
    "vy_max": 0.5,
    "wz_max": 1.0,
    "iteration_count": 1,
    "prune_distance": 1.7,
    "transform_tolerance": 0.1,
    "temperature": 0.3,
    "gamma": 0.015,
    "motion_model": "DiffDrive",
    "visualize": False,
    "regenerate_noises": True,
    "AckermannConstraints": {"min_turning_r": 0.2},
    "critics": [
        "ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic",
        "PathAlignCritic", "PathFollowCritic", "PathAngleCritic",
        "PreferForwardCritic",
    ],
    "ConstraintCritic": {"enabled": True, "cost_power": 1, "cost_weight": 4.0},
    "GoalCritic": {
        "enabled": True, "cost_power": 1, "cost_weight": 5.0,
        "threshold_to_consider": 1.4,
    },
    "GoalAngleCritic": {
        "enabled": True, "cost_power": 1, "cost_weight": 3.0,
        "threshold_to_consider": 0.5,
    },
    "PreferForwardCritic": {
        "enabled": True, "cost_power": 1, "cost_weight": 5.0,
        "threshold_to_consider": 0.5,
    },
    "CostCritic": {
        "enabled": True, "cost_power": 1, "cost_weight": 3.81,
        "near_collision_cost": 253, "critical_cost": 300.0,
        "consider_footprint": False, "collision_cost": 1000000.0,
        "near_goal_distance": 1.0, "trajectory_point_step": 2,
    },
    "PathAlignCritic": {
        "enabled": True, "cost_power": 1, "cost_weight": 14.0,
        "max_path_occupancy_ratio": 0.05, "trajectory_point_step": 4,
        "threshold_to_consider": 0.5, "offset_from_furthest": 20,
        "use_path_orientations": False,
    },
    "PathFollowCritic": {
        "enabled": True, "cost_power": 1, "cost_weight": 5.0,
        "offset_from_furthest": 5, "threshold_to_consider": 1.4,
    },
    "PathAngleCritic": {
        "enabled": True, "cost_power": 1, "cost_weight": 2.0,
        "offset_from_furthest": 4, "threshold_to_consider": 0.5,
        "max_angle_to_furthest": 1.0, "mode": 0,
    },
}

_DWB = {
    "plugin": "dwb_core::DWBLocalPlanner",
    "debug_trajectory_details": True,
    "min_vel_x": 0.0,
    "min_vel_y": 0.0,
    "max_vel_x": 0.26,
    "max_vel_y": 0.0,
    "max_vel_theta": 1.0,
    "min_speed_xy": 0.0,
    "max_speed_xy": 0.26,
    "min_speed_theta": 0.0,
    "acc_lim_x": 2.5,
    "acc_lim_y": 0.0,
    "acc_lim_theta": 3.2,
    "decel_lim_x": -2.5,
    "decel_lim_y": 0.0,
    "decel_lim_theta": -3.2,
    "vx_samples": 20,
    "vy_samples": 5,
    "vtheta_samples": 20,
    "sim_time": 1.7,
    "linear_granularity": 0.05,
    "angular_granularity": 0.025,
    "transform_tolerance": 0.2,
    "xy_goal_tolerance": 0.25,
    "trans_stopped_velocity": 0.25,
    "short_circuit_trajectory_evaluation": True,
    "stateful": True,
    "critics": [
        "RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign",
        "PathAlign", "PathDist", "GoalDist",
    ],
    "BaseObstacle.scale": 0.02,
    "PathAlign.scale": 32.0,
    "PathAlign.forward_point_distance": 0.1,
    "GoalAlign.scale": 24.0,
    "GoalAlign.forward_point_distance": 0.1,
    "PathDist.scale": 32.0,
    "GoalDist.scale": 24.0,
    "RotateToGoal.scale": 32.0,
    "RotateToGoal.slowing_factor": 5.0,
    "RotateToGoal.lookahead_time": -1.0,
}

_RPP = {
    "plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    "desired_linear_vel": 0.26,
    "lookahead_dist": 0.6,
    "min_lookahead_dist": 0.3,
    "max_lookahead_dist": 0.9,
    "lookahead_time": 1.5,
    "rotate_to_heading_angular_vel": 1.0,
    "transform_tolerance": 0.5,
    "use_velocity_scaled_lookahead_dist": False,
    "min_approach_linear_velocity": 0.05,
    "approach_velocity_scaling_dist": 0.6,
    "use_collision_detection": True,
    "max_allowed_time_to_collision_up_to_carrot": 1.0,
    "use_regulated_linear_velocity_scaling": True,
    "use_fixed_curvature_lookahead": False,
    "curvature_lookahead_dist": 0.25,
    "use_cost_regulated_linear_velocity_scaling": False,
    "regulated_linear_scaling_min_radius": 0.9,
    "regulated_linear_scaling_min_speed": 0.25,
    "use_rotate_to_heading": True,
    "allow_reversing": False,
    "rotate_to_heading_min_angle": 0.785,
    "max_angular_accel": 3.2,
    "max_robot_pose_search_dist": 10.0,
}

CONTROLLERS: dict[str, dict] = {"mppi": _MPPI, "dwb": _DWB, "rpp": _RPP}

# ---------------------------------------------------------------------------
# Profile sets
# ---------------------------------------------------------------------------
# The first entry of "curated" is treated as the hard CI gate baseline.
CURATED: list[dict] = [
    {"name": "navfn_rpp_baseline", "planner": "navfn", "controller": "rpp", "inflation": 0.30},
    {"name": "navfn_dwb_tight", "planner": "navfn", "controller": "dwb", "inflation": 0.25},
    {"name": "smac2d_rpp_loose", "planner": "smac2d", "controller": "rpp", "inflation": 0.40},
]


def build_profiles(set_name: str) -> list[dict]:
    if set_name == "curated":
        return [dict(p) for p in CURATED]
    if set_name == "full":
        profiles: list[dict] = []
        for planner in PLANNERS:
            for controller in CONTROLLERS:
                for inflation in (0.25, 0.45):
                    inf_tag = str(inflation).replace(".", "")
                    profiles.append(
                        {
                            "name": f"{planner}_{controller}_inf{inf_tag}",
                            "planner": planner,
                            "controller": controller,
                            "inflation": inflation,
                        }
                    )
        return profiles
    raise ValueError(f"Unknown profile set: {set_name!r} (use 'curated' or 'full')")


def default_base_params() -> Path:
    """Locate Nav2's stock nav2_params.yaml."""
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("nav2_bringup"))
        return share / "params" / "nav2_params.yaml"
    except Exception:
        distro = os.environ.get("ROS_DISTRO", "jazzy")
        return Path(f"/opt/ros/{distro}/share/nav2_bringup/params/nav2_params.yaml")


def render_profile(base: dict, spec: dict) -> dict:
    params = copy.deepcopy(base)

    planner_block = copy.deepcopy(PLANNERS[spec["planner"]])
    params["planner_server"]["ros__parameters"]["GridBased"] = planner_block

    controller_block = copy.deepcopy(CONTROLLERS[spec["controller"]])
    params["controller_server"]["ros__parameters"]["FollowPath"] = controller_block

    inflation = float(spec["inflation"])
    local = params["local_costmap"]["local_costmap"]["ros__parameters"]
    local["inflation_layer"]["inflation_radius"] = inflation
    glob = params["global_costmap"]["global_costmap"]["ros__parameters"]
    # Decouple global vs local inflation. The generated worlds contain ~0.7 m
    # junction pinch-points alongside genuinely wide (>=1.0 m) routes between the
    # same rooms. With a uniform small inflation the global planner takes the
    # *shortest* path, which threads the pinch, leaving the footprint <0.15 m of
    # margin -- the robot localizes onto a lethal cell ("Start occupied") and the
    # plan thrashes. Inflate the GLOBAL costmap aggressively (large radius + steep
    # cost falloff) so near-wall cells are expensive and NavFn prefers the wide
    # channel, while the LOCAL costmap keeps the profile's smaller inflation so the
    # controller still has room to actually drive that channel.
    global_inflation = max(inflation, 0.55)
    glob["inflation_layer"]["inflation_radius"] = global_inflation
    glob["inflation_layer"]["cost_scaling_factor"] = 5.0
    # Enlarge the GLOBAL planning footprint so the planner treats cells within
    # ~0.4 m of any obstacle as lethal. Cost-shaping alone cannot reroute NavFn off
    # a pinch when the pinch route is barely longer than straight-line (here 15.06 m
    # planned vs 14.55 m straight); making the ~0.7 m pinches *impassable* in the
    # global costmap forces NavFn onto the guaranteed >=1.0 m channel (the start/goal
    # picker erodes by 10 cells = 0.5 m, so such a channel always exists). The local
    # costmap keeps the true 0.22 m radius so the controller can still drive it.
    glob["robot_radius"] = 0.40

    # Enlarge the rolling local costmap window and loosen transform tolerances.
    # The default 3 m window plus clock/TF jitter lets the controller's pruned
    # path fall out of the costmap ("Resulting plan has 0 poses in it"); a 7 m
    # window with generous transform tolerance keeps the path in view.
    local["width"] = 7
    local["height"] = 7
    local["transform_tolerance"] = 0.5
    glob["transform_tolerance"] = 0.5

    # Relax the goal checker for a position-based navigation benchmark: reaching
    # the goal *location* is what matters, not the final heading. The stock yaw
    # tolerance (0.25 rad) makes the robot fail the final in-place rotation near
    # walls even after it has arrived, so we widen the position tolerance slightly
    # and effectively ignore the goal yaw.
    ctrl = params["controller_server"]["ros__parameters"]
    goal_checker = ctrl.setdefault("general_goal_checker", {})
    goal_checker["xy_goal_tolerance"] = 0.35
    goal_checker["yaw_goal_tolerance"] = 3.15

    # The benchmark worlds are fully known static maps. Plan the *global* path on
    # the static map + inflation only, dropping the live lidar obstacle layer.
    # On a slow, software-rendered sim, transient lidar marks plus AMCL/TF jitter
    # otherwise paint spurious lethal cells that seal wide-open gates and make the
    # planner report "Failed to create plan". The local costmap keeps its lidar
    # layers so the controller still reacts to real obstacles for collision safety.
    glob["plugins"] = ["static_layer", "inflation_layer"]
    glob.pop("obstacle_layer", None)
    glob.pop("voxel_layer", None)

    return params


def write_profiles(
    base_params_path: Path, out_dir: Path, set_name: str
) -> dict[str, Path]:
    with base_params_path.open(encoding="utf-8") as handle:
        base = yaml.safe_load(handle)

    out_dir.mkdir(parents=True, exist_ok=True)
    specs = build_profiles(set_name)
    manifest: list[dict] = []
    paths: dict[str, Path] = {}

    for index, spec in enumerate(specs):
        params = render_profile(base, spec)
        path = out_dir / f"{spec['name']}.yaml"
        with path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(params, handle, sort_keys=False, default_flow_style=False)
        paths[spec["name"]] = path
        entry = dict(spec)
        entry["params_file"] = str(path)
        entry["is_gate"] = index == 0 and set_name == "curated"
        manifest.append(entry)

    manifest_path = out_dir / "profiles.json"
    with manifest_path.open("w", encoding="utf-8") as handle:
        json.dump({"set": set_name, "profiles": manifest}, handle, indent=2)
        handle.write("\n")

    return paths


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True, help="Output directory.")
    parser.add_argument("--set", default="curated", choices=["curated", "full"])
    parser.add_argument(
        "--base-params",
        type=Path,
        default=None,
        help="Path to Nav2 nav2_params.yaml (auto-detected by default).",
    )
    args = parser.parse_args(argv)

    base = args.base_params or default_base_params()
    if not base.is_file():
        parser.error(f"Base params file not found: {base}")

    paths = write_profiles(base, args.out, args.set)
    print(f"Wrote {len(paths)} profiles to {args.out}:")
    for name, path in paths.items():
        print(f"  - {name}: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

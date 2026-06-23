#!/usr/bin/env python3
"""Run a single Nav2 navigation trial and record benchmark metrics.

This node drives one TurtleBot3 from a fixed start pose to a fixed goal pose
(read from the generator's ``nav_task.json``) using whatever Nav2 stack is
currently running, and writes a single result JSON describing the outcome.

It MUST be run with a sourced ROS 2 environment (it imports ``rclpy`` and
``nav2_simple_commander``), not the uv project venv.

Pass/fail gate (matches the demo spec): a trial PASSES iff the goal is reached
within tolerance (Nav2 goal checker) before the timeout AND the robot never came
within the collision threshold of an obstacle (scan-based proxy).

The process always exits 0 after writing the result JSON so the parent launch
file can shut down cleanly; the orchestrator decides pass/fail from the JSON.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from threading import Thread


def _euclidean(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nav-task", required=True, help="Path to nav_task.json")
    parser.add_argument("--output", required=True, help="Result JSON output path")
    parser.add_argument("--timeout", type=float, default=120.0,
                        help="Max navigation time (sim seconds) before failing.")
    parser.add_argument("--wall-timeout", type=float, default=400.0,
                        help="Max wall-clock seconds before aborting the trial.")
    parser.add_argument("--collision-threshold", type=float, default=0.16,
                        help="Min laser range (m) below which a collision is flagged.")
    parser.add_argument("--goal-tolerance", type=float, default=0.35,
                        help="Position tolerance (m) for benchmark success.")
    parser.add_argument("--world-id", default="")
    parser.add_argument("--profile", default="")
    parser.add_argument("--planner", default="")
    parser.add_argument("--controller", default="")
    parser.add_argument("--inflation", default="")
    args, _ = parser.parse_known_args(argv)

    with open(args.nav_task, encoding="utf-8") as handle:
        task = json.load(handle)

    result: dict = {
        "world_id": args.world_id,
        "profile": args.profile,
        "planner": args.planner,
        "controller": args.controller,
        "inflation": args.inflation,
        "result": "ERROR",
        "passed": False,
        "collision": False,
        "timed_out": False,
        "nav_time_s": None,
        "wall_time_s": None,
        "planned_path_length_m": None,
        "distance_traveled_m": None,
        "min_clearance_m": None,
        "min_distance_to_goal_m": None,
        "position_reached": False,
        "n_recoveries": None,
        "straight_line_distance_m": _euclidean(
            task["start"]["x"], task["start"]["y"],
            task["goal"]["x"], task["goal"]["y"],
        ),
        "error": None,
    }

    try:
        _run_trial(args, task, result)
    except Exception as exc:  # noqa: BLE001 - always record, never crash the launch
        result["error"] = f"{type(exc).__name__}: {exc}"

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2, sort_keys=True)
        handle.write("\n")

    print(f"[benchmark_runner] result={result['result']} passed={result['passed']} "
          f"-> {args.output}")
    return 0


def _run_trial(args, task: dict, result: dict) -> None:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

    class MetricsCollector(Node):
        def __init__(self) -> None:
            super().__init__("benchmark_metrics_collector")
            self.min_clearance = float("inf")
            self.distance_traveled = 0.0
            self._last_xy: tuple[float, float] | None = None
            self.scan_count = 0
            self.odom_count = 0
            self.tf_count = 0
            self.tf_edges: set[str] = set()
            self.tf_edge_stamps: dict[str, float] = {}
            self.tf_edge_first_stamps: dict[str, float] = {}
            self.create_subscription(LaserScan, "scan", self._on_scan, 10)
            self.create_subscription(Odometry, "odom", self._on_odom, 10)
            self.create_subscription(TFMessage, "tf", self._on_tf, 10)

        def _on_scan(self, msg: LaserScan) -> None:
            self.scan_count += 1
            for r in msg.ranges:
                if math.isinf(r) or math.isnan(r):
                    continue
                if r < msg.range_min:
                    continue
                if r < self.min_clearance:
                    self.min_clearance = r

        def _on_odom(self, msg: Odometry) -> None:
            self.odom_count += 1
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            if self._last_xy is not None:
                self.distance_traveled += _euclidean(x, y, *self._last_xy)
            self._last_xy = (x, y)

        def _on_tf(self, msg: TFMessage) -> None:
            self.tf_count += 1
            for transform in msg.transforms:
                edge = f"{transform.header.frame_id}->{transform.child_frame_id}"
                stamp = (
                    transform.header.stamp.sec
                    + transform.header.stamp.nanosec / 1e9
                )
                self.tf_edges.add(edge)
                self.tf_edge_stamps[edge] = max(
                    stamp,
                    self.tf_edge_stamps.get(edge, 0.0),
                )
                first_stamp = self.tf_edge_first_stamps.get(edge)
                if first_stamp is None or stamp < first_stamp:
                    self.tf_edge_first_stamps[edge] = stamp

    rclpy.init()
    collector = MetricsCollector()
    executor = SingleThreadedExecutor()
    executor.add_node(collector)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    navigator = BasicNavigator()

    # Align both nodes to simulation time so goal/pose stamps match the TF tree.
    sim_param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
    collector.set_parameters([sim_param])
    navigator.set_parameters([sim_param])

    # Wait until the simulation clock is flowing before interacting with Nav2.
    clock_deadline = time.time() + 30.0
    while rclpy.ok() and navigator.get_clock().now().nanoseconds == 0:
        rclpy.spin_once(navigator, timeout_sec=0.1)
        if time.time() > clock_deadline:
            break
    def _stamp_msg(stamp_s: float):
        from builtin_interfaces.msg import Time

        sec = int(stamp_s)
        return Time(sec=sec, nanosec=int((stamp_s - sec) * 1e9))

    def make_pose(pose: dict, *, stamp_s: float | None = None) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = task.get("frame_id", "map")
        if stamp_s is None:
            msg.header.stamp = navigator.get_clock().now().to_msg()
        else:
            msg.header.stamp = _stamp_msg(stamp_s)
        msg.pose.position.x = float(pose["x"])
        msg.pose.position.y = float(pose["y"])
        qx, qy, qz, qw = _yaw_to_quat(float(pose.get("yaw", 0.0)))
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def wait_for_demo_nav_ready(timeout_sec: float = 60.0) -> None:
        deadline = time.time() + timeout_sec
        next_initial_pose_publish = 0.0
        initial_pose_publish_count = 0
        last_initial_pose_stamp = None

        # BasicNavigator.waitUntilNav2Active() can block on lifecycle get_state
        # discovery even after Nav2 reports active. For this demo, AMCL pose and
        # action server readiness are the runtime contracts the runner needs.
        while rclpy.ok() and not navigator.initial_pose_received:
            now = time.time()
            if now > deadline:
                raise TimeoutError("Timed out waiting for AMCL to accept the initial pose")
            if now >= next_initial_pose_publish:
                odom_stamp = collector.tf_edge_stamps.get("odom->base_footprint")
                if odom_stamp is None:
                    rclpy.spin_once(navigator, timeout_sec=0.2)
                    continue
                # Stamp 0 asks TF for the latest transform, avoiding a race where
                # AMCL's listener lags the runner's TF subscription by a few frames.
                navigator.initial_pose.header.stamp = _stamp_msg(0.0)
                navigator._setInitialPose()
                initial_pose_publish_count += 1
                last_initial_pose_stamp = 0.0
                next_initial_pose_publish = now + 1.0
            rclpy.spin_once(navigator, timeout_sec=0.2)

        for name, client in (
            ("navigate_to_pose", navigator.nav_to_pose_client),
            ("compute_path_to_pose", navigator.compute_path_to_pose_client),
        ):
            while rclpy.ok() and not client.wait_for_server(timeout_sec=0.5):
                if time.time() > deadline:
                    raise TimeoutError(f"Timed out waiting for {name} action server")

    def wait_for_navigation_tf(timeout_sec: float) -> float:
        deadline = time.time() + timeout_sec
        while rclpy.ok():
            map_stamp = collector.tf_edge_stamps.get("map->odom", 0.0)
            map_first_stamp = collector.tf_edge_first_stamps.get("map->odom", 0.0)
            odom_stamp = collector.tf_edge_stamps.get("odom->base_footprint", 0.0)
            if map_first_stamp > 0.0 and odom_stamp >= map_first_stamp:
                return odom_stamp
            if time.time() > deadline:
                raise TimeoutError("Timed out waiting for map/odom/base TF")
            rclpy.spin_once(navigator, timeout_sec=0.2)

    def wait_for_localization_graph(timeout_sec: float) -> None:
        deadline = time.time() + timeout_sec
        required_topics = {"/clock", "/odom", "/scan", "/initialpose"}
        while rclpy.ok():
            node_names = {name for name, _ in navigator.get_node_names_and_namespaces()}
            topic_names = {name for name, _ in navigator.get_topic_names_and_types()}
            service_names = {name for name, _ in navigator.get_service_names_and_types()}
            ready = (
                "amcl" in node_names
                and "map_server" in node_names
                and "lifecycle_manager_localization" in node_names
                and required_topics.issubset(topic_names)
                and "/amcl/get_state" in service_names
            )
            if ready:
                return
            if time.time() > deadline:
                raise TimeoutError("Timed out waiting for localization graph")
            rclpy.spin_once(navigator, timeout_sec=0.2)

    def wait_for_lifecycle_active(
        timeout_sec: float,
        node_names: tuple[str, ...],
        *,
        label: str,
    ) -> None:
        from lifecycle_msgs.msg import State
        from lifecycle_msgs.srv import GetState

        deadline = time.time() + timeout_sec
        clients = {
            name: navigator.create_client(GetState, f"/{name}/get_state")
            for name in node_names
        }
        last_states: dict[str, dict] = {}
        while rclpy.ok():
            all_active = True
            for name, client in clients.items():
                if not client.wait_for_service(timeout_sec=0.1):
                    all_active = False
                    last_states[name] = {"available": False}
                    continue
                future = client.call_async(GetState.Request())
                state_deadline = time.time() + 0.5
                while rclpy.ok() and not future.done() and time.time() < state_deadline:
                    rclpy.spin_once(navigator, timeout_sec=0.05)
                if not future.done() or future.result() is None:
                    all_active = False
                    last_states[name] = {"available": True, "responded": False}
                    continue
                state = future.result().current_state
                last_states[name] = {
                    "available": True,
                    "responded": True,
                    "id": state.id,
                    "label": state.label,
                }
                if state.id != State.PRIMARY_STATE_ACTIVE:
                    all_active = False
            if all_active:
                return
            if time.time() > deadline:
                raise TimeoutError(f"Timed out waiting for {label} lifecycle active")
            rclpy.spin_once(navigator, timeout_sec=0.2)

    localization_graph_timeout = min(20.0, max(3.0, float(args.wall_timeout) - 5.0))
    wait_for_localization_graph(timeout_sec=localization_graph_timeout)
    wait_for_lifecycle_active(
        timeout_sec=localization_graph_timeout,
        node_names=("map_server", "amcl"),
        label="localization",
    )
    start_pose = make_pose(task["start"])
    goal_pose = make_pose(task["goal"])
    navigator.initial_pose = start_pose
    readiness_timeout = min(60.0, max(5.0, float(args.wall_timeout) - 5.0))
    wait_for_demo_nav_ready(timeout_sec=readiness_timeout)
    wait_for_lifecycle_active(
        timeout_sec=readiness_timeout,
        node_names=("controller_server", "planner_server", "bt_navigator"),
        label="navigation",
    )
    navigation_tf_stamp = wait_for_navigation_tf(
        min(10.0, max(2.0, float(args.wall_timeout) - 5.0))
    )
    start_pose = make_pose(task["start"], stamp_s=navigation_tf_stamp)
    goal_pose = make_pose(task["goal"], stamp_s=navigation_tf_stamp)

    # Planned global path length (best-effort).
    try:
        path = navigator.getPath(start_pose, goal_pose)
        if path is not None and path.poses:
            length = 0.0
            for prev, cur in zip(path.poses[:-1], path.poses[1:]):
                length += _euclidean(
                    prev.pose.position.x, prev.pose.position.y,
                    cur.pose.position.x, cur.pose.position.y,
                )
            result["planned_path_length_m"] = length
    except Exception:  # noqa: BLE001
        pass

    wall_start = time.time()
    navigator.goToPose(goal_pose)

    timed_out = False
    last_feedback = None
    min_distance_remaining = float("inf")
    while not navigator.isTaskComplete():
        last_feedback = navigator.getFeedback()
        if last_feedback is not None:
            min_distance_remaining = min(
                min_distance_remaining, float(last_feedback.distance_remaining)
            )
            nav_time = (
                last_feedback.navigation_time.sec
                + last_feedback.navigation_time.nanosec / 1e9
            )
            if nav_time > args.timeout:
                timed_out = True
                navigator.cancelTask()
                break
        if time.time() - wall_start > args.wall_timeout:
            timed_out = True
            navigator.cancelTask()
            break
        time.sleep(0.2)

    task_result = navigator.getResult()
    result["wall_time_s"] = time.time() - wall_start
    if last_feedback is not None:
        result["nav_time_s"] = (
            last_feedback.navigation_time.sec
            + last_feedback.navigation_time.nanosec / 1e9
        )
        result["n_recoveries"] = int(last_feedback.number_of_recoveries)

    result["min_distance_to_goal_m"] = (
        None if math.isinf(min_distance_remaining) else round(min_distance_remaining, 4)
    )
    result["position_reached"] = bool(
        result["min_distance_to_goal_m"] is not None
        and result["min_distance_to_goal_m"] <= args.goal_tolerance
    )
    result["distance_traveled_m"] = round(collector.distance_traveled, 4)
    clearance = collector.min_clearance
    result["min_clearance_m"] = None if math.isinf(clearance) else round(clearance, 4)
    collision = (not math.isinf(clearance)) and clearance < args.collision_threshold
    result["collision"] = bool(collision)
    result["timed_out"] = bool(timed_out)

    result["result"] = {
        TaskResult.SUCCEEDED: "SUCCEEDED",
        TaskResult.CANCELED: "CANCELED",
        TaskResult.FAILED: "FAILED",
    }.get(task_result, "UNKNOWN")

    if last_feedback is None and result["result"] == "SUCCEEDED":
        result["result"] = "REJECTED"
        result["error"] = "NavigateToPose finished without feedback; goal was likely rejected"
    elif result["result"] != "SUCCEEDED" and result["position_reached"]:
        result["result"] = "POSITION_REACHED"

    result["passed"] = bool(
        result["result"] in {"SUCCEEDED", "POSITION_REACHED"}
        and not timed_out
        and not collision
    )

    executor.shutdown()
    try:
        rclpy.shutdown()
    except Exception:  # noqa: BLE001
        pass


if __name__ == "__main__":
    sys.exit(main())

# Navigation CI/CD Demo

An automated, scripted navigation benchmark built on top of the procedural world
generator. It generates TurtleBot3-tailored worlds, then drives a **TurtleBot3
Waffle** from a fixed start to a fixed goal in headless **Gazebo + Nav2** under a
sweep of **planner x controller x costmap** profiles, collects per-run metrics,
and gates a CI build on a baseline profile succeeding on every world.

## Pipeline

```
configs/turtlebot_nav.yaml
        │  (random_gazebo_world generate)
        ▼
worlds/world_<seed>/  ── world.sdf, map.png/yaml, nav_task.json (fixed start/goal)
        │  (scripts/augment_world.py: inject Gazebo system plugins + ode physics,
        │   rasterize polyline solids -> collidable+visible boxes)
        ▼
world_nav.sdf
        │            nav2_profiles/<profile>.yaml   (scripts/make_profiles.py)
        ▼                       │
launch/trial.launch.py ◄────────┘
   tb3_simulation_launch (Gazebo + TB3 + Nav2)  +  scripts/benchmark_runner.py
        │  (one result JSON per world x profile)
        ▼
reports/results.csv + summary.md + success_rate.png      (orchestrate.py)
        │
        ▼
   CI gate: baseline profile must pass on every world (exit nonzero otherwise)
```

## Components

| File | Role |
| --- | --- |
| [`configs/turtlebot_nav.yaml`](configs/turtlebot_nav.yaml) | Generator config tuned for the Waffle (wider gates/passages, TB-scale rooms). |
| [`scripts/augment_world.py`](scripts/augment_world.py) | Injects `gz-sim-*` system plugins + `ode` physics, and rasterizes the generator's non-extrudable polyline solids into collision+visual boxes (see Known limitations). |
| [`scripts/make_profiles.py`](scripts/make_profiles.py) | Generates Nav2 param files for each planner/controller/inflation profile. |
| [`scripts/benchmark_runner.py`](scripts/benchmark_runner.py) | rclpy + `BasicNavigator`; runs one trial, records metrics, writes result JSON. |
| [`launch/trial.launch.py`](launch/trial.launch.py) | Brings up Gazebo + TB3 + Nav2 for one world/profile and runs the runner. |
| [`orchestrate.py`](orchestrate.py) | Top-level driver: generate → augment → profiles → trials → report → gate. |
| [`scripts/ros_env.sh`](scripts/ros_env.sh) | Loopback DDS discovery setup (see Troubleshooting). |
| [`config/fastdds_localhost.xml`](config/fastdds_localhost.xml) | Fast DDS loopback-only transport profile. |

## Robot and start/goal

- Robot: **TurtleBot3 Waffle** (the model bundled in `nav2_minimal_tb3_sim`).
- Start/goal: the generator samples a deterministic, guaranteed-free, mutually
  reachable start and goal per world and writes them (world-frame metres) to
  `nav_task.json`. The robot is spawned at the start; the runner commands the
  goal. Because they come from the seeded generator, they are reproducible.

## Profiles (the benchmark matrix)

`make_profiles.py` swaps three axes in Nav2's stock `nav2_params.yaml`:

- **Planner**: NavFn, Smac 2D, Theta\*.
- **Controller**: MPPI, DWB, Regulated Pure Pursuit.
- **Costmap**: `inflation_radius` (e.g. 0.25 / 0.30 / 0.40 m). The global costmap is
  inflated more aggressively than the local one so the planner prefers open routes.

`--profiles curated` runs a hand-picked set (the first, `navfn_rpp_baseline`, is
the CI gate). `--profiles full` runs the planner x controller x inflation cross
product.

## Pass/fail gate

A trial **passes** iff:

1. Nav2 reports `SUCCEEDED` (goal reached within the goal checker tolerance),
2. before the navigation `--timeout` (sim seconds), and
3. the robot never came within `--collision-threshold` m of an obstacle
   (minimum `/scan` range; a scan-based collision proxy).

The orchestrator exits non-zero if any **baseline** trial fails, which fails the
CI job. Non-baseline profiles are reported but never fail the build.

## Metrics (per trial)

`result`, `passed`, `collision`, `timed_out`, `nav_time_s`, `wall_time_s`,
`planned_path_length_m`, `distance_traveled_m`, `min_clearance_m`,
`n_recoveries`, `straight_line_distance_m`. Aggregated into
`reports/results.csv`, `reports/summary.md`, and `reports/success_rate.png`.

## Run locally

Prerequisites: ROS 2 Jazzy + Gazebo + Nav2 on `PATH`, and `uv`.

```bash
uv sync
# Generate worlds, run the curated sweep, write reports, apply the gate:
uv run python demo/orchestrate.py \
  --config demo/configs/turtlebot_nav.yaml \
  --worlds 3 --profiles curated --out demo/reports
```

Useful flags: `--seeds 10,11,12`, `--profiles full`, `--timeout`,
`--wall-timeout`, `--headless False` (watch a run in the Gazebo GUI),
`--skip-generate` (reuse worlds), `--dds off` (don't touch host DDS).

Generate the individual artifacts by hand:

```bash
# one world
uv run python -m random_gazebo_world.cli generate \
  --config demo/configs/turtlebot_nav.yaml --seed 10 --out /tmp/w10
uv run python demo/scripts/augment_world.py /tmp/w10/world.sdf
# the profiles
uv run python demo/scripts/make_profiles.py --out demo/nav2_profiles --set curated
# one trial directly
ros2 launch demo/launch/trial.launch.py \
  world:=/tmp/w10/world_nav.sdf map:=/tmp/w10/map.yaml \
  params_file:=demo/nav2_profiles/navfn_rpp_baseline.yaml \
  nav_task:=/tmp/w10/nav_task.json result_out:=/tmp/r.json headless:=True
```

## CI

[`.github/workflows/nav-benchmark.yml`](../.github/workflows/nav-benchmark.yml)
installs Nav2 + Gazebo + the TB3 sim in a `ros:jazzy` container, runs a reduced
sweep headless under `xvfb` with software GL, uploads `reports/**` as artifacts,
and fails the job on a gate failure. Trigger it on `workflow_dispatch` or via the
PR paths filter.

## Troubleshooting

### DDS discovery (multi-process) failures — WSL2 in particular

Nav2, the Gazebo bridge and the benchmark runner are separate processes that
must discover each other over DDS. Two host conditions break that:

- a stale `ROS_DISCOVERY_SERVER` pointing at an unreachable address, and/or
- a loopback interface without the `MULTICAST` flag (common on **WSL2**), so the
  default multicast discovery never reaches other local processes.

`orchestrate.py --dds server` (the default) works around this by starting a
**local Fast DDS discovery server** on `127.0.0.1:11811`, clearing the inherited
discovery env, and pointing every trial process at it through the loopback-only
transport profile in [`config/fastdds_localhost.xml`](config/fastdds_localhost.xml).
You can apply the same setup to an interactive shell with:

```bash
source demo/scripts/ros_env.sh        # start server + export env
source demo/scripts/ros_env.sh stop   # tear it down
```

Verify discovery works with a talker/listener in two terminals (after sourcing
`ros_env.sh` in both): `ros2 run demo_nodes_cpp talker` / `... listener`. If the
listener still hears nothing, your host's DDS is misconfigured at a level below
this demo; installing and selecting CycloneDDS
(`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) is the most reliable fallback on WSL2.

### Headless rendering

Gazebo's sensor system renders the lidar with ogre2 and needs an OpenGL context.
Without a GPU, run under `xvfb-run` with `LIBGL_ALWAYS_SOFTWARE=1` (as the CI
workflow does). If rendering is unavailable entirely, switch the trial backend to
Nav2's GPU-free kinematic `tb3_loopback_simulation.launch.py`.

### Solid fills: polyline → box rasterization

The generator exports its "solid" fill regions (unused Voronoi cells, passage
leftovers) as SDF `<polyline>` geometry. Gazebo **cannot extrude** these
(non-convex, keyhole-shaped) polygons — it logs `Unable to extrude mesh` and
drops them, producing a *phantom* obstacle: occupied on the Nav2 map but with no
collision (the robot drives through it) and no visual (the lidar, which raytraces
the rendered *visual* scene, never sees it), which makes AMCL scan-matching
diverge. `augment_world.py` fixes this by rasterizing each polygon into
axis-aligned `<box>` collision **and** visual primitives, so the simulator
matches the map. (`--solid-resolution` controls the box granularity.)

## Known limitations

This demo is a faithful, fully-automated **pipeline**, but on the current host
(WSL2, software-GL Gazebo) the benchmark does **not** pass green yet. The two
remaining issues, both documented here for transparency:

1. **Planners thread narrow pinch-points.** The generated worlds contain ~0.70 m
   junction pinches alongside genuinely wide (≥1.0 m) routes between the same
   rooms. NavFn and Smac 2D plan the near-shortest path (here 15.07 m planned vs
   14.55 m straight-line) straight through the pinch, *ignoring the inflation
   gradient* — raising `inflation_radius`, `cost_scaling_factor`, or the global
   `robot_radius` does not reroute them. A 0.70 m gap leaves a 0.44 m-wide robot
   only ~0.13 m of margin, so it wedges, exhausts its recoveries, and the trial
   fails. The robust fix is to give the **global** planner its own no-go map where
   the pinches are closed (a second eroded-map `map_server`, or a Nav2
   Keepout-Filter on the global costmap) while AMCL and the local costmap keep the
   true map — offline analysis confirms this yields a connected, pinch-free route
   (min width 0.80 m) in every test world. It is not yet wired into the launch.

2. **Localization drift on a slow sim.** Software-rendered Gazebo runs well below
   real time, so TF/scan timing jitter (`Failed to transform initial pose`,
   "Lookup would require extrapolation") lets AMCL drift over long paths. Once the
   robot is wedged at a pinch the drift places its estimated pose inside an
   obstacle ("Start occupied"), and Nav2 occasionally SIGSEGVs (intercepted by
   ImageMagick's process-wide signal handler, hence the `Magick:` abort line). A
   ground-truth `map→odom` publisher would remove this for CI.

The pass/fail gate, metrics collection, reporting, and CI wiring all work; the
gate currently reports the trials as failing for the reasons above.

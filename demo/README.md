# Navigation CI/CD Demo

Automated Gazebo + Nav2 navigation benchmark for generated worlds.

The demo builds on the procedural generator in the repository root. It creates
TurtleBot3 Waffle-scale worlds, augments the generated SDF for Gazebo Sim, runs
Nav2 under several planner/controller/costmap profiles, writes per-trial metrics,
and applies a CI-style gate to the baseline profile.

## Current Status

Current local status in the WSL2 / ROS 2 Jazzy environment:

- generator unit tests pass: `124 passed`
- generated/augmented SDFs validate with `gz sdf -k`
- `demo/orchestrate.py --dds auto` detects the local Fast-CDR ABI mismatch and
  selects CycloneDDS
- one-world curated run for seed `10` passes the baseline gate

Verified one-world gate result:

```text
gate: 1/1 baseline trials passed
gate PASSED
```

Example result rows from the verified run:

```text
navfn_rpp_baseline  POSITION_REACHED  passed=True
navfn_dwb_tight     POSITION_REACHED  passed=True
smac2d_rpp_loose    FAILED            passed=False
```

Only the baseline profile is a hard gate. Non-baseline profiles are reported for
comparison and may still fail on the current host.

## Pipeline

```text
demo/configs/turtlebot_nav.yaml
        |
        | random_gazebo_world generate
        v
reports/worlds/world_<seed>/
  world.sdf
  map.png / map.yaml
  nav_task.json
        |
        | demo/scripts/augment_world.py
        v
  world_nav.sdf
        |
        | demo/scripts/make_profiles.py
        v
reports/profiles/<profile>.yaml
        |
        | demo/launch/trial.launch.py
        v
Gazebo Sim + TurtleBot3 Waffle + Nav2 + benchmark_runner.py
        |
        v
reports/trials/*.json
reports/results.csv
reports/summary.md
reports/success_rate.png
        |
        v
CI gate: all baseline rows must pass
```

## Components

| File | Role |
| --- | --- |
| [configs/turtlebot_nav.yaml](configs/turtlebot_nav.yaml) | Generator config tuned for TurtleBot3 Waffle navigation. |
| [scripts/augment_world.py](scripts/augment_world.py) | Injects Gazebo systems, switches physics to ODE, and converts legacy polyline solids if present. |
| [scripts/make_profiles.py](scripts/make_profiles.py) | Generates Nav2 params for curated or full profile sets. |
| [scripts/benchmark_runner.py](scripts/benchmark_runner.py) | Runs one `BasicNavigator` trial, collects metrics, writes result JSON. |
| [launch/trial.launch.py](launch/trial.launch.py) | Launches Gazebo, TurtleBot3, Nav2, and the benchmark runner. |
| [orchestrate.py](orchestrate.py) | Top-level benchmark driver: generate, augment, profile, trial, report, gate. |
| [scripts/ros_env.sh](scripts/ros_env.sh) | Optional interactive DDS setup helper. |
| [config/fastdds_localhost.xml](config/fastdds_localhost.xml) | Fast DDS loopback-only discovery-server profile. |

## Requirements

Required for local benchmark runs:

- ROS 2 Jazzy
- Gazebo Sim 8 from ROS Jazzy vendor packages
- Nav2 bringup
- Nav2 simple commander
- TurtleBot3 minimal simulation
- `ros_gz_sim` and `ros_gz_bridge`
- `uv`
- an OpenGL context for Gazebo sensors, or software GL via `LIBGL_ALWAYS_SOFTWARE=1`

On this WSL2 host, the installed ROS pieces include:

- `/opt/ros/jazzy`
- `nav2_bringup`
- `ros_gz_sim`
- `ros_gz_bridge`
- `rmw_fastrtps_cpp`
- `rmw_cyclonedds_cpp`

`gz` is available at:

```bash
/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz
```

Use this setup in each shell:

```bash
source /opt/ros/jazzy/setup.bash
export PATH=/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH
export LIBGL_ALWAYS_SOFTWARE=1
```

For CI or headless hosts without a display, run under `xvfb-run` as the workflow
does.

## Run The Benchmark

Install Python dependencies from the repository root:

```bash
uv sync
```

Run a fast one-world baseline gate check:

```bash
source /opt/ros/jazzy/setup.bash
export PATH=/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH
export LIBGL_ALWAYS_SOFTWARE=1

uv run python demo/orchestrate.py \
  --config demo/configs/turtlebot_nav.yaml \
  --seeds 10 \
  --profiles curated \
  --out /tmp/worldgen_nav_check \
  --timeout 60 \
  --wall-timeout 140 \
  --launch-timeout 60 \
  --headless True \
  --dds auto
```

Run the default local benchmark:

```bash
uv run python demo/orchestrate.py \
  --config demo/configs/turtlebot_nav.yaml \
  --worlds 3 \
  --profiles curated \
  --out demo/reports
```

Run specific seeds:

```bash
uv run python demo/orchestrate.py \
  --config demo/configs/turtlebot_nav.yaml \
  --seeds 10,11,12 \
  --profiles curated \
  --out demo/reports
```

Run the full planner/controller/inflation matrix:

```bash
uv run python demo/orchestrate.py \
  --config demo/configs/turtlebot_nav.yaml \
  --seeds 10 \
  --profiles full \
  --out demo/reports_full
```

## Orchestrator Options

Important `demo/orchestrate.py` flags:

| Flag | Default | Meaning |
| --- | --- | --- |
| `--config` | `demo/configs/turtlebot_nav.yaml` | Generator config. |
| `--worlds` | `3` | Number of worlds; seeds are `config.random_seed + i`. |
| `--seeds` | none | Comma-separated explicit seeds; overrides `--worlds`. |
| `--profiles` | `curated` | `curated` or `full`. |
| `--out` | `demo/reports` | Report and generated-world output directory. |
| `--timeout` | `120` | Navigation timeout in sim seconds. |
| `--wall-timeout` | `300` | Per-run wall-clock timeout for the benchmark runner. |
| `--launch-timeout` | `180` | Extra wall-clock time before killing the launch group. |
| `--collision-threshold` | `0.16` | Minimum scan range below which a collision is flagged. |
| `--goal-tolerance` | `0.35` | Position tolerance for benchmark success. |
| `--headless` | `True` | `False` opens the Gazebo GUI. |
| `--skip-generate` | off | Reuse existing `<out>/worlds/world_<seed>` artifacts. |
| `--dds` | `auto` | `auto`, `server`, `cyclone`, or `off`. |
| `--no-kill-stragglers` | off | Leave spawned simulator/Nav2 processes alone between trials. |

When `--skip-generate` is used with a fresh output directory, the orchestrator
also checks `demo/reports/worlds` for reusable generated worlds.

## DDS Modes

The benchmark launches many processes, so DDS discovery must work reliably.

`--dds auto` is the recommended default:

- uses Fast DDS discovery-server mode when Fast DDS appears healthy
- switches to CycloneDDS if the installed Fast-CDR library is ABI-incompatible
  with Nav2 message typesupport

On this host, `--dds auto` prints:

```text
Fast-CDR ABI check failed; auto-selecting CycloneDDS
using CycloneDDS (rmw_cyclonedds_cpp)
```

Other modes:

```bash
--dds server   # force Fast DDS discovery server on 127.0.0.1:11811
--dds cyclone  # force RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
--dds off      # do not change host DDS environment
```

For interactive shells:

```bash
source demo/scripts/ros_env.sh
DEMO_DDS=cyclone source demo/scripts/ros_env.sh
source demo/scripts/ros_env.sh stop
```

Use `--dds cyclone` on WSL2 if you see errors like:

```text
undefined symbol: _ZN8eprosima7fastcdr3Cdr9serializeEj
```

or if local process discovery is unreliable with Fast DDS.

## Profiles

`demo/scripts/make_profiles.py` starts from Nav2's stock `nav2_params.yaml` and
rewrites selected planner/controller/costmap settings.

Curated profiles:

| Name | Planner | Controller | Inflation | Gate |
| --- | --- | --- | --- | --- |
| `navfn_rpp_baseline` | NavFn | Regulated Pure Pursuit | `0.30` | yes |
| `navfn_dwb_tight` | NavFn | DWB | `0.25` | no |
| `smac2d_rpp_loose` | Smac 2D | Regulated Pure Pursuit | `0.40` | no |

The first curated profile is the CI gate. Non-baseline profiles are included in
the reports but do not fail the build.

The generated profiles also:

- use `base_footprint` consistently for Nav2 robot-base frames
- inflate the global costmap more aggressively than the local costmap
- enlarge the global planning radius so narrow pinch points are less attractive
- remove live obstacle layers from the global costmap
- keep local lidar layers for collision safety
- relax the final heading requirement so the benchmark measures reaching the
  goal location, not final yaw precision

## Pass/Fail Semantics

A trial passes when all are true:

1. Nav2 reports `SUCCEEDED`, or the runner observes the robot within
   `--goal-tolerance` metres of the goal position.
2. The trial finishes before `--timeout` sim seconds and `--wall-timeout` wall
   seconds.
3. The scan-based collision proxy never drops below `--collision-threshold`.

If the robot reaches the position tolerance but Nav2 eventually returns
`FAILED`, the runner records:

```json
{
  "result": "POSITION_REACHED",
  "position_reached": true,
  "passed": true
}
```

This behavior is intentional for the CI gate because the demo benchmarks
goal-position reachability in a slow, software-rendered simulator where Nav2 can
fail during post-arrival replanning/recovery bookkeeping.

Rejected goals are not treated as success. If `BasicNavigator` finishes without
feedback after goal submission, the runner records `REJECTED`.

## Reports

`orchestrate.py` writes:

```text
<out>/
  worlds/world_<seed>/         # generated worlds
  profiles/*.yaml             # generated Nav2 params
  profiles/profiles.json       # profile manifest
  trials/*.json                # one JSON result per world/profile trial
  results.csv                  # all trial rows
  summary.md                   # human-readable summary
  success_rate.png             # profile success-rate chart, if matplotlib works
```

CSV/result fields include:

- `world_id`, `seed`
- `profile`, `planner`, `controller`, `inflation`
- `is_gate`
- `result`, `passed`
- `position_reached`, `min_distance_to_goal_m`
- `collision`, `timed_out`
- `nav_time_s`, `wall_time_s`
- `planned_path_length_m`
- `distance_traveled_m`
- `min_clearance_m`
- `n_recoveries`
- `straight_line_distance_m`
- `error`

## Manual Step-By-Step Run

Generate one world:

```bash
uv run python -m random_gazebo_world.cli generate \
  --config demo/configs/turtlebot_nav.yaml \
  --seed 10 \
  --out /tmp/w10
```

Augment it for Gazebo + Nav2:

```bash
uv run python demo/scripts/augment_world.py \
  /tmp/w10/world.sdf \
  --output /tmp/w10/world_nav.sdf
```

Validate the augmented SDF:

```bash
source /opt/ros/jazzy/setup.bash
export PATH=/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH

gz sdf -k /tmp/w10/world_nav.sdf
```

Generate Nav2 profiles:

```bash
uv run python demo/scripts/make_profiles.py \
  --out /tmp/nav2_profiles \
  --set curated
```

Run one trial:

```bash
source /opt/ros/jazzy/setup.bash
export PATH=/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH
export LIBGL_ALWAYS_SOFTWARE=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT ROS_DISCOVERY_INTERFACE
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE

ros2 launch demo/launch/trial.launch.py \
  world:=/tmp/w10/world_nav.sdf \
  map:=/tmp/w10/map.yaml \
  params_file:=/tmp/nav2_profiles/navfn_rpp_baseline.yaml \
  nav_task:=/tmp/w10/nav_task.json \
  result_out:=/tmp/r.json \
  timeout:=60 \
  wall_timeout:=140 \
  collision_threshold:=0.16 \
  goal_tolerance:=0.35 \
  headless:=True \
  use_rviz:=False \
  world_id:=10 \
  profile:=navfn_rpp_baseline \
  planner:=navfn \
  controller:=rpp \
  inflation:=0.3
```

Inspect the result:

```bash
cat /tmp/r.json
```

## CI

The workflow is [.github/workflows/nav-benchmark.yml](../.github/workflows/nav-benchmark.yml).

It runs in a `ros:jazzy` container, installs Nav2, Gazebo, TurtleBot3 sim,
Fast DDS, CycloneDDS, software GL support, and `uv`, then runs:

```bash
xvfb-run -a --server-args="-screen 0 1280x1024x24" \
  uv run python demo/orchestrate.py \
    --config demo/configs/turtlebot_nav.yaml \
    --worlds <input> \
    --profiles <input> \
    --out demo/reports \
    --timeout 90 \
    --wall-timeout 240
```

The workflow uploads report artifacts even on failure.

## Troubleshooting

### `gz` Exists But Prints No Commands

Run with the ROS setup and vendor bin on `PATH`:

```bash
source /opt/ros/jazzy/setup.bash
export PATH=/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH
gz sim --versions
```

On this host, `gz sim --versions` reports `8.11.0`.

### Fast-CDR Symbol Errors

If Nav2 nodes die with:

```text
undefined symbol: _ZN8eprosima7fastcdr3Cdr9serializeEj
```

use:

```bash
--dds auto
```

or force:

```bash
--dds cyclone
```

The underlying issue is a host package mismatch between Fast-CDR and Nav2
message typesupport. CycloneDDS avoids that Fast DDS path.

### DDS Discovery Failures

Symptoms:

- runner cannot see Nav2 services/actions
- nodes appear to launch but do not discover each other
- WSL2 multicast-related issues

Try:

```bash
uv run python demo/orchestrate.py ... --dds auto
uv run python demo/orchestrate.py ... --dds cyclone
uv run python demo/orchestrate.py ... --dds server
```

For manual testing, verify ROS discovery with two shells using talker/listener.

### Headless Rendering

Gazebo lidar rendering needs an OpenGL context. On headless systems:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
xvfb-run -a --server-args="-screen 0 1280x1024x24" \
  uv run python demo/orchestrate.py ...
```

If rendering is unavailable entirely, the next robust direction is replacing the
full Gazebo backend with Nav2's loopback simulator for CI-only testing.

### Leftover Simulator Processes

The orchestrator kills common stragglers between trials by default:

- `gz sim`
- `ros_gz_bridge`
- `parameter_bridge`
- `component_container_isolated`
- `benchmark_runner.py`
- `robot_state_publisher`

Use `--no-kill-stragglers` only when actively debugging a live launch.

## Known Limitations

### Baseline Gate Passes; Non-Baseline Profiles May Fail

The current gate is intentionally limited to `navfn_rpp_baseline`. On the
verified seed `10`, baseline and DWB reached the goal position, while
`smac2d_rpp_loose` still failed before reaching the goal.

This is acceptable for the CI gate because non-baseline profiles are diagnostic
comparison runs.

### `POSITION_REACHED` Is A Benchmark Result, Not A Native Nav2 Result

Nav2 can return `FAILED` after the robot has already reached the goal position,
usually because of late replanning, recovery, or localization drift near the
goal. The runner maps this condition to `POSITION_REACHED` when
`min_distance_to_goal_m <= --goal-tolerance` and no collision/timeout occurred.

### Slow Software Simulation Can Drift

In WSL2 with software rendering, Gazebo can run below real time. TF and scan
timing jitter can cause AMCL drift, and the global planner may later report:

```text
Start occupied
Lookup would require extrapolation
Failed to create plan
```

A future CI-hardening option is a ground-truth `map -> odom` publisher or a
loopback simulation backend for deterministic tests.

### Planner Pinch Points Are Still A Real Scenario

Generated worlds may contain narrow local pinch points alongside wider routes.
The profiles make the global costmap more conservative, but planners can still
prefer short routes that become hard for the controller under localization drift.

Potential future fixes:

- create a separate eroded global-planning map
- add a Nav2 keepout filter for pinch closures
- enforce wider minimum passage geometry in demo-specific generation
- sample start/goal pairs with a stronger clearance and route-width constraint

### Gazebo Teardown Can Exit With `-11`

Gazebo Sim may exit with `-11` during shutdown after the benchmark result has
already been written and the launch is being torn down. The orchestrator uses
the runner's JSON result as the source of truth and then kills stragglers before
the next trial.

### Legacy Polyline Solids

Modern generator output exports solid fills as boxes or meshes. The augmentation
script still supports legacy `<polyline>` solid conversion because Gazebo cannot
extrude many non-convex/keyhole polyline polygons into reliable collision and
visual geometry.

## Useful Verification Commands

Run all unit tests:

```bash
.venv/bin/python -m pytest tests -q -s
```

Validate an augmented world:

```bash
gz sdf -k /tmp/w10/world_nav.sdf
```

Run the one-world gate check:

```bash
uv run python demo/orchestrate.py \
  --config demo/configs/turtlebot_nav.yaml \
  --seeds 10 \
  --profiles curated \
  --out /tmp/worldgen_nav_check \
  --timeout 60 \
  --wall-timeout 140 \
  --launch-timeout 60 \
  --headless True \
  --dds auto
```

Inspect the gate summary:

```bash
cat /tmp/worldgen_nav_check/summary.md
cat /tmp/worldgen_nav_check/results.csv
```

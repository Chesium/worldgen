# random-gazebo-world

Procedural Gazebo world generator for flat indoor rectangular-room layouts.

## Status

Pass 0 complete: project scaffold, config loading, and CLI skeleton.

## Run

```bash
python -m random_gazebo_world.cli generate \
  --config configs/default.yaml \
  --seed 42 \
  --out outputs/world_42
```

## Test

```bash
uv run worldgen-test
```

On systems with ROS pytest plugins installed, this disables external plugin autoload to avoid conflicts.

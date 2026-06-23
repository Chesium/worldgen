# AGENTS.md

## Cursor Cloud specific instructions

This repo is a single Python package, `random_gazebo_world` (a procedural Gazebo
world generator), managed with **uv** (Python 3.12). The startup update script
already runs `uv sync`, so the `.venv` is ready when a session begins.

### Services / scope
- **Core product (set up here):** the `random_gazebo_world` generator — CLI + tests.
- **`demo/` (optional, NOT set up here):** a heavy ROS 2 Jazzy + Gazebo + Nav2
  navigation benchmark (`demo/orchestrate.py`). It is designed to run inside a
  `ros:jazzy` container (see `.github/workflows/nav-benchmark.yml`) and requires
  ROS 2 / Gazebo / TurtleBot3 sim packages that are not installed in this VM.

### Common commands (see `README.md` for full details)
- Tests: `uv run worldgen-test` (wraps pytest; disables external plugin autoload).
- Generate a world:
  `uv run python -m random_gazebo_world.cli generate --config configs/default.yaml --seed <seed> --out outputs/world_<seed>`
- No linter/formatter is configured (no ruff/flake8/black/mypy). `uv run python -m compileall random_gazebo_world` works as a quick syntax check.

### Non-obvious gotchas
- `tests/test_config.py::test_load_default_config` and `test_with_seed_override`
  currently FAIL: they expect `random_seed == 42`, but `configs/default.yaml`
  ships `random_seed: 10667`. This is a pre-existing repo data mismatch, not an
  environment problem (121/123 tests pass).
- Some generator seeds raise `SdfExportError: Could not triangulate solid
  polygon ...` during SDF export (e.g. seeds 42, 7, 10667). This is a generator
  robustness edge case, not an environment issue. Seeds 1 and 100 succeed and
  produce full output (`world.sdf`, `map.png`/`map.yaml`, `layout.json`,
  `metadata.json`, `nav_task.json`, and `debug/` stage images).

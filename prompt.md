# PRD: Procedural Gazebo World Generator (v1)

Build v1 of a Python procedural Gazebo world generator for flat indoor rectangular-room layouts.

## Goal

Generate random one-level Gazebo worlds consisting of rectangular rooms connected by configurable gates and passages. The generator must emphasize visual debugging by exporting images/SVGs of every major generation stage. Output must be reproducible from a random seed.

## Core Assumptions

- 2D floor plan only.
- No slopes, no ceiling.
- Same wall height everywhere.
- Rooms are axis-aligned rectangles.
- Gates only occur between two adjacent room cells sharing a wall.
- Passages connect two rooms through one or more non-room/empty cells.
- Passage/gate width must be configurable.
- Output is reproducible from a random seed.

## Architecture

Use a rectangular partition / BSP-style layout substrate.

- Use Python dataclasses for `Cell`, `Room`, `Opening`, `WallSegment`, `Connection`.
- Use `shapely` for geometry operations if helpful.
- Use `networkx` for graph algorithms if helpful.
- Use `matplotlib` or SVG generation for debug visualization.
- Keep Gazebo export simple: static model with box wall collisions/visuals.
- Prioritize correctness, reproducibility, and visual inspectability over realism.

## Target Project Structure

```
random_gazebo_world/
  pyproject.toml
  README.md
  configs/
    default.yaml
  random_gazebo_world/
    __init__.py
    config.py
    geometry.py
    partition.py
    adjacency.py
    topology.py
    openings.py
    walls.py
    export_sdf.py
    export_map.py
    visualize.py
    metadata.py
    cli.py
  tests/
    test_geometry.py
    test_connectivity.py
    test_openings.py
  outputs/
```

## V1 Scope Limits

Do not implement arbitrary polygon rooms, Voronoi cells, furniture, dynamic obstacles, multi-floor layouts, slopes, doors with physics, textures, or realistic decoration. Focus only on robust rectangular rooms, gates, passages, wall generation, Gazebo SDF export, Nav2 map export, and staged visualization.

---

# Implementation Passes

Each pass is a self-contained increment. Implement one pass at a time, run its validation, and only proceed once the validation gate passes. Every pass that produces geometry or topology must also emit its corresponding debug visualization.

---

## Pass 0 — Project Scaffold & Config

**Objective:** Establish the package, dependencies, config loading, and CLI skeleton.

**Tasks:**
- Create the project structure above.
- Define dependencies in `pyproject.toml` (`shapely`, `networkx`, `matplotlib`, `pyyaml`, `numpy`, `pytest`).
- Implement `config.py`: load/validate a YAML config into a typed dataclass.
- Create `configs/default.yaml` with all fields below.
- Implement `cli.py` skeleton: `generate --config <path> --seed <int> --out <dir>` that parses args, loads config, seeds RNG, and creates the output directory.

**Config fields:**
- world width/height
- min/max cell size
- min/max room count
- wall height
- wall thickness
- gate width range
- passage width range
- extra loop probability
- map resolution
- random seed

**Validation gate:**
- `python -m random_gazebo_world.cli generate --config configs/default.yaml --seed 42 --out outputs/world_42` runs without error and creates `outputs/world_42/`.
- Invalid configs raise clear validation errors.
- Same seed produces identical RNG state (reproducibility smoke test).

---

## Pass 1 — Geometry Primitives

**Objective:** Core geometry types and helpers used by all later passes.

**Tasks:**
- Implement `geometry.py`: dataclasses for `Cell` (axis-aligned rectangle), plus helpers for rectangle intersection, shared-edge detection, and overlap-length computation between two adjacent rectangles.
- Add `tests/test_geometry.py`.

**Validation gate:**
- `test_geometry.py` passes: adjacency detection, shared wall segment geometry, and overlap length are correct for known fixtures (including edge-touching, corner-touching, and non-adjacent cases).

---

## Pass 2 — BSP Partition

**Objective:** Recursively partition the world boundary into rectangular cells.

**Tasks:**
- Implement `partition.py`: start from the world boundary and recursively split into rectangular cells, respecting min/max cell size.
- Assign stable integer cell IDs.
- Implement `visualize.py` enough to render stage 1.

**Debug output:** `01_partition.svg/png` — all rectangular BSP cells with cell IDs.

**Validation gate:**
- Cells tile the full world boundary with no gaps or overlaps.
- Every cell respects min/max cell size constraints.
- Same seed produces an identical partition.
- `01_partition` renders correctly.

---

## Pass 3 — Cell Adjacency Graph

**Objective:** Build the adjacency graph over partition cells.

**Tasks:**
- Implement `adjacency.py`: nodes = cells, edges = cells sharing a wall segment; store shared wall geometry and overlap length on each edge.

**Debug output:** `03_cell_adjacency_graph.svg/png` — adjacency graph overlaid on partition.

**Validation gate:**
- Every adjacency edge corresponds to a real shared wall segment with positive overlap length.
- Graph is connected (the full partition is reachable).
- `03_cell_adjacency_graph` renders correctly.

---

## Pass 4 — Room Selection

**Objective:** Randomly choose room cells; mark the rest as unused/empty.

**Tasks:**
- Select between min/max room count cells as rooms; remaining cells become unused/empty.

**Debug output:** `02_selected_rooms.svg/png` — selected room cells highlighted; unused cells shown separately.

**Validation gate:**
- Room count is within `[min_room_count, max_room_count]`.
- Same seed produces identical room selection.
- `02_selected_rooms` renders correctly.

---

## Pass 5 — Candidate Connections

**Objective:** Generate all candidate room-to-room connections.

**Tasks:**
- Implement candidate generation in `topology.py`:
  - if two room cells are adjacent → candidate type = `gate`
  - otherwise find a shortest path through non-room cells → candidate type = `passage`

**Debug output:** `04_candidate_connections.svg/png` — possible gates/passages between rooms.

**Validation gate:**
- Every gate candidate maps to a real shared wall with overlap ≥ required gate width.
- Every passage candidate has a valid path through non-room cells.
- `04_candidate_connections` renders correctly.

---

## Pass 6 — Room Graph Selection

**Objective:** Select a connected room graph from candidate connections.

**Tasks:**
- Build a randomized spanning tree over candidate room connections.
- Optionally add extra loop connections per `extra loop probability`.

**Debug output:** `05_selected_room_graph.svg/png` — final connected room graph after spanning-tree selection.

**Validation gate:**
- Final room graph is connected (all rooms reachable).
- Loop edges respect the configured probability behavior.
- Same seed produces identical selection.
- `05_selected_room_graph` renders correctly.

---

## Pass 7 — Apply Connections (Passage Cells)

**Objective:** Realize selected connections into the layout.

**Tasks:**
- Implement in `openings.py`/`topology.py`:
  - gate: record an opening on the shared wall between adjacent rooms.
  - passage: mark intermediate cells as passage cells and create openings between consecutive cells.

**Debug output:** `06_passage_cells.svg/png` — passage-support cells highlighted.

**Validation gate:**
- Every selected passage has its intermediate cells marked as passage cells.
- No room cell is reclassified as a passage cell.
- `06_passage_cells` renders correctly.

---

## Pass 8 — Openings

**Objective:** Compute concrete gate and passage doorway openings on shared walls.

**Tasks:**
- Implement `openings.py`: place gate openings (within gate width range) and passage doorways (within passage width range) on the appropriate shared wall segments.
- Add `tests/test_openings.py`.

**Debug output:** `07_openings.svg/png` — gates and passage doorways drawn on shared walls.

**Validation gate:**
- `test_openings.py` passes.
- Every opening fits within its shared wall segment and respects its configured width range.
- `07_openings` renders correctly.

---

## Pass 9 — Wall Segments

**Objective:** Generate final wall segments along cell boundaries, subtracting openings.

**Tasks:**
- Implement `walls.py` per boundary rules:
  - room/unused or passage/unused boundary → wall
  - room/room boundary → wall unless a gate opening exists
  - room/passage boundary → wall unless doorway opening exists
  - passage/passage boundary → usually open
  - outside/free-space boundary → wall

**Debug output:** `08_wall_segments.svg/png` — final wall segments after subtracting openings.

**Validation gate:**
- No wall segment overlaps an opening.
- No invalid or tiny (sub-threshold) wall segments.
- `08_wall_segments` renders correctly.

---

## Pass 10 — Occupancy Map Export

**Objective:** Rasterize the layout into a Nav2-style occupancy grid.

**Tasks:**
- Implement `export_map.py`: produce `map.png` + `map.yaml` at the configured resolution.

**Debug output:** `09_occupancy_map_preview.png` — rasterized occupancy grid.

**Validation gate:**
- Free space in the occupancy grid is a single connected component.
- A sampled start/goal pair lies inside free space and is reachable.
- `map.yaml` resolution/origin matches the generated geometry.
- `09_occupancy_map_preview` renders correctly.

---

## Pass 11 — Gazebo SDF Export

**Objective:** Export the world as Gazebo SDF using static box walls.

**Tasks:**
- Implement `export_sdf.py`: emit `world.sdf` with a static model whose box collisions/visuals represent each wall segment, using configured wall height/thickness.

**Validation gate:**
- `world.sdf` is well-formed SDF and loads in Gazebo without errors.
- Wall boxes match wall segment geometry, height, and thickness.

---

## Pass 12 — Metadata, Layout & Final Floorplan

**Objective:** Persist machine-readable layout/metadata and produce the final clean visualization.

**Tasks:**
- Implement `metadata.py`: write `layout.json` (cells, rooms, openings, walls, connections) and `metadata.json` (seed, config snapshot, counts, generation stats).

**Debug output:** `10_final_floorplan.svg/png` — final clean floor plan with rooms, gates, passages, and walls.

**Validation gate:**
- `layout.json` round-trips (re-loadable into the same data structures).
- `metadata.json` records the seed and config used.
- `10_final_floorplan` renders correctly.

---

## Pass 13 — End-to-End Validation & Regeneration

**Objective:** Wire the full pipeline together with reject/regenerate behavior.

**Tasks:**
- Run the full pipeline from the CLI, producing all outputs below.
- Implement reject/regenerate: if a world fails validation, resample (with a bounded retry count) until valid.
- Add `tests/test_connectivity.py` for end-to-end connectivity guarantees.

**Reject/regenerate a world if:**
- selected rooms cannot be connected
- any gate/shared wall overlap is too short
- any passage path is impossible
- final room graph is disconnected
- wall segments are invalid or tiny
- occupancy grid free space is disconnected
- sampled start/goal points are outside free space

**Validation gate:**
- A single CLI run produces the full output set below.
- Same seed reproduces byte-stable (or structurally identical) outputs.
- `test_connectivity.py` passes.

---

## Visualization Requirements

Use clear colors/labels in all stage visualizations for: rooms, passage cells, unused/solid cells, gates, passage openings, wall segments, room graph edges.

For each generated world, create a `debug/` folder with visualizations for every stage (mapping to the passes above):

- `01_partition` (Pass 2)
- `02_selected_rooms` (Pass 4)
- `03_cell_adjacency_graph` (Pass 3)
- `04_candidate_connections` (Pass 5)
- `05_selected_room_graph` (Pass 6)
- `06_passage_cells` (Pass 7)
- `07_openings` (Pass 8)
- `08_wall_segments` (Pass 9)
- `09_occupancy_map_preview` (Pass 10)
- `10_final_floorplan` (Pass 12)

## CLI Requirement

```
python -m random_gazebo_world.cli generate \
  --config configs/default.yaml \
  --seed 42 \
  --out outputs/world_42
```

## Generated Output

```
outputs/world_42/
  world.sdf
  map.png
  map.yaml
  layout.json
  metadata.json
  debug/
    01_partition.svg
    02_selected_rooms.svg
    ...
    10_final_floorplan.svg
```

## README Requirements

The README should explain:
- generation pipeline
- meaning of gates vs passages
- how to run generator
- how to inspect debug visualizations
- how to load the generated world in Gazebo
- how to use generated map with Nav2

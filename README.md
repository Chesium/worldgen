# random-gazebo-world

Procedural Gazebo world generator for flat indoor rectangular-room layouts.

Generate random one-level worlds with rectangular rooms connected by **gates** (doorways through shared walls between adjacent rooms) and **passages** (routes through unused corridor cells). Every generation stage is exported to `debug/` for visual inspection.

## Generation pipeline

1. **BSP partition** — split the world boundary into rectangular cells.
2. **Adjacency graph** — connect cells that share a wall segment.
3. **Room selection** — randomly mark some cells as rooms; the rest stay unused.
4. **Candidate connections** — adjacent room pairs become gate candidates; non-adjacent pairs get passage candidates through unused cells.
5. **Room graph selection** — pick a randomized spanning tree, optionally adding loop edges.
6. **Apply connections** — mark passage cells and logical openings.
7. **Passage constraints** — validate corridor-cell topology (openings per edge, open-edge count); retry with a new seed if violated.
8. **Openings** — place concrete gate/passage doorway widths on shared walls.
9. **Walls** — emit wall segments with openings subtracted.
10. **Occupancy map** — rasterize walkable space for Nav2.
11. **Gazebo SDF** — export static box walls.
12. **Metadata** — write `layout.json` and `metadata.json`.
13. **Validation** — reject and retry with a new derived seed if connectivity or map checks fail.

## Gates vs passages

- **Gate**: two rooms share a wall; a doorway is cut directly in that wall.
- **Passage**: two rooms are not adjacent; unused cells between them become corridor cells with doorways at each step along the path.

Passage cells are currently left as open floor (no corridor geometry yet). `passage_width_min` / `passage_width_max` control doorway gap width at passage boundaries, not the corridor width itself.

## Passage constraints

After connections are applied, each corridor (`PASSAGE`) cell is checked before openings and walls are generated. These constraints prepare layouts for future passage geometry:

- **`max_openings_per_passage_edge`** (default `1`): at most one opening per side of a passage cell.
- **`max_open_edges_per_passage`** (default `4`, range `2`–`4`): cap how many sides of a passage cell may carry openings. Set to `2` to allow only straight or L-shaped two-doorway corridors (no tee/junction cells).

Violations raise a retryable error; the pipeline reseeds and tries again up to **`max_attempts`**.

## Configuration

All generation parameters live in a YAML file (see [`configs/default.yaml`](configs/default.yaml)). Required keys must be present; keys with defaults may be omitted.

| Key | Type | Default | Description |
| --- | --- | --- | --- |
| `world_width` | float | — | World extent along X (meters). |
| `world_height` | float | — | World extent along Y (meters). |
| `min_cell_size` | float | — | Minimum BSP cell edge length. |
| `max_cell_size` | float | — | Maximum BSP cell edge length. |
| `min_room_count` | int | — | Minimum rooms to select. |
| `max_room_count` | int | — | Maximum rooms to select. |
| `wall_height` | float | — | Exported wall height (meters). |
| `wall_thickness` | float | — | Exported wall thickness (meters). |
| `gate_width_min` | float | — | Minimum gate doorway width. |
| `gate_width_max` | float | — | Maximum gate doorway width. |
| `passage_width_min` | float | — | Minimum passage doorway width. |
| `passage_width_max` | float | — | Maximum passage doorway width. |
| `max_openings_per_passage_edge` | int | `1` | Max openings on one edge of a passage cell. |
| `max_open_edges_per_passage` | int | `4` | Max edges of a passage cell that may have openings (`2`–`4`). |
| `extra_loop_probability` | float | — | Chance to add non-tree loop connections (`0`–`1`). |
| `map_resolution` | float | — | Occupancy map resolution (meters/pixel). |
| `random_seed` | int | — | Base random seed. |
| `max_attempts` | int | `100000` | Max generation retries before failing. |

Each retry uses `random_seed + attempt`. Tight passage constraints or high room counts may require raising `max_attempts`.

## Run the generator

```bash
uv sync
python -m random_gazebo_world.cli generate \
  --config configs/default.yaml \
  --seed 42 \
  --out outputs/world_42
```

## Inspect debug visualizations

Each run writes staged images under `outputs/world_42/debug/`:

| Stage | File prefix | Shows |
| --- | --- | --- |
| Partition | `01_partition` | BSP cells with IDs |
| Rooms | `02_selected_rooms` | selected rooms vs unused cells |
| Adjacency | `03_cell_adjacency_graph` | cell adjacency graph |
| Candidates | `04_candidate_connections` | possible gates/passages |
| Selected graph | `05_selected_room_graph` | final room connectivity |
| Passage cells | `06_passage_cells` | corridor cells |
| Openings | `07_openings` | doorway placements |
| Walls | `08_wall_segments` | final wall segments |
| Occupancy | `09_occupancy_map_preview` | Nav2 grid preview |
| Final plan | `10_final_floorplan` | clean composite floor plan |

## Load the world in Gazebo

```bash
gz sim outputs/world_42/world.sdf
```

The exported world contains a static `walls` model made of box collisions/visuals aligned to the generated wall segments.

## Use the map with Nav2

Generated maps follow the standard ROS map format:

- `outputs/world_42/map.yaml`
- `outputs/world_42/map.png`

Point Nav2’s map server at those files. The YAML uses origin `[0.0, 0.0, 0.0]` and the configured `map_resolution`.

## Output layout

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

## Test

```bash
uv run worldgen-test
```

On systems with ROS pytest plugins installed, this disables external plugin autoload to avoid conflicts.

## Notes

- Geometry and adjacency use pure Python helpers; `networkx` powers graph algorithms.
- Generation is reproducible for a given seed when the first attempt succeeds. If a layout fails validation (connectivity, passage constraints, openings, walls, or map checks), the pipeline retries with `seed + attempt` until a valid world is found or `max_attempts` is exhausted.

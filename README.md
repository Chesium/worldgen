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
7. **Openings** — place concrete gate/passage doorway widths on shared walls.
8. **Walls** — emit wall segments with openings subtracted.
9. **Occupancy map** — rasterize walkable space for Nav2.
10. **Gazebo SDF** — export static box walls.
11. **Metadata** — write `layout.json` and `metadata.json`.
12. **Validation** — reject and retry with a new derived seed if connectivity or map checks fail.

## Gates vs passages

- **Gate**: two rooms share a wall; a doorway is cut directly in that wall.
- **Passage**: two rooms are not adjacent; unused cells between them become corridor cells with doorways at each step.

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
- Generation is reproducible for a given seed when the first attempt succeeds. If a layout fails validation, the pipeline retries with `seed + attempt` until a valid world is found.

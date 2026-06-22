# random-gazebo-world

Procedural Gazebo world generator for flat indoor rectangular-room layouts.

Generates one-level worlds with rooms connected by **gates** (doorways in shared walls) and **passages** (corridors through reclassified cells). Exports a Gazebo SDF, Nav2 occupancy map, and staged debug images.

## Pipeline

1. **BSP partition** — split the world into rectangular cells.
2. **Adjacency graph** — edges where cells share a wall.
3. **Room selection** — mark some cells as rooms; the rest start as unused.
4. **Candidate connections** — gate candidates for adjacent room pairs; passage candidates through unused cells otherwise.
5. **Room graph selection** — randomized spanning tree, plus optional loop edges.
6. **Apply connections** — reclassify passage cells and record logical openings.
7. **Passage constraints** — validate opening topology per corridor cell; retry on violation.
8. **Openings** — place concrete doorway widths on shared walls.
9. **Passage geometry** — build corridor strips (straight / L / Z paths) inside passage cells; leftover area becomes solid fill.
10. **Walls** — emit thin wall segments (rooms and passage boundaries only).
11. **Occupancy map** — rasterize walkable space for Nav2.
12. **Gazebo SDF** — export ground slab, walls, and solid fills.
13. **Metadata** — write `layout.json` and `metadata.json`.
14. **Validation** — retry with `random_seed + attempt` on failure.

## Cell roles and geometry

| Role | Walkable (map) | SDF geometry |
| --- | --- | --- |
| **Room** | full cell | perimeter walls with gate openings |
| **Passage** | corridor strips only | corridor floor + solid fill in leftover area; walls on room/unused boundaries |
| **Unused** | none | full-cell solid fill (no thin walls) |

**Gates** connect adjacent rooms through a shared wall. **Passages** route through unused cells; each corridor cell gets axis-aligned paths between its openings (width = min of the pair's doorway widths), unioned into a corridor polygon.

## Passage constraints

Checked after connections are applied (before geometry):

- `max_openings_per_passage_edge` (default `1`) — at most one opening per side.
- `max_open_edges_per_passage` (default `4`, range `2`–`4`) — cap sides with openings; `2` forbids tee/junction cells.

## Configuration

See [`configs/default.yaml`](configs/default.yaml). Required keys must be present; optional keys fall back to defaults.

| Key | Default | Description |
| --- | --- | --- |
| `world_width`, `world_height` | — | World extent (m). |
| `min_cell_size`, `max_cell_size` | — | BSP cell size range (m). |
| `min_room_count`, `max_room_count` | — | Room count range. |
| `wall_height`, `wall_thickness` | — | Exported wall dimensions (m). |
| `gate_width_min`, `gate_width_max` | — | Gate doorway width range (m). |
| `passage_width_min`, `passage_width_max` | — | Passage doorway width range (m). |
| `max_openings_per_passage_edge` | `1` | Max openings on one passage edge. |
| `max_open_edges_per_passage` | `4` | Max open edges per passage cell (`2`–`4`). |
| `extra_loop_probability` | — | Chance to add loop connections (`0`–`1`). |
| `map_resolution` | — | Occupancy map resolution (m/px). |
| `random_seed` | — | Base seed. |
| `max_attempts` | `100000` | Max retries before failing. |
| `ground_thickness` | `0.1` | Ground slab thickness in SDF (m). |

## Run

```bash
uv sync
python -m random_gazebo_world.cli generate \
  --config configs/default.yaml \
  --seed 42 \
  --out outputs/world_42
```

## Debug stages

Under `outputs/world_42/debug/`:

| Stage | Shows |
| --- | --- |
| `01_partition` | BSP cells |
| `02_selected_rooms` | rooms vs unused |
| `03_cell_adjacency_graph` | cell adjacency |
| `04_candidate_connections` | gate/passage candidates |
| `05_selected_room_graph` | final connectivity |
| `06_passage_cells` | corridor cells |
| `07_openings` | doorway placements |
| `08_wall_segments` | wall segments |
| `09_occupancy_map_preview` | Nav2 grid |
| `10_final_floorplan` | composite plan |
| `11_passage_geometry` | corridors vs solid fill |

## Outputs

```
outputs/world_42/
  world.sdf          # Gazebo world
  map.png / map.yaml # Nav2 map (origin [0, 0, 0])
  layout.json        # full layout geometry
  metadata.json      # seed, config, counts
  debug/             # staged SVG + PNG
```

### `world.sdf`

Static models with shared scene lighting (matching reference `np.sdf`):

- **`ground`** — box slab `world_width × world_height × ground_thickness`, top at z = 0.
- **`walls`** — thin wall segments, passage/unused solid fills, same PBR material.

Load in Gazebo:

```bash
gz sim outputs/world_42/world.sdf
```

## Test

```bash
uv run worldgen-test
```

On systems with ROS pytest plugins, this disables external plugin autoload to avoid conflicts.

## Notes

- Pure Python geometry; `networkx` for graph algorithms; `shapely` for passage corridor unions.
- Reproducible when the first attempt succeeds. Retries increment the seed until validation passes or `max_attempts` is exhausted.

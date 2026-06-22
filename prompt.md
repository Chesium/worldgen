Build v1 of a Python procedural Gazebo world generator for flat indoor rectangular-room layouts.

Goal:
Generate random one-level Gazebo worlds consisting of rectangular rooms connected by configurable gates and passages. The generator must emphasize visual debugging by exporting images/SVGs of every major generation stage.

Core assumptions:
- 2D floor plan only.
- No slopes, no ceiling.
- Same wall height everywhere.
- Rooms are axis-aligned rectangles.
- Gates only occur between two adjacent room cells sharing a wall.
- Passages connect two rooms through one or more non-room/empty cells.
- Passage/gate width must be configurable.
- Output should be reproducible from a random seed.

Architecture:
Use a rectangular partition / BSP-style layout substrate.

Pipeline:
1. Start from a large rectangular world boundary.
2. Recursively partition it into rectangular cells.
3. Build a cell adjacency graph:
   - nodes = rectangular cells
   - edges = cells sharing a wall segment
   - store shared wall geometry and overlap length
4. Randomly select some cells as rooms.
5. Mark remaining cells as unused/empty.
6. Generate candidate room connections:
   - if two room cells are adjacent, candidate type = gate
   - otherwise find a shortest path through non-room cells, candidate type = passage
7. Select a connected room graph:
   - use randomized spanning tree over candidate room connections
   - optionally add extra loop connections
8. Apply selected connections:
   - gate: create opening on shared wall between adjacent rooms
   - passage: mark intermediate cells as passage cells and create openings between consecutive cells
9. Generate wall segments along cell boundaries:
   - room/unused or passage/unused boundary => wall
   - room/room boundary => wall unless a gate opening exists
   - room/passage boundary => wall unless doorway opening exists
   - passage/passage boundary => usually open
   - outside/free-space boundary => wall
10. Export Gazebo SDF using simple static box walls.
11. Export Nav2-style occupancy map PNG + YAML.
12. Export metadata JSON.

Important visualization requirement:
For each generated world, create a `debug/` folder with visualizations for every stage:

- `01_partition.svg/png`: all rectangular BSP cells with cell IDs.
- `02_selected_rooms.svg/png`: selected room cells highlighted; unused cells shown separately.
- `03_cell_adjacency_graph.svg/png`: adjacency graph overlaid on partition.
- `04_candidate_connections.svg/png`: possible gate/passages between rooms.
- `05_selected_room_graph.svg/png`: final connected room graph after spanning-tree selection.
- `06_passage_cells.svg/png`: passage-support cells highlighted.
- `07_openings.svg/png`: gates and passage doorways drawn on shared walls.
- `08_wall_segments.svg/png`: final wall segments after subtracting openings.
- `09_occupancy_map_preview.png`: rasterized occupancy grid.
- `10_final_floorplan.svg/png`: final clean floor plan with rooms, gates, passages, and walls.

Use clear colors/labels in visualization:
- rooms
- passage cells
- unused/solid cells
- gates
- passage openings
- wall segments
- room graph edges

Suggested project structure:
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

CLI requirements:
Implement a command like:

python -m random_gazebo_world.cli generate \
  --config configs/default.yaml \
  --seed 42 \
  --out outputs/world_42

Config should include:
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

Generated output:
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

Implementation guidance:
- Use Python dataclasses for Cell, Room, Opening, WallSegment, Connection.
- Use shapely for geometry operations if helpful.
- Use networkx for graph algorithms if helpful.
- Use matplotlib or SVG generation for debug visualization.
- Keep Gazebo export simple: static model with box wall collisions/visuals.
- Prioritize correctness, reproducibility, and visual inspectability over realism.

Validation requirements:
Reject/regenerate worlds if:
- selected rooms cannot be connected
- any gate/shared wall overlap is too short
- any passage path is impossible
- final room graph is disconnected
- wall segments are invalid or tiny
- occupancy grid free space is disconnected
- sampled start/goal points are outside free space

README should explain:
- generation pipeline
- meaning of gates vs passages
- how to run generator
- how to inspect debug visualizations
- how to load the generated world in Gazebo
- how to use generated map with Nav2

V1 scope limits:
Do not implement arbitrary polygon rooms, Voronoi cells, furniture, dynamic obstacles, multi-floor layouts, slopes, doors with physics, textures, or realistic decoration. Focus only on robust rectangular rooms, gates, passages, wall generation, Gazebo SDF export, Nav2 map export, and staged visualization.
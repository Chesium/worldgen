from __future__ import annotations

import random
import shutil
from dataclasses import dataclass
from pathlib import Path

import networkx as nx

from random_gazebo_world.adjacency import AdjacencyGraph, build_adjacency_graph
from random_gazebo_world.config import Config
from random_gazebo_world.export_map import (
    OccupancyMap,
    OccupancyMapError,
    generate_occupancy_map,
    write_occupancy_map_files,
)
from random_gazebo_world.export_sdf import export_world_sdf
from random_gazebo_world.metadata import (
    LayoutDocument,
    build_layout_document,
    export_layout_json,
    export_metadata_json,
)
from random_gazebo_world.openings import OpeningError, OpeningLayout, generate_openings
from random_gazebo_world.partition import Partition, PartitionError, generate_partition
from random_gazebo_world.rng import create_seeded_rng
from random_gazebo_world.topology import (
    AppliedLayout,
    AppliedLayoutError,
    CandidateConnectionError,
    CandidateConnections,
    RoomGraphSelectionError,
    RoomSelection,
    RoomSelectionError,
    SelectedRoomGraph,
    apply_connections,
    generate_candidate_connections,
    select_room_graph,
    select_rooms,
    validate_selected_room_graph,
)
from random_gazebo_world.walls import WallGenerationError, WallLayout, generate_walls
from random_gazebo_world.visualize import (
    render_adjacency_graph,
    render_candidate_connections,
    render_final_floorplan,
    render_openings,
    render_partition,
    render_passage_cells,
    render_selected_room_graph,
    render_selected_rooms,
    render_wall_segments,
)


DEFAULT_MAX_ATTEMPTS = 32

REQUIRED_OUTPUTS = (
    "world.sdf",
    "map.png",
    "map.yaml",
    "layout.json",
    "metadata.json",
)

REQUIRED_DEBUG_STAGES = (
    "01_partition",
    "02_selected_rooms",
    "03_cell_adjacency_graph",
    "04_candidate_connections",
    "05_selected_room_graph",
    "06_passage_cells",
    "07_openings",
    "08_wall_segments",
    "09_occupancy_map_preview",
    "10_final_floorplan",
)


class WorldValidationError(RuntimeError):
    """Raised when a generated world fails end-to-end validation."""


class WorldGenerationError(RuntimeError):
    """Raised when no valid world could be generated within retry limits."""


RETRYABLE_ERRORS = (
    WorldValidationError,
    RoomGraphSelectionError,
    RoomSelectionError,
    CandidateConnectionError,
    AppliedLayoutError,
    OpeningError,
    WallGenerationError,
    OccupancyMapError,
    PartitionError,
)


@dataclass(frozen=True)
class GeneratedWorld:
    config: Config
    partition: Partition
    adjacency: AdjacencyGraph
    room_selection: RoomSelection
    candidates: CandidateConnections
    selected_graph: SelectedRoomGraph
    applied_layout: AppliedLayout
    opening_layout: OpeningLayout
    wall_layout: WallLayout
    occupancy: OccupancyMap
    layout_document: LayoutDocument
    attempt: int


def generate_valid_world(
    config: Config,
    max_attempts: int = DEFAULT_MAX_ATTEMPTS,
) -> GeneratedWorld:
    last_error: Exception | None = None

    for attempt in range(max_attempts):
        attempt_config = config.with_seed(config.random_seed + attempt)
        rng = create_seeded_rng(attempt_config.random_seed)
        try:
            world = _generate_world_attempt(attempt_config, rng, attempt)
            validate_world_connectivity(world)
            return world
        except RETRYABLE_ERRORS as exc:
            last_error = exc
            continue

    raise WorldGenerationError(
        f"Failed to generate a valid world after {max_attempts} attempts"
    ) from last_error


def write_world_outputs(world: GeneratedWorld, out_dir: Path) -> None:
    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    debug_dir = out_dir / "debug"
    debug_dir.mkdir(parents=True, exist_ok=True)

    export_layout_json(out_dir / "layout.json", world.layout_document)
    export_metadata_json(
        out_dir / "metadata.json",
        world.config,
        world.layout_document,
        world.selected_graph,
    )
    render_partition(world.partition, debug_dir / "01_partition")
    render_selected_rooms(
        world.partition,
        world.room_selection,
        debug_dir / "02_selected_rooms",
    )
    render_adjacency_graph(
        world.partition,
        world.adjacency,
        debug_dir / "03_cell_adjacency_graph",
    )
    render_candidate_connections(
        world.partition,
        world.room_selection,
        world.candidates,
        debug_dir / "04_candidate_connections",
    )
    render_selected_room_graph(
        world.partition,
        world.selected_graph,
        debug_dir / "05_selected_room_graph",
    )
    render_passage_cells(world.applied_layout, debug_dir / "06_passage_cells")
    render_openings(world.opening_layout, debug_dir / "07_openings")
    render_wall_segments(world.wall_layout, debug_dir / "08_wall_segments")
    write_occupancy_map_files(world.occupancy, out_dir)
    export_world_sdf(world.wall_layout, world.config, out_dir / "world.sdf")
    render_final_floorplan(world.wall_layout, debug_dir / "10_final_floorplan")

    _validate_output_tree(out_dir)


def validate_world_connectivity(world: GeneratedWorld) -> None:
    room_ids = sorted(world.room_selection.room_cell_ids)
    if len(room_ids) <= 1:
        return

    candidate_graph = nx.Graph()
    candidate_graph.add_nodes_from(room_ids)
    for connection in world.candidates.connections:
        candidate_graph.add_edge(connection.room_a_id, connection.room_b_id)

    if not nx.is_connected(candidate_graph):
        raise WorldValidationError("Selected rooms cannot be connected via candidates")

    validate_selected_room_graph(world.selected_graph, world.config)


def _generate_world_attempt(
    config: Config,
    rng: random.Random,
    attempt: int,
) -> GeneratedWorld:
    partition = generate_partition(config, rng)
    adjacency = build_adjacency_graph(partition)
    room_selection = select_rooms(partition, config, rng)
    candidates = generate_candidate_connections(room_selection, adjacency, config)
    selected_graph = select_room_graph(candidates, config, rng)
    applied_layout = apply_connections(selected_graph, adjacency)
    opening_layout = generate_openings(applied_layout, config, rng)
    wall_layout = generate_walls(opening_layout, adjacency, config)
    occupancy = generate_occupancy_map(wall_layout, config, rng)
    layout_document = build_layout_document(applied_layout, opening_layout, wall_layout)

    return GeneratedWorld(
        config=config,
        partition=partition,
        adjacency=adjacency,
        room_selection=room_selection,
        candidates=candidates,
        selected_graph=selected_graph,
        applied_layout=applied_layout,
        opening_layout=opening_layout,
        wall_layout=wall_layout,
        occupancy=occupancy,
        layout_document=layout_document,
        attempt=attempt,
    )


def _validate_output_tree(out_dir: Path) -> None:
    for relative_path in REQUIRED_OUTPUTS:
        path = out_dir / relative_path
        if not path.is_file():
            raise WorldValidationError(f"Missing required output: {path}")

    debug_dir = out_dir / "debug"
    for stage in REQUIRED_DEBUG_STAGES:
        png_path = debug_dir / f"{stage}.png"
        if stage == "09_occupancy_map_preview":
            if not png_path.is_file():
                raise WorldValidationError(f"Missing debug output: {png_path}")
            continue
        svg_path = debug_dir / f"{stage}.svg"
        if not png_path.is_file() or not svg_path.is_file():
            raise WorldValidationError(
                f"Missing debug output: {png_path} or {svg_path}"
            )

from __future__ import annotations

from pathlib import Path

import pytest

from random_gazebo_world.adjacency import build_adjacency_graph
from random_gazebo_world.config import Config
from random_gazebo_world.geometry import Cell
from random_gazebo_world.openings import OpeningLayout, generate_openings
from random_gazebo_world.partition import Partition, generate_partition
from random_gazebo_world.rng import create_seeded_rng
from random_gazebo_world.topology import (
    CandidateConnections,
    CellRole,
    RoomSelection,
    SelectedRoomGraph,
    apply_connections,
    generate_candidate_connections,
    select_room_graph,
)
from random_gazebo_world.visualize import render_wall_segments
from random_gazebo_world.walls import (
    WallSegment,
    _should_generate_interior_wall,
    generate_walls,
    validate_wall_layout,
    wall_segment_line,
)


def _sample_config(**overrides: float | int) -> Config:
    values = {
        "world_width": 20.0,
        "world_height": 20.0,
        "min_cell_size": 2.0,
        "max_cell_size": 6.0,
        "min_room_count": 3,
        "max_room_count": 8,
        "wall_height": 2.5,
        "wall_thickness": 0.15,
        "gate_width_min": 0.8,
        "gate_width_max": 1.2,
        "passage_width_min": 0.8,
        "passage_width_max": 1.2,
        "extra_loop_probability": 0.0,
        "map_resolution": 0.05,
        "random_seed": 42,
    }
    values.update(overrides)
    config = Config(**values)  # type: ignore[arg-type]
    config.validate()
    return config


def _grid_partition() -> Partition:
    cells = (
        Cell.from_origin_size(0, 0.0, 0.0, 5.0, 5.0),
        Cell.from_origin_size(1, 5.0, 0.0, 5.0, 5.0),
        Cell.from_origin_size(2, 0.0, 5.0, 5.0, 5.0),
        Cell.from_origin_size(3, 5.0, 5.0, 5.0, 5.0),
    )
    return Partition(cells=cells, world_width=10.0, world_height=10.0)


def _build_wall_layout(room_ids: set[int], config: Config, seed: int):
    partition = _grid_partition()
    adjacency = build_adjacency_graph(partition)
    selection = RoomSelection(partition=partition, room_cell_ids=frozenset(room_ids))
    candidates = generate_candidate_connections(selection, adjacency, config)
    selected = select_room_graph(candidates, config, create_seeded_rng(seed))
    applied = apply_connections(selected, adjacency)
    opening_layout = generate_openings(applied, config, create_seeded_rng(seed + 1000))
    wall_layout = generate_walls(opening_layout, adjacency, config)
    return partition, opening_layout, wall_layout


def test_wall_segments_do_not_overlap_openings() -> None:
    config = _sample_config()
    _, opening_layout, wall_layout = _build_wall_layout({0, 1, 3}, config, 42)
    validate_wall_layout(wall_layout, config)

    for opening in opening_layout.openings:
        for segment in wall_layout.segments:
            if segment.orientation != opening.shared_wall.orientation:
                continue
            if abs(segment.fixed_coord - opening.shared_wall.fixed_coord) > 1e-9:
                continue
            overlap_start = max(segment.span_start, opening.span_start)
            overlap_end = min(segment.span_end, opening.span_end)
            assert overlap_end - overlap_start <= 1e-9


def test_passage_passage_boundary_skips_interior_wall() -> None:
    assert not _should_generate_interior_wall(CellRole.PASSAGE, CellRole.PASSAGE)
    assert _should_generate_interior_wall(CellRole.ROOM, CellRole.ROOM)
    assert not _should_generate_interior_wall(CellRole.ROOM, CellRole.UNUSED)
    assert not _should_generate_interior_wall(CellRole.UNUSED, CellRole.UNUSED)
    assert _should_generate_interior_wall(CellRole.ROOM, CellRole.PASSAGE)


def test_unused_cells_produce_solid_fill_not_walls() -> None:
    config = _sample_config(min_room_count=2, max_room_count=2)
    partition = _grid_partition()
    adjacency = build_adjacency_graph(partition)
    selection = RoomSelection(partition=partition, room_cell_ids=frozenset({0, 1}))
    candidates = CandidateConnections(room_selection=selection, connections=())
    selected = SelectedRoomGraph(
        candidates=candidates,
        connections=(),
        spanning_tree_connections=(),
        loop_connections=(),
    )
    applied = apply_connections(selected, adjacency)
    opening_layout = OpeningLayout(applied_layout=applied, openings=())
    wall_layout = generate_walls(opening_layout, adjacency, config)

    unused_ids = {
        cell.id
        for cell in partition.cells
        if applied.role_for(cell.id) is CellRole.UNUSED
    }
    assert unused_ids == {2, 3}
    assert len(wall_layout.unused_solids) == 2
    for rect in wall_layout.unused_solids:
        assert rect.width > 0
        assert rect.height > 0


def test_gate_opening_splits_shared_wall() -> None:
    config = _sample_config()
    _, opening_layout, wall_layout = _build_wall_layout({0, 1}, config, 1)
    gate = next(opening for opening in opening_layout.openings if opening.kind == "gate")
    shared_wall = gate.shared_wall

    matching = [
        segment
        for segment in wall_layout.segments
        if segment.orientation == shared_wall.orientation
        and abs(segment.fixed_coord - shared_wall.fixed_coord) <= 1e-9
        and segment.span_start >= shared_wall.span_start - 1e-9
        and segment.span_end <= shared_wall.span_end + 1e-9
    ]
    assert len(matching) == 2
    total_length = sum(segment.length for segment in matching)
    assert total_length == pytest.approx(shared_wall.length - gate.width, rel=1e-6)


def test_all_wall_segments_meet_minimum_length() -> None:
    config = _sample_config()
    _, _, wall_layout = _build_wall_layout({0, 1, 3}, config, 7)
    for segment in wall_layout.segments:
        assert segment.length + 1e-9 >= config.wall_thickness


def test_generated_world_wall_layout_validates() -> None:
    config = _sample_config()
    partition = generate_partition(config, create_seeded_rng(42))
    adjacency = build_adjacency_graph(partition)
    selection = RoomSelection(
        partition=partition,
        room_cell_ids=frozenset(cell.id for cell in partition.cells[: config.min_room_count]),
    )
    candidates = generate_candidate_connections(selection, adjacency, config)
    selected = select_room_graph(candidates, config, create_seeded_rng(99))
    applied = apply_connections(selected, adjacency)
    opening_layout = generate_openings(applied, config, create_seeded_rng(1001))
    wall_layout = generate_walls(opening_layout, adjacency, config)
    validate_wall_layout(wall_layout, config)


def test_render_wall_segments_writes_svg_and_png(tmp_path: Path) -> None:
    config = _sample_config()
    _, _, wall_layout = _build_wall_layout({0, 1, 3}, config, 42)
    output_base = tmp_path / "08_wall_segments"
    render_wall_segments(wall_layout, output_base)

    assert output_base.with_suffix(".png").is_file()
    assert output_base.with_suffix(".svg").is_file()
    assert output_base.with_suffix(".png").stat().st_size > 0
    assert output_base.with_suffix(".svg").stat().st_size > 0


def test_wall_segment_line_matches_orientation() -> None:
    segment = WallSegment("vertical", 1.0, 0.0, 2.0)
    assert wall_segment_line(segment) == ((1.0, 0.0), (1.0, 2.0))

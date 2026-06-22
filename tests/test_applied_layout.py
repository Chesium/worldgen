from __future__ import annotations

from pathlib import Path

import pytest

from random_gazebo_world.adjacency import build_adjacency_graph
from random_gazebo_world.config import Config
from random_gazebo_world.geometry import Cell
from random_gazebo_world.partition import Partition, generate_partition
from random_gazebo_world.rng import create_seeded_rng
from random_gazebo_world.topology import (
    CellRole,
    ConnectionType,
    RoomSelection,
    apply_connections,
    generate_candidate_connections,
    select_room_graph,
    validate_applied_layout,
)
from random_gazebo_world.visualize import render_passage_cells


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


def _build_selected_graph(room_ids: set[int], config: Config, seed: int):
    partition = _grid_partition()
    adjacency = build_adjacency_graph(partition)
    selection = RoomSelection(partition=partition, room_cell_ids=frozenset(room_ids))
    candidates = generate_candidate_connections(selection, adjacency, config)
    selected = select_room_graph(candidates, config, create_seeded_rng(seed))
    return partition, adjacency, selected


def test_passage_marks_intermediate_cells() -> None:
    config = _sample_config()
    partition, adjacency, selected = _build_selected_graph({0, 3}, config, 42)
    layout = apply_connections(selected, adjacency)
    validate_applied_layout(layout)

    passage_connections = [
        connection
        for connection in selected.connections
        if connection.connection_type is ConnectionType.PASSAGE
    ]
    assert passage_connections
    for connection in passage_connections:
        for cell_id in connection.path_cell_ids[1:-1]:
            assert layout.role_for(cell_id) is CellRole.PASSAGE
            assert cell_id in layout.passage_cell_ids


def test_no_room_cells_reclassified_as_passage() -> None:
    config = _sample_config()
    _, adjacency, selected = _build_selected_graph({0, 1, 3}, config, 7)
    layout = apply_connections(selected, adjacency)

    for room_id in selected.room_selection.room_cell_ids:
        assert layout.role_for(room_id) is CellRole.ROOM
        assert room_id not in layout.passage_cell_ids


def test_gate_connection_records_logical_opening() -> None:
    config = _sample_config()
    _, adjacency, selected = _build_selected_graph({0, 1}, config, 1)
    layout = apply_connections(selected, adjacency)

    assert selected.connections[0].connection_type is ConnectionType.GATE
    assert layout.passage_cell_ids == frozenset()
    assert len(layout.logical_openings) == 1
    assert layout.logical_openings[0].kind == "gate"


def test_passage_connection_records_step_openings() -> None:
    config = _sample_config()
    _, adjacency, selected = _build_selected_graph({0, 3}, config, 42)
    layout = apply_connections(selected, adjacency)
    passage = next(
        connection
        for connection in selected.connections
        if connection.connection_type is ConnectionType.PASSAGE
    )

    assert len(layout.logical_openings) == len(passage.path_cell_ids) - 1
    assert all(opening.kind == "passage" for opening in layout.logical_openings)


def test_generated_world_applied_layout_validates() -> None:
    config = _sample_config()
    partition = generate_partition(config, create_seeded_rng(42))
    adjacency = build_adjacency_graph(partition)
    selection = RoomSelection(
        partition=partition,
        room_cell_ids=frozenset(cell.id for cell in partition.cells[: config.min_room_count]),
    )
    candidates = generate_candidate_connections(selection, adjacency, config)
    selected = select_room_graph(candidates, config, create_seeded_rng(99))
    layout = apply_connections(selected, adjacency)
    validate_applied_layout(layout)


def test_render_passage_cells_writes_svg_and_png(tmp_path: Path) -> None:
    config = _sample_config()
    partition, adjacency, selected = _build_selected_graph({0, 1, 3}, config, 42)
    layout = apply_connections(selected, adjacency)
    output_base = tmp_path / "06_passage_cells"
    render_passage_cells(layout, output_base)

    assert output_base.with_suffix(".png").is_file()
    assert output_base.with_suffix(".svg").is_file()
    assert output_base.with_suffix(".png").stat().st_size > 0
    assert output_base.with_suffix(".svg").stat().st_size > 0

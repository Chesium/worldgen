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
    generate_candidate_connections,
    validate_candidate_connections,
)
from random_gazebo_world.visualize import render_candidate_connections


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
        "extra_loop_probability": 0.2,
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


def _grid_room_selection(room_ids: set[int]) -> RoomSelection:
    return RoomSelection(partition=_grid_partition(), room_cell_ids=frozenset(room_ids))


def test_adjacent_rooms_generate_gate_candidate() -> None:
    config = _sample_config()
    partition = _grid_partition()
    adjacency = build_adjacency_graph(partition)
    selection = _grid_room_selection({0, 1})

    candidates = generate_candidate_connections(selection, adjacency, config)
    assert len(candidates.connections) == 1
    gate = candidates.connections[0]
    assert gate.connection_type is ConnectionType.GATE
    assert gate.shared_wall is not None
    assert gate.shared_wall.length >= config.gate_width_min


def test_non_adjacent_rooms_generate_passage_candidate() -> None:
    config = _sample_config()
    partition = _grid_partition()
    adjacency = build_adjacency_graph(partition)
    selection = _grid_room_selection({0, 3})

    candidates = generate_candidate_connections(selection, adjacency, config)
    assert len(candidates.connections) == 1
    passage = candidates.connections[0]
    assert passage.connection_type is ConnectionType.PASSAGE
    assert passage.path_cell_ids[0] == 0
    assert passage.path_cell_ids[-1] == 3
    for cell_id in passage.path_cell_ids[1:-1]:
        assert selection.role_for(cell_id) is CellRole.UNUSED


def test_short_shared_wall_is_not_a_gate_candidate() -> None:
    config = _sample_config(gate_width_min=5.5, gate_width_max=6.0)
    partition = _grid_partition()
    adjacency = build_adjacency_graph(partition)
    selection = _grid_room_selection({0, 1})

    candidates = generate_candidate_connections(selection, adjacency, config)
    assert candidates.connections == ()


def test_generated_world_candidate_connections_validate() -> None:
    config = _sample_config()
    partition = generate_partition(config, create_seeded_rng(42))
    adjacency = build_adjacency_graph(partition)
    selection = RoomSelection(
        partition=partition,
        room_cell_ids=frozenset({cell.id for cell in partition.cells[:4]}),
    )
    if selection.room_count < config.min_room_count:
        selection = RoomSelection(
            partition=partition,
            room_cell_ids=frozenset(cell.id for cell in partition.cells[: config.min_room_count]),
        )

    candidates = generate_candidate_connections(selection, adjacency, config)
    validate_candidate_connections(candidates, adjacency, config)


def test_render_candidate_connections_writes_svg_and_png(tmp_path: Path) -> None:
    config = _sample_config()
    partition = _grid_partition()
    adjacency = build_adjacency_graph(partition)
    selection = _grid_room_selection({0, 1, 3})
    candidates = generate_candidate_connections(selection, adjacency, config)
    output_base = tmp_path / "04_candidate_connections"
    render_candidate_connections(partition, selection, candidates, output_base)

    assert output_base.with_suffix(".png").is_file()
    assert output_base.with_suffix(".svg").is_file()
    assert output_base.with_suffix(".png").stat().st_size > 0
    assert output_base.with_suffix(".svg").stat().st_size > 0

from __future__ import annotations

from pathlib import Path

import networkx as nx
import pytest

from random_gazebo_world.adjacency import build_adjacency_graph
from random_gazebo_world.config import Config
from random_gazebo_world.geometry import Cell
from random_gazebo_world.partition import Partition, generate_partition
from random_gazebo_world.rng import create_seeded_rng
from random_gazebo_world.topology import (
    RoomSelection,
    generate_candidate_connections,
    select_room_graph,
    validate_selected_room_graph,
)
from random_gazebo_world.visualize import render_selected_room_graph


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


def _build_grid_selection_and_candidates(
    room_ids: set[int],
    config: Config,
):
    partition = _grid_partition()
    adjacency = build_adjacency_graph(partition)
    selection = RoomSelection(partition=partition, room_cell_ids=frozenset(room_ids))
    candidates = generate_candidate_connections(selection, adjacency, config)
    return partition, selection, adjacency, candidates


def test_selected_room_graph_is_connected() -> None:
    config = _sample_config(extra_loop_probability=0.0)
    _, _, adjacency, candidates = _build_grid_selection_and_candidates(
        {0, 1, 2, 3}, config
    )
    selected = select_room_graph(candidates, adjacency, config, create_seeded_rng(42))
    validate_selected_room_graph(selected, config)

    graph = nx.Graph()
    graph.add_nodes_from(sorted(selected.room_selection.room_cell_ids))
    for connection in selected.connections:
        graph.add_edge(connection.room_a_id, connection.room_b_id)
    assert nx.is_connected(graph)


def test_zero_loop_probability_keeps_spanning_tree_only() -> None:
    config = _sample_config(extra_loop_probability=0.0)
    _, _, adjacency, candidates = _build_grid_selection_and_candidates(
        {0, 1, 2, 3}, config
    )
    selected = select_room_graph(candidates, adjacency, config, create_seeded_rng(7))

    assert len(selected.connections) == selected.room_selection.room_count - 1
    assert selected.loop_connections == ()


def test_full_loop_probability_adds_all_remaining_candidates() -> None:
    config = _sample_config(extra_loop_probability=1.0)
    _, _, adjacency, candidates = _build_grid_selection_and_candidates(
        {0, 1, 2, 3}, config
    )
    selected = select_room_graph(candidates, adjacency, config, create_seeded_rng(7))

    assert len(selected.connections) == len(candidates.connections)
    assert len(selected.loop_connections) == (
        len(candidates.connections) - (selected.room_selection.room_count - 1)
    )


def test_same_seed_produces_identical_selected_room_graph() -> None:
    config = _sample_config(extra_loop_probability=0.35)
    _, _, adjacency, candidates = _build_grid_selection_and_candidates(
        {0, 1, 2, 3}, config
    )
    first = select_room_graph(candidates, adjacency, config, create_seeded_rng(123))
    second = select_room_graph(candidates, adjacency, config, create_seeded_rng(123))
    assert first == second


def test_generated_world_selected_room_graph_validates() -> None:
    config = _sample_config(extra_loop_probability=0.25)
    partition = generate_partition(config, create_seeded_rng(42))
    adjacency = build_adjacency_graph(partition)
    selection = RoomSelection(
        partition=partition,
        room_cell_ids=frozenset(cell.id for cell in partition.cells[: config.min_room_count]),
    )
    candidates = generate_candidate_connections(selection, adjacency, config)
    selected = select_room_graph(candidates, adjacency, config, create_seeded_rng(99))
    validate_selected_room_graph(selected, config)


def test_render_selected_room_graph_writes_svg_and_png(tmp_path: Path) -> None:
    config = _sample_config(extra_loop_probability=0.5)
    partition, _, adjacency, candidates = _build_grid_selection_and_candidates(
        {0, 1, 2, 3}, config
    )
    selected = select_room_graph(candidates, adjacency, config, create_seeded_rng(42))
    output_base = tmp_path / "05_selected_room_graph"
    render_selected_room_graph(partition, selected, output_base)

    assert output_base.with_suffix(".png").is_file()
    assert output_base.with_suffix(".svg").is_file()
    assert output_base.with_suffix(".png").stat().st_size > 0
    assert output_base.with_suffix(".svg").stat().st_size > 0

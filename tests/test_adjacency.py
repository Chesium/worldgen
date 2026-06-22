from __future__ import annotations

from pathlib import Path

import networkx as nx
import pytest

from random_gazebo_world.adjacency import (
    build_adjacency_graph,
    validate_adjacency_graph,
)
from random_gazebo_world.config import Config
from random_gazebo_world.geometry import Cell, are_adjacent, get_shared_wall
from random_gazebo_world.partition import Partition, generate_partition
from random_gazebo_world.rng import create_seeded_rng
from random_gazebo_world.visualize import render_adjacency_graph


def _grid_partition() -> Partition:
    cells = (
        Cell.from_origin_size(0, 0.0, 0.0, 5.0, 5.0),
        Cell.from_origin_size(1, 5.0, 0.0, 5.0, 5.0),
        Cell.from_origin_size(2, 0.0, 5.0, 5.0, 5.0),
        Cell.from_origin_size(3, 5.0, 5.0, 5.0, 5.0),
    )
    return Partition(cells=cells, world_width=10.0, world_height=10.0)


def test_adjacency_edges_match_shared_walls() -> None:
    partition = _grid_partition()
    adjacency = build_adjacency_graph(partition)

    assert len(adjacency.edges) == 4
    for edge in adjacency.edges:
        cell_a = adjacency.cell_by_id(edge.cell_a_id)
        cell_b = adjacency.cell_by_id(edge.cell_b_id)
        assert are_adjacent(cell_a, cell_b)
        assert edge.shared_wall == get_shared_wall(cell_a, cell_b)
        assert edge.overlap_length == pytest.approx(edge.shared_wall.length)
        assert edge.overlap_length > 0.0


def test_adjacency_graph_is_connected_for_grid() -> None:
    partition = _grid_partition()
    adjacency = build_adjacency_graph(partition)
    assert nx.is_connected(adjacency.graph)


def test_generated_partition_adjacency_is_connected() -> None:
    config = Config(
        world_width=20.0,
        world_height=20.0,
        min_cell_size=2.0,
        max_cell_size=6.0,
        min_room_count=3,
        max_room_count=8,
        wall_height=2.5,
        wall_thickness=0.15,
        gate_width_min=0.8,
        gate_width_max=1.2,
        passage_width_min=0.8,
        passage_width_max=1.2,
        extra_loop_probability=0.2,
        map_resolution=0.05,
        random_seed=42,
    )
    partition = generate_partition(config, create_seeded_rng(42))
    adjacency = build_adjacency_graph(partition)
    validate_adjacency_graph(adjacency, partition)
    assert nx.is_connected(adjacency.graph)


def test_render_adjacency_graph_writes_svg_and_png(tmp_path: Path) -> None:
    partition = _grid_partition()
    adjacency = build_adjacency_graph(partition)
    output_base = tmp_path / "03_cell_adjacency_graph"
    render_adjacency_graph(partition, adjacency, output_base)

    assert output_base.with_suffix(".png").is_file()
    assert output_base.with_suffix(".svg").is_file()
    assert output_base.with_suffix(".png").stat().st_size > 0
    assert output_base.with_suffix(".svg").stat().st_size > 0

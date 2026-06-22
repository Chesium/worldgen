from __future__ import annotations

from pathlib import Path

import pytest

from random_gazebo_world.adjacency import build_adjacency_graph
from random_gazebo_world.config import Config
from random_gazebo_world.geometry import Cell, SharedWall
from random_gazebo_world.openings import (
    LogicalOpening,
    OpeningError,
    generate_openings,
    place_opening,
    validate_openings,
)
from random_gazebo_world.partition import Partition, generate_partition
from random_gazebo_world.rng import create_seeded_rng
from random_gazebo_world.topology import (
    RoomSelection,
    apply_connections,
    generate_candidate_connections,
    select_room_graph,
)
from random_gazebo_world.visualize import render_openings


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


def _build_opening_layout(room_ids: set[int], config: Config, seed: int):
    partition = _grid_partition()
    adjacency = build_adjacency_graph(partition)
    selection = RoomSelection(partition=partition, room_cell_ids=frozenset(room_ids))
    candidates = generate_candidate_connections(selection, adjacency, config)
    selected = select_room_graph(candidates, adjacency, config, create_seeded_rng(seed))
    applied = apply_connections(selected, adjacency)
    opening_layout = generate_openings(applied, config, create_seeded_rng(seed + 1000))
    return partition, opening_layout


def test_gate_opening_respects_width_range() -> None:
    config = _sample_config()
    _, opening_layout = _build_opening_layout({0, 1}, config, 42)
    gate_openings = [opening for opening in opening_layout.openings if opening.kind == "gate"]
    assert len(gate_openings) == 1

    opening = gate_openings[0]
    assert config.gate_width_min <= opening.width <= config.gate_width_max + 1e-9
    assert opening.span_start >= -1e-9
    assert opening.span_end <= opening.shared_wall.length + 1e-9
    assert opening.span_end - opening.span_start == pytest.approx(opening.width)


def test_passage_opening_respects_width_range() -> None:
    config = _sample_config()
    _, opening_layout = _build_opening_layout({0, 3}, config, 42)
    passage_openings = [
        opening for opening in opening_layout.openings if opening.kind == "passage"
    ]
    assert passage_openings

    for opening in passage_openings:
        assert config.passage_width_min <= opening.width <= config.passage_width_max + 1e-9
        assert opening.span_start >= -1e-9
        assert opening.span_end <= opening.shared_wall.length + 1e-9


def test_opening_rejects_wall_shorter_than_minimum_width() -> None:
    wall = SharedWall(
        orientation="vertical",
        fixed_coord=5.0,
        span_start=0.0,
        span_end=0.5,
    )
    logical = LogicalOpening(cell_a_id=0, cell_b_id=1, shared_wall=wall, kind="gate")
    with pytest.raises(OpeningError, match="shorter than minimum width"):
        place_opening(logical, 0.8, 1.2, create_seeded_rng(1))


def test_same_seed_produces_identical_openings() -> None:
    config = _sample_config()
    partition = _grid_partition()
    adjacency = build_adjacency_graph(partition)
    selection = RoomSelection(partition=partition, room_cell_ids=frozenset({0, 1, 3}))
    candidates = generate_candidate_connections(selection, adjacency, config)
    selected = select_room_graph(candidates, adjacency, config, create_seeded_rng(7))
    applied = apply_connections(selected, adjacency)

    first = generate_openings(applied, config, create_seeded_rng(500))
    second = generate_openings(applied, config, create_seeded_rng(500))
    assert first == second


def test_generated_world_openings_validate() -> None:
    config = _sample_config()
    partition = generate_partition(config, create_seeded_rng(42))
    adjacency = build_adjacency_graph(partition)
    selection = RoomSelection(
        partition=partition,
        room_cell_ids=frozenset(cell.id for cell in partition.cells[: config.min_room_count]),
    )
    candidates = generate_candidate_connections(selection, adjacency, config)
    selected = select_room_graph(candidates, adjacency, config, create_seeded_rng(99))
    applied = apply_connections(selected, adjacency)
    opening_layout = generate_openings(applied, config, create_seeded_rng(1001))
    validate_openings(opening_layout, config)


def test_render_openings_writes_svg_and_png(tmp_path: Path) -> None:
    config = _sample_config()
    _, opening_layout = _build_opening_layout({0, 1, 3}, config, 42)
    output_base = tmp_path / "07_openings"
    render_openings(opening_layout, output_base)

    assert output_base.with_suffix(".png").is_file()
    assert output_base.with_suffix(".svg").is_file()
    assert output_base.with_suffix(".png").stat().st_size > 0
    assert output_base.with_suffix(".svg").stat().st_size > 0

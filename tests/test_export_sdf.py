from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from random_gazebo_world.adjacency import build_adjacency_graph
from random_gazebo_world.config import Config
from random_gazebo_world.export_sdf import (
    export_world_sdf,
    validate_world_sdf,
    wall_segment_to_box,
)
from random_gazebo_world.geometry import Cell
from random_gazebo_world.openings import generate_openings
from random_gazebo_world.partition import Partition, generate_partition
from random_gazebo_world.rng import create_seeded_rng
from random_gazebo_world.topology import (
    RoomSelection,
    apply_connections,
    generate_candidate_connections,
    select_room_graph,
)
from random_gazebo_world.walls import WallSegment, generate_walls


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
    return generate_walls(opening_layout, adjacency, config)


def test_wall_segment_to_box_vertical() -> None:
    segment = WallSegment("vertical", 5.0, 0.0, 4.0)
    box = wall_segment_to_box(segment, wall_height=2.5, wall_thickness=0.15, index=0)
    assert box.center_x == pytest.approx(5.0)
    assert box.center_y == pytest.approx(2.0)
    assert box.center_z == pytest.approx(1.25)
    assert box.size_x == pytest.approx(0.15)
    assert box.size_y == pytest.approx(4.0)
    assert box.size_z == pytest.approx(2.5)


def test_wall_segment_to_box_horizontal() -> None:
    segment = WallSegment("horizontal", 3.0, 1.0, 6.0)
    box = wall_segment_to_box(segment, wall_height=2.0, wall_thickness=0.2, index=1)
    assert box.center_x == pytest.approx(3.5)
    assert box.center_y == pytest.approx(3.0)
    assert box.size_x == pytest.approx(5.0)
    assert box.size_y == pytest.approx(0.2)
    assert box.size_z == pytest.approx(2.0)


def test_export_world_sdf_is_well_formed_xml(tmp_path: Path) -> None:
    config = _sample_config()
    wall_layout = _build_wall_layout({0, 1}, config, 1)
    sdf_path = tmp_path / "world.sdf"
    export_world_sdf(wall_layout, config, sdf_path)

    tree = ET.parse(sdf_path)
    root = tree.getroot()
    assert root.tag == "sdf"
    assert root.find("world/model") is not None
    validate_world_sdf(sdf_path, wall_layout, config)


def test_export_world_sdf_matches_all_wall_segments(tmp_path: Path) -> None:
    config = _sample_config()
    wall_layout = _build_wall_layout({0, 1, 3}, config, 42)
    sdf_path = tmp_path / "world.sdf"
    export_world_sdf(wall_layout, config, sdf_path)

    link = ET.parse(sdf_path).getroot().find("world/model/link")
    assert link is not None
    expected = len(wall_layout.segments) + len(wall_layout.unused_solids)
    if wall_layout.passage_geometry is not None:
        expected += len(wall_layout.passage_geometry.solids)
    assert len(link.findall("collision")) == expected
    assert len(link.findall("visual")) == expected


def test_generated_world_sdf_exports(tmp_path: Path) -> None:
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
    sdf_path = tmp_path / "world.sdf"
    export_world_sdf(wall_layout, config, sdf_path)
    validate_world_sdf(sdf_path, wall_layout, config)

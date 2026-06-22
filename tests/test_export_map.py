from __future__ import annotations

from pathlib import Path

import yaml

from random_gazebo_world.adjacency import build_adjacency_graph
from random_gazebo_world.config import Config
from random_gazebo_world.export_map import (
    export_occupancy_map,
    generate_occupancy_map,
    validate_occupancy_map,
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
from random_gazebo_world.walls import generate_walls


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
    selected = select_room_graph(candidates, adjacency, config, create_seeded_rng(seed))
    applied = apply_connections(selected, adjacency)
    opening_layout = generate_openings(applied, config, create_seeded_rng(seed + 1000))
    return generate_walls(opening_layout, adjacency, config)


def test_occupancy_map_free_space_is_connected() -> None:
    config = _sample_config()
    wall_layout = _build_wall_layout({0, 1, 3}, config, 42)
    occupancy = generate_occupancy_map(wall_layout, config, create_seeded_rng(7))
    validate_occupancy_map(occupancy)


def test_sampled_start_goal_are_reachable() -> None:
    config = _sample_config()
    wall_layout = _build_wall_layout({0, 1, 3}, config, 42)
    occupancy = generate_occupancy_map(wall_layout, config, create_seeded_rng(11))
    assert occupancy.data[occupancy.start_cell] == 254
    assert occupancy.data[occupancy.goal_cell] == 254
    validate_occupancy_map(occupancy)


def test_export_writes_map_png_yaml_and_preview(tmp_path: Path) -> None:
    config = _sample_config()
    wall_layout = _build_wall_layout({0, 1, 3}, config, 42)
    occupancy = export_occupancy_map(
        wall_layout,
        config,
        tmp_path,
        create_seeded_rng(99),
    )

    assert (tmp_path / "map.png").is_file()
    assert (tmp_path / "map.yaml").is_file()
    assert (tmp_path / "debug" / "09_occupancy_map_preview.png").is_file()
    assert occupancy.resolution == config.map_resolution


def test_map_yaml_matches_geometry(tmp_path: Path) -> None:
    config = _sample_config(map_resolution=0.1)
    wall_layout = _build_wall_layout({0, 1}, config, 1)
    export_occupancy_map(wall_layout, config, tmp_path, create_seeded_rng(3))

    with (tmp_path / "map.yaml").open(encoding="utf-8") as handle:
        payload = yaml.safe_load(handle)

    assert payload["resolution"] == 0.1
    assert payload["origin"] == [0.0, 0.0, 0.0]
    assert payload["image"] == "map.png"


def test_generated_world_occupancy_map_validates() -> None:
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
    wall_layout = generate_walls(opening_layout, adjacency, config)
    occupancy = generate_occupancy_map(wall_layout, config, create_seeded_rng(2000))
    validate_occupancy_map(occupancy)

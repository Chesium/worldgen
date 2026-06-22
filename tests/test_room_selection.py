from __future__ import annotations

from pathlib import Path

import pytest

from random_gazebo_world.config import Config
from random_gazebo_world.geometry import Cell
from random_gazebo_world.partition import Partition
from random_gazebo_world.rng import create_seeded_rng
from random_gazebo_world.topology import (
    CellRole,
    RoomSelectionError,
    select_rooms,
    validate_room_selection,
)
from random_gazebo_world.visualize import render_selected_rooms


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


def test_room_count_within_config_bounds() -> None:
    config = _sample_config(min_room_count=2, max_room_count=3)
    partition = _grid_partition()
    selection = select_rooms(partition, config, create_seeded_rng(42))
    validate_room_selection(selection, config)
    assert 2 <= selection.room_count <= 3


def test_same_seed_produces_identical_room_selection() -> None:
    config = _sample_config(min_room_count=2, max_room_count=4)
    partition = _grid_partition()
    first = select_rooms(partition, config, create_seeded_rng(99))
    second = select_rooms(partition, config, create_seeded_rng(99))
    assert first == second


def test_room_and_unused_roles_partition_all_cells() -> None:
    config = _sample_config(min_room_count=2, max_room_count=2)
    partition = _grid_partition()
    selection = select_rooms(partition, config, create_seeded_rng(7))

    roles = {cell.id: selection.role_for(cell.id) for cell in partition.cells}
    assert sum(role is CellRole.ROOM for role in roles.values()) == 2
    assert sum(role is CellRole.UNUSED for role in roles.values()) == 2


def test_too_few_cells_raises_clear_error() -> None:
    config = _sample_config(min_room_count=5, max_room_count=8)
    partition = _grid_partition()
    with pytest.raises(RoomSelectionError, match="Partition has 4 cells"):
        select_rooms(partition, config, create_seeded_rng(1))


def test_render_selected_rooms_writes_svg_and_png(tmp_path: Path) -> None:
    config = _sample_config(min_room_count=2, max_room_count=3)
    partition = _grid_partition()
    selection = select_rooms(partition, config, create_seeded_rng(42))
    output_base = tmp_path / "02_selected_rooms"
    render_selected_rooms(partition, selection, output_base)

    assert output_base.with_suffix(".png").is_file()
    assert output_base.with_suffix(".svg").is_file()
    assert output_base.with_suffix(".png").stat().st_size > 0
    assert output_base.with_suffix(".svg").stat().st_size > 0

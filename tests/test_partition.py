from __future__ import annotations

from pathlib import Path

import pytest

from random_gazebo_world.config import Config
from random_gazebo_world.geometry import rectangles_intersect
from random_gazebo_world.partition import Partition, generate_partition, validate_partition
from random_gazebo_world.rng import create_seeded_rng
from random_gazebo_world.visualize import render_partition


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


def test_partition_cells_respect_size_constraints() -> None:
    config = _sample_config()
    partition = generate_partition(config, create_seeded_rng(42))
    validate_partition(partition, config)


def test_partition_tiles_world_without_overlaps() -> None:
    config = _sample_config()
    partition = generate_partition(config, create_seeded_rng(7))

    total_area = sum(cell.width * cell.height for cell in partition.cells)
    assert total_area == pytest.approx(config.world_width * config.world_height)

    for left_index, left in enumerate(partition.cells):
        for right in partition.cells[left_index + 1 :]:
            assert not rectangles_intersect(left, right)


def test_same_seed_produces_identical_partition() -> None:
    config = _sample_config()
    first = generate_partition(config, create_seeded_rng(123))
    second = generate_partition(config, create_seeded_rng(123))
    assert first == second


def test_partition_assigns_stable_cell_ids() -> None:
    config = _sample_config()
    partition = generate_partition(config, create_seeded_rng(42))
    ids = [cell.id for cell in partition.cells]
    assert ids == list(range(len(partition.cells)))


def test_render_partition_writes_svg_and_png(tmp_path: Path) -> None:
    config = _sample_config(world_width=12.0, world_height=12.0)
    partition = generate_partition(config, create_seeded_rng(42))
    output_base = tmp_path / "01_partition"
    render_partition(partition, output_base)

    assert output_base.with_suffix(".png").is_file()
    assert output_base.with_suffix(".svg").is_file()
    assert output_base.with_suffix(".png").stat().st_size > 0
    assert output_base.with_suffix(".svg").stat().st_size > 0

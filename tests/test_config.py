from __future__ import annotations

from pathlib import Path

import pytest

from random_gazebo_world.config import Config, ConfigError, load_config
from random_gazebo_world.rng import create_seeded_rng


def test_load_default_config() -> None:
    config = load_config(Path("configs/default.yaml"))
    assert config.world_width == 20.0
    assert config.random_seed == 42


def test_invalid_config_raises_clear_error(tmp_path: Path) -> None:
    bad_config = tmp_path / "bad.yaml"
    bad_config.write_text(
        "\n".join(
            [
                "world_width: 20.0",
                "world_height: 20.0",
                "min_cell_size: 8.0",
                "max_cell_size: 4.0",
                "min_room_count: 3",
                "max_room_count: 8",
                "wall_height: 2.5",
                "wall_thickness: 0.15",
                "gate_width_min: 0.8",
                "gate_width_max: 1.2",
                "passage_width_min: 0.8",
                "passage_width_max: 1.2",
                "extra_loop_probability: 0.2",
                "map_resolution: 0.05",
                "random_seed: 42",
            ]
        ),
        encoding="utf-8",
    )

    with pytest.raises(ConfigError, match="min cell size"):
        load_config(bad_config)


def test_missing_field_raises_clear_error(tmp_path: Path) -> None:
    bad_config = tmp_path / "missing.yaml"
    bad_config.write_text("world_width: 20.0\n", encoding="utf-8")

    with pytest.raises(ConfigError, match="Missing required config field"):
        load_config(bad_config)


def test_same_seed_produces_identical_rng_state() -> None:
    first = [create_seeded_rng(42).random() for _ in range(5)]
    second = [create_seeded_rng(42).random() for _ in range(5)]
    assert first == second


def test_with_seed_override() -> None:
    config = load_config(Path("configs/default.yaml"))
    updated = config.with_seed(99)
    assert updated.random_seed == 99
    assert config.random_seed == 42

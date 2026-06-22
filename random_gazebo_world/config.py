from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import yaml


class ConfigError(ValueError):
    """Raised when a config file is invalid or fails validation."""


@dataclass(frozen=True)
class Config:
    world_width: float
    world_height: float
    min_cell_size: float
    max_cell_size: float
    min_room_count: int
    max_room_count: int
    wall_height: float
    wall_thickness: float
    gate_width_min: float
    gate_width_max: float
    passage_width_min: float
    passage_width_max: float
    extra_loop_probability: float
    map_resolution: float
    random_seed: int

    def validate(self) -> None:
        _require_positive(self.world_width, "world_width")
        _require_positive(self.world_height, "world_height")
        _require_positive(self.min_cell_size, "min_cell_size")
        _require_positive(self.max_cell_size, "max_cell_size")
        _require_min_max(self.min_cell_size, self.max_cell_size, "cell size")
        _require_positive_int(self.min_room_count, "min_room_count")
        _require_positive_int(self.max_room_count, "max_room_count")
        _require_min_max(self.min_room_count, self.max_room_count, "room count")
        _require_positive(self.wall_height, "wall_height")
        _require_positive(self.wall_thickness, "wall_thickness")
        _require_positive(self.gate_width_min, "gate_width_min")
        _require_positive(self.gate_width_max, "gate_width_max")
        _require_min_max(self.gate_width_min, self.gate_width_max, "gate width")
        _require_positive(self.passage_width_min, "passage_width_min")
        _require_positive(self.passage_width_max, "passage_width_max")
        _require_min_max(
            self.passage_width_min, self.passage_width_max, "passage width"
        )
        _require_probability(self.extra_loop_probability, "extra_loop_probability")
        _require_positive(self.map_resolution, "map_resolution")

    def with_seed(self, seed: int) -> Config:
        return replace(self, random_seed=seed)


def load_config(path: Path | str) -> Config:
    config_path = Path(path)
    if not config_path.is_file():
        raise ConfigError(f"Config file not found: {config_path}")

    with config_path.open(encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)

    if not isinstance(raw, dict):
        raise ConfigError(f"Config root must be a mapping: {config_path}")

    try:
        config = Config(
            world_width=_require_field(raw, "world_width"),
            world_height=_require_field(raw, "world_height"),
            min_cell_size=_require_field(raw, "min_cell_size"),
            max_cell_size=_require_field(raw, "max_cell_size"),
            min_room_count=_require_field(raw, "min_room_count"),
            max_room_count=_require_field(raw, "max_room_count"),
            wall_height=_require_field(raw, "wall_height"),
            wall_thickness=_require_field(raw, "wall_thickness"),
            gate_width_min=_require_field(raw, "gate_width_min"),
            gate_width_max=_require_field(raw, "gate_width_max"),
            passage_width_min=_require_field(raw, "passage_width_min"),
            passage_width_max=_require_field(raw, "passage_width_max"),
            extra_loop_probability=_require_field(raw, "extra_loop_probability"),
            map_resolution=_require_field(raw, "map_resolution"),
            random_seed=_require_field(raw, "random_seed"),
        )
    except KeyError as exc:
        raise ConfigError(f"Missing required config field: {exc.args[0]}") from exc
    except TypeError as exc:
        raise ConfigError(f"Invalid config value in {config_path}: {exc}") from exc

    config.validate()
    return config


def _require_field(raw: dict[str, Any], name: str) -> Any:
    if name not in raw:
        raise KeyError(name)
    return raw[name]


def _require_positive(value: float, name: str) -> None:
    if value <= 0:
        raise ConfigError(f"{name} must be positive, got {value}")


def _require_positive_int(value: int, name: str) -> None:
    if not isinstance(value, int) or isinstance(value, bool):
        raise ConfigError(f"{name} must be an integer, got {value!r}")
    if value <= 0:
        raise ConfigError(f"{name} must be positive, got {value}")


def _require_min_max(min_value: float, max_value: float, label: str) -> None:
    if min_value > max_value:
        raise ConfigError(
            f"min {label} ({min_value}) must be <= max {label} ({max_value})"
        )


def _require_probability(value: float, name: str) -> None:
    if not 0.0 <= value <= 1.0:
        raise ConfigError(f"{name} must be between 0 and 1, got {value}")

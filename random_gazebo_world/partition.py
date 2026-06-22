from __future__ import annotations

import random
from dataclasses import dataclass

from random_gazebo_world.config import Config
from random_gazebo_world.geometry import EPS, Cell, rectangles_intersect


class PartitionError(RuntimeError):
    """Raised when the world cannot be partitioned under size constraints."""


@dataclass(frozen=True)
class Partition:
    cells: tuple[Cell, ...]
    world_width: float
    world_height: float


def generate_partition(config: Config, rng: random.Random) -> Partition:
    if config.partition_method == "voronoi":
        from random_gazebo_world.voronoi import generate_voronoi_partition

        return generate_voronoi_partition(config, rng)
    return generate_bsp_partition(config, rng)


def generate_bsp_partition(config: Config, rng: random.Random) -> Partition:
    cells: list[Cell] = []
    next_id = 0

    _partition_region(
        x_min=0.0,
        y_min=0.0,
        x_max=config.world_width,
        y_max=config.world_height,
        min_size=config.min_cell_size,
        max_size=config.max_cell_size,
        rng=rng,
        cells=cells,
        next_id=next_id,
    )

    partition = Partition(
        cells=tuple(cells),
        world_width=config.world_width,
        world_height=config.world_height,
    )
    validate_partition(partition, config)
    return partition


def validate_partition(partition: Partition, config: Config) -> None:
    if not partition.cells:
        raise PartitionError("Partition produced no cells")

    total_area = 0.0
    for cell in partition.cells:
        if cell.width < config.min_cell_size - EPS:
            raise PartitionError(
                f"Cell {cell.id} width {cell.width} below min_cell_size"
            )
        if cell.height < config.min_cell_size - EPS:
            raise PartitionError(
                f"Cell {cell.id} height {cell.height} below min_cell_size"
            )
        if cell.width > config.max_cell_size + EPS:
            raise PartitionError(
                f"Cell {cell.id} width {cell.width} above max_cell_size"
            )
        if cell.height > config.max_cell_size + EPS:
            raise PartitionError(
                f"Cell {cell.id} height {cell.height} above max_cell_size"
            )
        if cell.x_min < -EPS or cell.y_min < -EPS:
            raise PartitionError(f"Cell {cell.id} extends below world origin")
        if cell.x_max > partition.world_width + EPS:
            raise PartitionError(f"Cell {cell.id} extends beyond world width")
        if cell.y_max > partition.world_height + EPS:
            raise PartitionError(f"Cell {cell.id} extends beyond world height")
        total_area += cell.width * cell.height

    for left_index, left in enumerate(partition.cells):
        for right in partition.cells[left_index + 1 :]:
            if rectangles_intersect(left, right):
                raise PartitionError(
                    f"Cells {left.id} and {right.id} overlap"
                )

    expected_area = partition.world_width * partition.world_height
    if abs(total_area - expected_area) > EPS:
        raise PartitionError(
            f"Partition area {total_area} does not tile world area {expected_area}"
        )


def _partition_region(
    x_min: float,
    y_min: float,
    x_max: float,
    y_max: float,
    min_size: float,
    max_size: float,
    rng: random.Random,
    cells: list[Cell],
    next_id: int,
) -> int:
    width = x_max - x_min
    height = y_max - y_min

    must_split_vertical = width > max_size + EPS
    must_split_horizontal = height > max_size + EPS

    vertical_bounds = _vertical_split_bounds(x_min, x_max, min_size)
    horizontal_bounds = _horizontal_split_bounds(y_min, y_max, min_size)

    can_split_vertical = vertical_bounds is not None
    can_split_horizontal = horizontal_bounds is not None

    if not must_split_vertical and not must_split_horizontal:
        cells.append(Cell(id=next_id, x_min=x_min, y_min=y_min, x_max=x_max, y_max=y_max))
        return next_id + 1

    split_vertical = _choose_split_axis(
        must_split_vertical=must_split_vertical,
        must_split_horizontal=must_split_horizontal,
        can_split_vertical=can_split_vertical,
        can_split_horizontal=can_split_horizontal,
        width=width,
        height=height,
        max_size=max_size,
        rng=rng,
    )

    if split_vertical:
        if vertical_bounds is None:
            raise PartitionError(
                f"Cannot split vertical region width={width} under size constraints"
            )
        split_x = rng.uniform(vertical_bounds[0], vertical_bounds[1])
        next_id = _partition_region(
            x_min,
            y_min,
            split_x,
            y_max,
            min_size,
            max_size,
            rng,
            cells,
            next_id,
        )
        return _partition_region(
            split_x,
            y_min,
            x_max,
            y_max,
            min_size,
            max_size,
            rng,
            cells,
            next_id,
        )

    if horizontal_bounds is None:
        raise PartitionError(
            f"Cannot split horizontal region height={height} under size constraints"
        )
    split_y = rng.uniform(horizontal_bounds[0], horizontal_bounds[1])
    next_id = _partition_region(
        x_min,
        y_min,
        x_max,
        split_y,
        min_size,
        max_size,
        rng,
        cells,
        next_id,
    )
    return _partition_region(
        x_min,
        split_y,
        x_max,
        y_max,
        min_size,
        max_size,
        rng,
        cells,
        next_id,
    )


def _choose_split_axis(
    must_split_vertical: bool,
    must_split_horizontal: bool,
    can_split_vertical: bool,
    can_split_horizontal: bool,
    width: float,
    height: float,
    max_size: float,
    rng: random.Random,
) -> bool:
    if must_split_vertical and not must_split_horizontal:
        return True
    if must_split_horizontal and not must_split_vertical:
        return False

    candidates: list[bool] = []
    if can_split_vertical:
        candidates.append(True)
    if can_split_horizontal:
        candidates.append(False)

    if not candidates:
        raise PartitionError(
            f"Region {width}x{height} exceeds max cell size but cannot be split"
        )
    if len(candidates) == 1:
        return candidates[0]

    vertical_overflow = max(0.0, width - max_size)
    horizontal_overflow = max(0.0, height - max_size)
    if vertical_overflow > horizontal_overflow + EPS:
        return True
    if horizontal_overflow > vertical_overflow + EPS:
        return False
    return rng.choice(candidates)


def _vertical_split_bounds(
    x_min: float,
    x_max: float,
    min_size: float,
) -> tuple[float, float] | None:
    low = x_min + min_size
    high = x_max - min_size
    if low > high + EPS:
        return None
    return low, high


def _horizontal_split_bounds(
    y_min: float,
    y_max: float,
    min_size: float,
) -> tuple[float, float] | None:
    low = y_min + min_size
    high = y_max - min_size
    if low > high + EPS:
        return None
    return low, high

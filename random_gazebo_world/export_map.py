from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

from random_gazebo_world.config import Config
from random_gazebo_world.geometry import EPS
from random_gazebo_world.topology import CellRole
from random_gazebo_world.walls import WallLayout, WallSegment


class OccupancyMapError(RuntimeError):
    """Raised when occupancy map generation or validation fails."""


FREE_VALUE = 254
OCCUPIED_VALUE = 0


@dataclass(frozen=True)
class OccupancyMap:
    data: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    world_width: float
    world_height: float
    start_cell: tuple[int, int]
    goal_cell: tuple[int, int]

    @property
    def width(self) -> int:
        return int(self.data.shape[1])

    @property
    def height(self) -> int:
        return int(self.data.shape[0])


def export_occupancy_map(
    wall_layout: WallLayout,
    config: Config,
    output_dir: Path,
    rng,
) -> OccupancyMap:
    occupancy = generate_occupancy_map(wall_layout, config, rng)
    write_occupancy_map_files(occupancy, output_dir)
    return occupancy


def write_occupancy_map_files(occupancy: OccupancyMap, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    preview_path = output_dir / "debug" / "09_occupancy_map_preview.png"
    preview_path.parent.mkdir(parents=True, exist_ok=True)

    Image.fromarray(occupancy.data).save(output_dir / "map.png")
    _write_map_yaml(output_dir / "map.yaml", occupancy)
    _write_preview(preview_path, occupancy)
    validate_occupancy_map(occupancy)


def generate_occupancy_map(
    wall_layout: WallLayout,
    config: Config,
    rng,
) -> OccupancyMap:
    layout = wall_layout.opening_layout.applied_layout
    partition = layout.partition
    resolution = config.map_resolution
    origin_x = 0.0
    origin_y = 0.0

    width = max(1, math.ceil(partition.world_width / resolution))
    height = max(1, math.ceil(partition.world_height / resolution))
    grid = np.full((height, width), OCCUPIED_VALUE, dtype=np.uint8)

    for cell in partition.cells:
        role = layout.role_for(cell.id)
        if role not in {CellRole.ROOM, CellRole.PASSAGE}:
            continue
        _mark_rectangle_free(
            grid,
            cell.x_min,
            cell.y_min,
            cell.x_max,
            cell.y_max,
            resolution,
            origin_x,
            origin_y,
        )

    half_thickness = config.wall_thickness / 2.0
    for segment in wall_layout.segments:
        _mark_wall_segment_occupied(
            grid,
            segment,
            half_thickness,
            resolution,
            origin_x,
            origin_y,
        )

    start_cell, goal_cell = _sample_start_goal_cells(
        grid,
        layout,
        resolution,
        origin_x,
        origin_y,
        rng,
    )
    occupancy = OccupancyMap(
        data=grid,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        world_width=partition.world_width,
        world_height=partition.world_height,
        start_cell=start_cell,
        goal_cell=goal_cell,
    )
    validate_occupancy_map(occupancy)
    return occupancy


def validate_occupancy_map(occupancy: OccupancyMap) -> None:
    free_cells = _free_cell_coordinates(occupancy.data)
    if not free_cells:
        raise OccupancyMapError("Occupancy map contains no free space")

    components = _connected_components(free_cells)
    if len(components) != 1:
        raise OccupancyMapError(
            f"Free space has {len(components)} connected components, expected 1"
        )

    if not _is_free(occupancy.data, occupancy.start_cell):
        raise OccupancyMapError("Sampled start lies outside free space")
    if not _is_free(occupancy.data, occupancy.goal_cell):
        raise OccupancyMapError("Sampled goal lies outside free space")
    if not _cells_reachable(occupancy.data, occupancy.start_cell, occupancy.goal_cell):
        raise OccupancyMapError("Sampled start and goal are not reachable")


def _write_map_yaml(path: Path, occupancy: OccupancyMap) -> None:
    payload = {
        "image": "map.png",
        "resolution": occupancy.resolution,
        "origin": [occupancy.origin_x, occupancy.origin_y, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    with path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(payload, handle, sort_keys=False)


def _write_preview(path: Path, occupancy: OccupancyMap) -> None:
    preview = np.stack([occupancy.data, occupancy.data, occupancy.data], axis=-1)
    Image.fromarray(preview).save(path)


def _mark_rectangle_free(
    grid: np.ndarray,
    x_min: float,
    y_min: float,
    x_max: float,
    y_max: float,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> None:
    height, width = grid.shape
    for row in range(height):
        for col in range(width):
            world_x, world_y = _grid_cell_center(
                col, row, resolution, origin_x, origin_y, height
            )
            if x_min - EPS <= world_x <= x_max + EPS and y_min - EPS <= world_y <= y_max + EPS:
                grid[row, col] = FREE_VALUE


def _mark_wall_segment_occupied(
    grid: np.ndarray,
    segment: WallSegment,
    half_thickness: float,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> None:
    height, width = grid.shape
    for row in range(height):
        for col in range(width):
            world_x, world_y = _grid_cell_center(
                col, row, resolution, origin_x, origin_y, height
            )
            if _point_in_wall_segment(world_x, world_y, segment, half_thickness):
                grid[row, col] = OCCUPIED_VALUE


def _point_in_wall_segment(
    world_x: float,
    world_y: float,
    segment: WallSegment,
    half_thickness: float,
) -> bool:
    if segment.orientation == "vertical":
        if abs(world_x - segment.fixed_coord) > half_thickness + EPS:
            return False
        return segment.span_start - EPS <= world_y <= segment.span_end + EPS

    if abs(world_y - segment.fixed_coord) > half_thickness + EPS:
        return False
    return segment.span_start - EPS <= world_x <= segment.span_end + EPS


def _grid_cell_center(
    col: int,
    row: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    height: int,
) -> tuple[float, float]:
    world_x = origin_x + (col + 0.5) * resolution
    world_y = origin_y + (height - 1 - row + 0.5) * resolution
    return world_x, world_y


def _free_cell_coordinates(grid: np.ndarray) -> list[tuple[int, int]]:
    rows, cols = np.where(grid == FREE_VALUE)
    return list(zip(rows.tolist(), cols.tolist(), strict=True))


def _connected_components(cells: list[tuple[int, int]]) -> list[set[tuple[int, int]]]:
    remaining = set(cells)
    components: list[set[tuple[int, int]]] = []

    while remaining:
        start = remaining.pop()
        component = {start}
        queue: deque[tuple[int, int]] = deque([start])

        while queue:
            row, col = queue.popleft()
            for neighbor in (
                (row - 1, col),
                (row + 1, col),
                (row, col - 1),
                (row, col + 1),
            ):
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    component.add(neighbor)
                    queue.append(neighbor)

        components.append(component)

    return components


def _is_free(grid: np.ndarray, cell: tuple[int, int]) -> bool:
    row, col = cell
    if row < 0 or col < 0 or row >= grid.shape[0] or col >= grid.shape[1]:
        return False
    return int(grid[row, col]) == FREE_VALUE


def _cells_reachable(
    grid: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> bool:
    if start == goal:
        return True

    visited = {start}
    queue: deque[tuple[int, int]] = deque([start])

    while queue:
        row, col = queue.popleft()
        for neighbor in (
            (row - 1, col),
            (row + 1, col),
            (row, col - 1),
            (row, col + 1),
        ):
            if neighbor in visited:
                continue
            if not _is_free(grid, neighbor):
                continue
            if neighbor == goal:
                return True
            visited.add(neighbor)
            queue.append(neighbor)

    return False


def _sample_start_goal_cells(
    grid: np.ndarray,
    layout,
    resolution: float,
    origin_x: float,
    origin_y: float,
    rng,
) -> tuple[tuple[int, int], tuple[int, int]]:
    room_ids = sorted(layout.room_selection.room_cell_ids)
    if len(room_ids) < 2:
        free_cells = _free_cell_coordinates(grid)
        if len(free_cells) < 2:
            raise OccupancyMapError("Not enough free cells to sample start and goal")
        return free_cells[0], free_cells[-1]

    first_room = room_ids[0]
    second_room = room_ids[-1]
    first_cells = _free_cells_for_room(
        grid, layout, first_room, resolution, origin_x, origin_y
    )
    second_cells = _free_cells_for_room(
        grid, layout, second_room, resolution, origin_x, origin_y
    )

    if not first_cells or not second_cells:
        raise OccupancyMapError("Could not find free cells in selected rooms")

    start = rng.choice(first_cells)
    goal = rng.choice(second_cells)
    if start == goal and len(second_cells) > 1:
        goal = rng.choice([cell for cell in second_cells if cell != start])
    return start, goal


def _free_cells_for_room(
    grid: np.ndarray,
    layout,
    room_id: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> list[tuple[int, int]]:
    cell = next(item for item in layout.partition.cells if item.id == room_id)
    cells: list[tuple[int, int]] = []
    height, width = grid.shape

    for row in range(height):
        for col in range(width):
            if int(grid[row, col]) != FREE_VALUE:
                continue
            world_x, world_y = _grid_cell_center(
                col, row, resolution, origin_x, origin_y, height
            )
            if (
                cell.x_min - EPS <= world_x <= cell.x_max + EPS
                and cell.y_min - EPS <= world_y <= cell.y_max + EPS
            ):
                cells.append((row, col))

    return cells

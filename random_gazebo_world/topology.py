from __future__ import annotations

import random
from dataclasses import dataclass
from enum import Enum

from random_gazebo_world.config import Config
from random_gazebo_world.geometry import Cell
from random_gazebo_world.partition import Partition


class RoomSelectionError(RuntimeError):
    """Raised when room selection cannot satisfy config constraints."""


class CellRole(str, Enum):
    ROOM = "room"
    UNUSED = "unused"


@dataclass(frozen=True)
class RoomSelection:
    partition: Partition
    room_cell_ids: frozenset[int]

    @property
    def room_count(self) -> int:
        return len(self.room_cell_ids)

    def role_for(self, cell_id: int) -> CellRole:
        if cell_id in self.room_cell_ids:
            return CellRole.ROOM
        return CellRole.UNUSED

    def room_cells(self) -> tuple[Cell, ...]:
        return tuple(
            cell for cell in self.partition.cells if cell.id in self.room_cell_ids
        )

    def unused_cells(self) -> tuple[Cell, ...]:
        return tuple(
            cell for cell in self.partition.cells if cell.id not in self.room_cell_ids
        )


def select_rooms(
    partition: Partition,
    config: Config,
    rng: random.Random,
) -> RoomSelection:
    available = len(partition.cells)
    if available < config.min_room_count:
        raise RoomSelectionError(
            f"Partition has {available} cells but min_room_count is "
            f"{config.min_room_count}"
        )

    max_rooms = min(config.max_room_count, available)
    room_count = rng.randint(config.min_room_count, max_rooms)
    cell_ids = [cell.id for cell in partition.cells]
    selected_ids = frozenset(rng.sample(cell_ids, room_count))

    selection = RoomSelection(partition=partition, room_cell_ids=selected_ids)
    validate_room_selection(selection, config)
    return selection


def validate_room_selection(selection: RoomSelection, config: Config) -> None:
    if not config.min_room_count <= selection.room_count <= config.max_room_count:
        raise RoomSelectionError(
            f"Selected {selection.room_count} rooms, expected "
            f"[{config.min_room_count}, {config.max_room_count}]"
        )

    partition_ids = {cell.id for cell in selection.partition.cells}
    unknown = selection.room_cell_ids - partition_ids
    if unknown:
        raise RoomSelectionError(f"Selected unknown cell ids: {sorted(unknown)}")

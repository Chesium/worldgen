from __future__ import annotations

import random
from collections import deque
from dataclasses import dataclass
from enum import Enum

from random_gazebo_world.adjacency import AdjacencyGraph
from random_gazebo_world.config import Config
from random_gazebo_world.geometry import EPS, Cell, SharedWall, are_adjacent, get_shared_wall
from random_gazebo_world.partition import Partition


class RoomSelectionError(RuntimeError):
    """Raised when room selection cannot satisfy config constraints."""


class CandidateConnectionError(RuntimeError):
    """Raised when candidate connections are invalid."""


class CellRole(str, Enum):
    ROOM = "room"
    UNUSED = "unused"


class ConnectionType(str, Enum):
    GATE = "gate"
    PASSAGE = "passage"


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


@dataclass(frozen=True)
class CandidateConnection:
    room_a_id: int
    room_b_id: int
    connection_type: ConnectionType
    shared_wall: SharedWall | None = None
    path_cell_ids: tuple[int, ...] = ()

    def __post_init__(self) -> None:
        if self.room_a_id >= self.room_b_id:
            raise ValueError("room_a_id must be less than room_b_id for stable ordering")


@dataclass(frozen=True)
class CandidateConnections:
    room_selection: RoomSelection
    connections: tuple[CandidateConnection, ...]


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


def generate_candidate_connections(
    room_selection: RoomSelection,
    adjacency: AdjacencyGraph,
    config: Config,
) -> CandidateConnections:
    room_ids = sorted(room_selection.room_cell_ids)
    connections: list[CandidateConnection] = []

    for left_index, room_a_id in enumerate(room_ids):
        for room_b_id in room_ids[left_index + 1 :]:
            if adjacency.graph.has_edge(room_a_id, room_b_id):
                shared_wall = adjacency.graph[room_a_id][room_b_id]["shared_wall"]
                if shared_wall.length + EPS < config.gate_width_min:
                    continue
                connections.append(
                    CandidateConnection(
                        room_a_id=room_a_id,
                        room_b_id=room_b_id,
                        connection_type=ConnectionType.GATE,
                        shared_wall=shared_wall,
                        path_cell_ids=(room_a_id, room_b_id),
                    )
                )
                continue

            path = _shortest_path_through_unused(
                room_a_id, room_b_id, adjacency, room_selection
            )
            if path is not None:
                connections.append(
                    CandidateConnection(
                        room_a_id=room_a_id,
                        room_b_id=room_b_id,
                        connection_type=ConnectionType.PASSAGE,
                        path_cell_ids=path,
                    )
                )

    candidates = CandidateConnections(
        room_selection=room_selection,
        connections=tuple(connections),
    )
    validate_candidate_connections(candidates, adjacency, config)
    return candidates


def validate_candidate_connections(
    candidates: CandidateConnections,
    adjacency: AdjacencyGraph,
    config: Config,
) -> None:
    room_ids = candidates.room_selection.room_cell_ids

    for connection in candidates.connections:
        if connection.room_a_id not in room_ids or connection.room_b_id not in room_ids:
            raise CandidateConnectionError("Connection references non-room cell")

        if connection.connection_type == ConnectionType.GATE:
            if connection.shared_wall is None:
                raise CandidateConnectionError(
                    f"Gate {connection.room_a_id}-{connection.room_b_id} missing wall"
                )
            if connection.shared_wall.length + EPS < config.gate_width_min:
                raise CandidateConnectionError(
                    f"Gate {connection.room_a_id}-{connection.room_b_id} wall too short"
                )
            cell_a = adjacency.cell_by_id(connection.room_a_id)
            cell_b = adjacency.cell_by_id(connection.room_b_id)
            if not are_adjacent(cell_a, cell_b):
                raise CandidateConnectionError(
                    f"Gate {connection.room_a_id}-{connection.room_b_id} not adjacent"
                )
            expected_wall = get_shared_wall(cell_a, cell_b)
            if expected_wall != connection.shared_wall:
                raise CandidateConnectionError(
                    f"Gate {connection.room_a_id}-{connection.room_b_id} wall mismatch"
                )
            continue

        if connection.connection_type != ConnectionType.PASSAGE:
            raise CandidateConnectionError(
                f"Unknown connection type: {connection.connection_type}"
            )

        if len(connection.path_cell_ids) < 3:
            raise CandidateConnectionError(
                f"Passage {connection.room_a_id}-{connection.room_b_id} path too short"
            )
        if connection.path_cell_ids[0] != connection.room_a_id:
            raise CandidateConnectionError("Passage path must start at room_a_id")
        if connection.path_cell_ids[-1] != connection.room_b_id:
            raise CandidateConnectionError("Passage path must end at room_b_id")

        for cell_id in connection.path_cell_ids[1:-1]:
            if candidates.room_selection.role_for(cell_id) != CellRole.UNUSED:
                raise CandidateConnectionError(
                    f"Passage {connection.room_a_id}-{connection.room_b_id} "
                    f"passes through non-unused cell {cell_id}"
                )

        for left, right in zip(connection.path_cell_ids, connection.path_cell_ids[1:]):
            if not adjacency.graph.has_edge(left, right):
                raise CandidateConnectionError(
                    f"Passage path step {left}->{right} is not adjacent"
                )


def _shortest_path_through_unused(
    room_a_id: int,
    room_b_id: int,
    adjacency: AdjacencyGraph,
    room_selection: RoomSelection,
) -> tuple[int, ...] | None:
    queue: deque[tuple[int, list[int]]] = deque([(room_a_id, [room_a_id])])
    visited = {room_a_id}

    while queue:
        current_id, path = queue.popleft()
        if current_id == room_b_id:
            return tuple(path)

        for neighbor_id in adjacency.graph.neighbors(current_id):
            if neighbor_id in visited:
                continue
            if neighbor_id == room_b_id:
                pass
            elif room_selection.role_for(neighbor_id) != CellRole.UNUSED:
                continue

            visited.add(neighbor_id)
            queue.append((neighbor_id, path + [neighbor_id]))

    return None

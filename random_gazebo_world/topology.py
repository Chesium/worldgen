from __future__ import annotations

import random
from collections import deque
from dataclasses import dataclass
from enum import Enum

import networkx as nx

from random_gazebo_world.adjacency import AdjacencyGraph
from random_gazebo_world.config import Config
from random_gazebo_world.geometry import EPS, Cell, SharedWall, are_adjacent, get_shared_wall
from random_gazebo_world.openings import LogicalOpening
from random_gazebo_world.partition import Partition


class RoomSelectionError(RuntimeError):
    """Raised when room selection cannot satisfy config constraints."""


class CandidateConnectionError(RuntimeError):
    """Raised when candidate connections are invalid."""


class RoomGraphSelectionError(RuntimeError):
    """Raised when a connected room graph cannot be selected."""


class CellRole(str, Enum):
    ROOM = "room"
    UNUSED = "unused"
    PASSAGE = "passage"


class AppliedLayoutError(RuntimeError):
    """Raised when selected connections cannot be applied to the layout."""


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


@dataclass(frozen=True)
class SelectedRoomGraph:
    candidates: CandidateConnections
    connections: tuple[CandidateConnection, ...]
    spanning_tree_connections: tuple[CandidateConnection, ...]
    loop_connections: tuple[CandidateConnection, ...]

    @property
    def room_selection(self) -> RoomSelection:
        return self.candidates.room_selection

    @property
    def loop_connection_pairs(self) -> frozenset[tuple[int, int]]:
        return frozenset(
            (connection.room_a_id, connection.room_b_id)
            for connection in self.loop_connections
        )


@dataclass(frozen=True)
class AppliedLayout:
    partition: Partition
    room_selection: RoomSelection
    selected_graph: SelectedRoomGraph
    passage_cell_ids: frozenset[int]
    logical_openings: tuple[LogicalOpening, ...]

    def role_for(self, cell_id: int) -> CellRole:
        if cell_id in self.room_selection.room_cell_ids:
            return CellRole.ROOM
        if cell_id in self.passage_cell_ids:
            return CellRole.PASSAGE
        return CellRole.UNUSED

    def passage_cells(self) -> tuple[Cell, ...]:
        return tuple(
            cell
            for cell in self.partition.cells
            if cell.id in self.passage_cell_ids
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


def select_room_graph(
    candidates: CandidateConnections,
    config: Config,
    rng: random.Random,
) -> SelectedRoomGraph:
    room_ids = sorted(candidates.room_selection.room_cell_ids)
    if len(room_ids) <= 1:
        selected = SelectedRoomGraph(
            candidates=candidates,
            connections=(),
            spanning_tree_connections=(),
            loop_connections=(),
        )
        validate_selected_room_graph(selected, config)
        return selected

    spanning_tree = _randomized_spanning_tree(candidates.connections, room_ids, rng)
    selected = list(spanning_tree)
    selected_pairs = {
        (connection.room_a_id, connection.room_b_id) for connection in spanning_tree
    }

    for connection in candidates.connections:
        pair = (connection.room_a_id, connection.room_b_id)
        if pair in selected_pairs:
            continue
        if rng.random() < config.extra_loop_probability:
            selected.append(connection)
            selected_pairs.add(pair)

    selected.sort(key=lambda connection: (connection.room_a_id, connection.room_b_id))
    spanning_pairs = {
        (connection.room_a_id, connection.room_b_id) for connection in spanning_tree
    }
    loop_connections = tuple(
        connection
        for connection in selected
        if (connection.room_a_id, connection.room_b_id) not in spanning_pairs
    )

    selected_graph = SelectedRoomGraph(
        candidates=candidates,
        connections=tuple(selected),
        spanning_tree_connections=spanning_tree,
        loop_connections=loop_connections,
    )
    validate_selected_room_graph(selected_graph, config)
    return selected_graph


def validate_selected_room_graph(
    selected: SelectedRoomGraph,
    config: Config,
) -> None:
    room_ids = sorted(selected.room_selection.room_cell_ids)
    candidate_pairs = {
        (connection.room_a_id, connection.room_b_id)
        for connection in selected.candidates.connections
    }

    for connection in selected.connections:
        pair = (connection.room_a_id, connection.room_b_id)
        if pair not in candidate_pairs:
            raise RoomGraphSelectionError(
                f"Selected connection {pair} is not a candidate"
            )

    if len(room_ids) <= 1:
        return

    if len(selected.spanning_tree_connections) != len(room_ids) - 1:
        raise RoomGraphSelectionError("Selected graph must include a spanning tree")

    graph = nx.Graph()
    graph.add_nodes_from(room_ids)
    for connection in selected.connections:
        graph.add_edge(connection.room_a_id, connection.room_b_id)

    if not nx.is_connected(graph):
        raise RoomGraphSelectionError("Selected room graph is disconnected")

    tree_graph = nx.Graph()
    tree_graph.add_nodes_from(room_ids)
    for connection in selected.spanning_tree_connections:
        tree_graph.add_edge(connection.room_a_id, connection.room_b_id)
    if not nx.is_tree(tree_graph):
        raise RoomGraphSelectionError("Spanning tree connections do not form a tree")


def apply_connections(
    selected_graph: SelectedRoomGraph,
    adjacency: AdjacencyGraph,
) -> AppliedLayout:
    passage_cell_ids: set[int] = set()
    logical_openings: list[LogicalOpening] = []
    room_ids = selected_graph.room_selection.room_cell_ids

    for connection in selected_graph.connections:
        if connection.connection_type == ConnectionType.GATE:
            if connection.shared_wall is None:
                raise AppliedLayoutError(
                    f"Gate {connection.room_a_id}-{connection.room_b_id} missing wall"
                )
            logical_openings.append(
                LogicalOpening(
                    cell_a_id=min(connection.room_a_id, connection.room_b_id),
                    cell_b_id=max(connection.room_a_id, connection.room_b_id),
                    shared_wall=connection.shared_wall,
                    kind="gate",
                )
            )
            continue

        if len(connection.path_cell_ids) < 3:
            raise AppliedLayoutError(
                f"Passage {connection.room_a_id}-{connection.room_b_id} path too short"
            )

        for cell_id in connection.path_cell_ids[1:-1]:
            if cell_id in room_ids:
                raise AppliedLayoutError(
                    f"Passage {connection.room_a_id}-{connection.room_b_id} "
                    f"would reclassify room cell {cell_id}"
                )
            passage_cell_ids.add(cell_id)

        for left_id, right_id in zip(
            connection.path_cell_ids, connection.path_cell_ids[1:]
        ):
            shared_wall = adjacency.graph[left_id][right_id]["shared_wall"]
            logical_openings.append(
                LogicalOpening(
                    cell_a_id=min(left_id, right_id),
                    cell_b_id=max(left_id, right_id),
                    shared_wall=shared_wall,
                    kind="passage",
                )
            )

    layout = AppliedLayout(
        partition=selected_graph.room_selection.partition,
        room_selection=selected_graph.room_selection,
        selected_graph=selected_graph,
        passage_cell_ids=frozenset(passage_cell_ids),
        logical_openings=tuple(logical_openings),
    )
    validate_applied_layout(layout)
    return layout


def validate_applied_layout(layout: AppliedLayout) -> None:
    room_ids = layout.room_selection.room_cell_ids

    if layout.passage_cell_ids & room_ids:
        overlap = sorted(layout.passage_cell_ids & room_ids)
        raise AppliedLayoutError(f"Room cells reclassified as passage: {overlap}")

    for connection in layout.selected_graph.connections:
        if connection.connection_type != ConnectionType.PASSAGE:
            continue
        intermediate_ids = set(connection.path_cell_ids[1:-1])
        if not intermediate_ids:
            raise AppliedLayoutError(
                f"Passage {connection.room_a_id}-{connection.room_b_id} "
                "has no intermediate cells"
            )
        if not intermediate_ids.issubset(layout.passage_cell_ids):
            missing = sorted(intermediate_ids - layout.passage_cell_ids)
            raise AppliedLayoutError(
                f"Passage {connection.room_a_id}-{connection.room_b_id} "
                f"missing passage cells: {missing}"
            )


def _randomized_spanning_tree(
    connections: tuple[CandidateConnection, ...],
    room_ids: list[int],
    rng: random.Random,
) -> tuple[CandidateConnection, ...]:
    if not connections:
        raise RoomGraphSelectionError("No candidate connections available")

    shuffled = list(connections)
    rng.shuffle(shuffled)

    parent = {room_id: room_id for room_id in room_ids}

    def find(room_id: int) -> int:
        while parent[room_id] != room_id:
            parent[room_id] = parent[parent[room_id]]
            room_id = parent[room_id]
        return room_id

    def union(left_id: int, right_id: int) -> bool:
        left_root = find(left_id)
        right_root = find(right_id)
        if left_root == right_root:
            return False
        parent[right_root] = left_root
        return True

    spanning: list[CandidateConnection] = []
    for connection in shuffled:
        if union(connection.room_a_id, connection.room_b_id):
            spanning.append(connection)
        if len(spanning) == len(room_ids) - 1:
            break

    if len(spanning) < len(room_ids) - 1:
        raise RoomGraphSelectionError(
            "Candidate connections cannot span all selected rooms"
        )

    spanning.sort(key=lambda connection: (connection.room_a_id, connection.room_b_id))
    return tuple(spanning)


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

from __future__ import annotations

from dataclasses import dataclass

import networkx as nx

from random_gazebo_world.geometry import (
    EPS,
    Cell,
    SharedWall,
    are_adjacent,
    get_shared_wall,
)
from random_gazebo_world.partition import Partition


class AdjacencyError(RuntimeError):
    """Raised when an adjacency graph is invalid."""


@dataclass(frozen=True)
class AdjacencyEdge:
    cell_a_id: int
    cell_b_id: int
    shared_wall: SharedWall

    @property
    def overlap_length(self) -> float:
        return self.shared_wall.length


@dataclass(frozen=True)
class AdjacencyGraph:
    cells: tuple[Cell, ...]
    edges: tuple[AdjacencyEdge, ...]
    graph: nx.Graph

    def cell_by_id(self, cell_id: int) -> Cell:
        for cell in self.cells:
            if cell.id == cell_id:
                return cell
        raise KeyError(f"Unknown cell id: {cell_id}")


def build_adjacency_graph(partition: Partition) -> AdjacencyGraph:
    cells_by_id = {cell.id: cell for cell in partition.cells}
    edges: list[AdjacencyEdge] = []

    for left_index, left in enumerate(partition.cells):
        for right in partition.cells[left_index + 1 :]:
            shared_wall = get_shared_wall(left, right)
            if shared_wall is None:
                continue
            edges.append(
                AdjacencyEdge(
                    cell_a_id=left.id,
                    cell_b_id=right.id,
                    shared_wall=shared_wall,
                )
            )

    graph = nx.Graph()
    for cell in partition.cells:
        graph.add_node(cell.id, cell=cell)
    for edge in edges:
        graph.add_edge(
            edge.cell_a_id,
            edge.cell_b_id,
            shared_wall=edge.shared_wall,
            overlap_length=edge.overlap_length,
        )

    adjacency = AdjacencyGraph(
        cells=partition.cells,
        edges=tuple(edges),
        graph=graph,
    )
    validate_adjacency_graph(adjacency, partition)
    return adjacency


def validate_adjacency_graph(adjacency: AdjacencyGraph, partition: Partition) -> None:
    if adjacency.cells != partition.cells:
        raise AdjacencyError("Adjacency graph cells do not match partition cells")

    if adjacency.graph.number_of_nodes() != len(partition.cells):
        raise AdjacencyError("Adjacency graph node count mismatch")

    for edge in adjacency.edges:
        if edge.overlap_length <= EPS:
            raise AdjacencyError(
                f"Edge {edge.cell_a_id}-{edge.cell_b_id} has non-positive overlap"
            )

        cell_a = adjacency.cell_by_id(edge.cell_a_id)
        cell_b = adjacency.cell_by_id(edge.cell_b_id)
        if not are_adjacent(cell_a, cell_b):
            raise AdjacencyError(
                f"Edge {edge.cell_a_id}-{edge.cell_b_id} is not geometrically adjacent"
            )

        expected_wall = get_shared_wall(cell_a, cell_b)
        if expected_wall != edge.shared_wall:
            raise AdjacencyError(
                f"Edge {edge.cell_a_id}-{edge.cell_b_id} shared wall mismatch"
            )

    if len(partition.cells) > 1 and not nx.is_connected(adjacency.graph):
        raise AdjacencyError("Adjacency graph is disconnected")

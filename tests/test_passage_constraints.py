from __future__ import annotations

import pytest

from random_gazebo_world.config import Config
from random_gazebo_world.geometry import Cell, get_shared_wall
from random_gazebo_world.openings import LogicalOpening
from random_gazebo_world.partition import Partition
from random_gazebo_world.topology import (
    AppliedLayout,
    AppliedLayoutError,
    CandidateConnections,
    RoomSelection,
    SelectedRoomGraph,
    validate_passage_constraints,
)


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
        "extra_loop_probability": 0.0,
        "map_resolution": 0.05,
        "random_seed": 42,
    }
    values.update(overrides)
    config = Config(**values)  # type: ignore[arg-type]
    config.validate()
    return config


def _make_layout(
    partition: Partition,
    passage_cell_ids: set[int],
    openings: list[LogicalOpening],
) -> AppliedLayout:
    selection = RoomSelection(partition=partition, room_cell_ids=frozenset())
    candidates = CandidateConnections(room_selection=selection, connections=())
    selected_graph = SelectedRoomGraph(
        candidates=candidates,
        connections=(),
        spanning_tree_connections=(),
        loop_connections=(),
    )
    return AppliedLayout(
        partition=partition,
        room_selection=selection,
        selected_graph=selected_graph,
        passage_cell_ids=frozenset(passage_cell_ids),
        logical_openings=tuple(openings),
    )


def _opening(cell_a: Cell, cell_b: Cell) -> LogicalOpening:
    shared_wall = get_shared_wall(cell_a, cell_b)
    assert shared_wall is not None
    return LogicalOpening(
        cell_a_id=min(cell_a.id, cell_b.id),
        cell_b_id=max(cell_a.id, cell_b.id),
        shared_wall=shared_wall,
        kind="passage",
    )


def _cross_partition() -> tuple[Partition, dict[int, Cell]]:
    center = Cell.from_origin_size(0, 5.0, 5.0, 5.0, 5.0)
    west = Cell.from_origin_size(1, 0.0, 5.0, 5.0, 5.0)
    east = Cell.from_origin_size(2, 10.0, 5.0, 5.0, 5.0)
    south = Cell.from_origin_size(3, 5.0, 0.0, 5.0, 5.0)
    cells = (center, west, east, south)
    partition = Partition(cells=cells, world_width=15.0, world_height=15.0)
    return partition, {cell.id: cell for cell in cells}


def test_two_opposite_edge_openings_pass() -> None:
    config = _sample_config()
    partition, by_id = _cross_partition()
    openings = [
        _opening(by_id[0], by_id[1]),
        _opening(by_id[0], by_id[2]),
    ]
    layout = _make_layout(partition, {0}, openings)

    validate_passage_constraints(layout, config)


def test_two_openings_on_same_edge_raise() -> None:
    config = _sample_config()
    center = Cell.from_origin_size(0, 5.0, 5.0, 5.0, 5.0)
    west_low = Cell.from_origin_size(1, 0.0, 5.0, 5.0, 2.5)
    west_high = Cell.from_origin_size(2, 0.0, 7.5, 5.0, 2.5)
    cells = (center, west_low, west_high)
    partition = Partition(cells=cells, world_width=15.0, world_height=15.0)
    by_id = {cell.id: cell for cell in cells}
    openings = [
        _opening(by_id[0], by_id[1]),
        _opening(by_id[0], by_id[2]),
    ]
    layout = _make_layout(partition, {0}, openings)

    with pytest.raises(AppliedLayoutError, match="edge x_min has 2 openings"):
        validate_passage_constraints(layout, config)


def test_three_open_edges_raise_when_capped_at_two() -> None:
    partition, by_id = _cross_partition()
    openings = [
        _opening(by_id[0], by_id[1]),
        _opening(by_id[0], by_id[2]),
        _opening(by_id[0], by_id[3]),
    ]
    layout = _make_layout(partition, {0}, openings)

    capped = _sample_config(max_open_edges_per_passage=2)
    with pytest.raises(AppliedLayoutError, match="openings on 3 edges"):
        validate_passage_constraints(layout, capped)

    allowed = _sample_config(max_open_edges_per_passage=3)
    validate_passage_constraints(layout, allowed)

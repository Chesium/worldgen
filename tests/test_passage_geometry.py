from __future__ import annotations

import pytest

from random_gazebo_world.config import Config
from random_gazebo_world.geometry import Cell, SharedWall
from random_gazebo_world.openings import Opening, OpeningLayout
from random_gazebo_world.partition import Partition
from random_gazebo_world.passage_geometry import (
    PassageGeometryError,
    generate_passage_geometry,
)
from random_gazebo_world.topology import (
    AppliedLayout,
    CandidateConnections,
    RoomSelection,
    SelectedRoomGraph,
)

PASSAGE_ID = 0
CELL = Cell.from_origin_size(PASSAGE_ID, 0.0, 0.0, 4.0, 4.0)


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
        "gate_width_min": 0.4,
        "gate_width_max": 1.2,
        "passage_width_min": 0.4,
        "passage_width_max": 1.2,
        "extra_loop_probability": 0.0,
        "map_resolution": 0.05,
        "random_seed": 42,
    }
    values.update(overrides)
    config = Config(**values)  # type: ignore[arg-type]
    config.validate()
    return config


def _opening(edge: str, center: float, width: float, other_id: int) -> Opening:
    half = width / 2.0
    if edge in {"x_min", "x_max"}:
        fixed = CELL.x_min if edge == "x_min" else CELL.x_max
        shared_wall = SharedWall(
            orientation="vertical",
            fixed_coord=fixed,
            span_start=center - half,
            span_end=center + half,
        )
    else:
        fixed = CELL.y_min if edge == "y_min" else CELL.y_max
        shared_wall = SharedWall(
            orientation="horizontal",
            fixed_coord=fixed,
            span_start=center - half,
            span_end=center + half,
        )
    return Opening(
        cell_a_id=PASSAGE_ID,
        cell_b_id=other_id,
        shared_wall=shared_wall,
        kind="passage",
        width=width,
        span_start=center - half,
        span_end=center + half,
    )


def _passage_layout(openings: list[Opening]) -> OpeningLayout:
    partition = Partition(cells=(CELL,), world_width=4.0, world_height=4.0)
    selection = RoomSelection(partition=partition, room_cell_ids=frozenset())
    candidates = CandidateConnections(room_selection=selection, connections=())
    selected = SelectedRoomGraph(
        candidates=candidates,
        connections=(),
        spanning_tree_connections=(),
        loop_connections=(),
    )
    applied = AppliedLayout(
        partition=partition,
        room_selection=selection,
        selected_graph=selected,
        passage_cell_ids=frozenset({PASSAGE_ID}),
        logical_openings=(),
    )
    return OpeningLayout(applied_layout=applied, openings=tuple(openings))


def _corridor_area(layout) -> float:
    return sum(rect.area for cell in layout.cells for rect in cell.corridor)


def _solid_area(layout) -> float:
    return sum(rect.area for cell in layout.cells for rect in cell.solids)


def _assert_tiles_cell(layout) -> None:
    cell_area = CELL.width * CELL.height
    assert _corridor_area(layout) + _solid_area(layout) == pytest.approx(cell_area)


def test_straight_corridor_aligned_openings() -> None:
    config = _sample_config()
    layout = _passage_layout(
        [
            _opening("x_min", 2.0, 1.0, 1),
            _opening("x_max", 2.0, 1.0, 2),
        ]
    )
    geometry = generate_passage_geometry(layout, config)

    assert _corridor_area(geometry) == pytest.approx(4.0)
    _assert_tiles_cell(geometry)

    cell_geometry = geometry.cells[0]
    y_min = min(rect.y_min for rect in cell_geometry.corridor)
    y_max = max(rect.y_max for rect in cell_geometry.corridor)
    assert y_max - y_min == pytest.approx(1.0)


def test_corridor_width_is_min_of_pair() -> None:
    config = _sample_config()
    layout = _passage_layout(
        [
            _opening("x_min", 2.0, 1.0, 1),
            _opening("x_max", 2.0, 0.6, 2),
        ]
    )
    geometry = generate_passage_geometry(layout, config)

    assert _corridor_area(geometry) == pytest.approx(4.0 * 0.6)
    _assert_tiles_cell(geometry)


def test_l_shaped_corridor_adjacent_edges() -> None:
    config = _sample_config()
    layout = _passage_layout(
        [
            _opening("x_min", 2.0, 1.0, 1),
            _opening("y_min", 2.0, 1.0, 2),
        ]
    )
    geometry = generate_passage_geometry(layout, config)

    _assert_tiles_cell(geometry)
    assert geometry.cells[0].corridor
    assert geometry.cells[0].solids


def test_z_shaped_corridor_offset_openings() -> None:
    config = _sample_config()
    layout = _passage_layout(
        [
            _opening("x_min", 1.0, 1.0, 1),
            _opening("x_max", 3.0, 1.0, 2),
        ]
    )
    geometry = generate_passage_geometry(layout, config)

    _assert_tiles_cell(geometry)
    assert geometry.cells[0].corridor


def test_three_way_junction_unions_paths() -> None:
    config = _sample_config()
    layout = _passage_layout(
        [
            _opening("x_min", 2.0, 1.0, 1),
            _opening("x_max", 2.0, 1.0, 2),
            _opening("y_min", 2.0, 1.0, 3),
        ]
    )
    geometry = generate_passage_geometry(layout, config)

    _assert_tiles_cell(geometry)
    assert _corridor_area(geometry) > 4.0


def test_single_opening_passage_raises() -> None:
    config = _sample_config()
    layout = _passage_layout([_opening("x_min", 2.0, 1.0, 1)])

    with pytest.raises(PassageGeometryError):
        generate_passage_geometry(layout, config)

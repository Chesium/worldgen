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
    """Build an opening centered at ``center`` on a full cell edge.

    ``span_start``/``span_end`` are arc-length offsets along the shared wall,
    which always begins at the lower corner of the edge so arc-length matches
    the absolute coordinate.
    """
    half = width / 2.0
    if edge in {"x_min", "x_max"}:
        fixed = CELL.x_min if edge == "x_min" else CELL.x_max
        shared_wall = SharedWall(p1=(fixed, CELL.y_min), p2=(fixed, CELL.y_max))
    else:
        fixed = CELL.y_min if edge == "y_min" else CELL.y_max
        shared_wall = SharedWall(p1=(CELL.x_min, fixed), p2=(CELL.x_max, fixed))
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
    return sum(cell.corridor.area for cell in layout.cells)


def _solid_area(layout) -> float:
    return sum(solid.area for cell in layout.cells for solid in cell.solids)


def _assert_tiles_cell(layout) -> None:
    cell_area = CELL.area
    assert _corridor_area(layout) + _solid_area(layout) == pytest.approx(
        cell_area, abs=1e-4
    )


def _corridor_reaches(cell_geometry, point: tuple[float, float]) -> bool:
    from shapely.geometry import Point

    return cell_geometry.corridor.distance(Point(point)) <= 1e-6


def test_straight_corridor_aligned_openings() -> None:
    config = _sample_config()
    layout = _passage_layout(
        [
            _opening("x_min", 2.0, 1.0, 1),
            _opening("x_max", 2.0, 1.0, 2),
        ]
    )
    geometry = generate_passage_geometry(layout, config)

    cell_geometry = geometry.cells[0]
    assert cell_geometry.corridor.geom_type == "Polygon"
    # Straight corridor across the cell: width 1 over length 4.
    assert _corridor_area(geometry) == pytest.approx(4.0, abs=1e-3)
    _assert_tiles_cell(geometry)
    assert _corridor_reaches(cell_geometry, (0.0, 2.0))
    assert _corridor_reaches(cell_geometry, (4.0, 2.0))


def test_corridor_width_matches_each_port() -> None:
    config = _sample_config()
    layout = _passage_layout(
        [
            _opening("x_min", 2.0, 1.0, 1),
            _opening("x_max", 2.0, 0.6, 2),
        ]
    )
    geometry = generate_passage_geometry(layout, config)

    cell_geometry = geometry.cells[0]
    _assert_tiles_cell(geometry)
    assert _corridor_reaches(cell_geometry, (0.0, 2.0))
    assert _corridor_reaches(cell_geometry, (4.0, 2.0))
    # Each leg keeps its own opening width, so the corridor is wider near the
    # wider opening than the old min-of-pair behavior.
    assert _corridor_area(geometry) > 4.0 * 0.6


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
    cell_geometry = geometry.cells[0]
    assert not cell_geometry.corridor.is_empty
    assert cell_geometry.solids
    assert _corridor_reaches(cell_geometry, (0.0, 2.0))
    assert _corridor_reaches(cell_geometry, (2.0, 0.0))


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
    cell_geometry = geometry.cells[0]
    assert not cell_geometry.corridor.is_empty
    assert _corridor_reaches(cell_geometry, (0.0, 1.0))
    assert _corridor_reaches(cell_geometry, (4.0, 3.0))


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

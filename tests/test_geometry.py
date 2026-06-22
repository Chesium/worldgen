from __future__ import annotations

import pytest

from random_gazebo_world.geometry import (
    Cell,
    SharedWall,
    are_adjacent,
    get_shared_wall,
    overlap_length,
    rectangles_intersect,
)


def test_cell_from_origin_size() -> None:
    cell = Cell.from_origin_size(1, 0.0, 0.0, 4.0, 3.0)
    assert cell.id == 1
    assert cell.x_min == 0.0
    assert cell.y_min == 0.0
    assert cell.x_max == 4.0
    assert cell.y_max == 3.0
    assert cell.width == 4.0
    assert cell.height == 3.0


def test_cell_rejects_invalid_bounds() -> None:
    with pytest.raises(ValueError, match="x_min"):
        Cell(id=0, x_min=2.0, y_min=0.0, x_max=2.0, y_max=3.0)
    with pytest.raises(ValueError, match="y_min"):
        Cell(id=0, x_min=0.0, y_min=3.0, x_max=4.0, y_max=3.0)


def test_rectangles_intersect_for_overlapping_cells() -> None:
    left = Cell.from_origin_size(0, 0.0, 0.0, 4.0, 4.0)
    overlap = Cell.from_origin_size(1, 2.0, 2.0, 4.0, 4.0)
    assert rectangles_intersect(left, overlap)


def test_rectangles_do_not_intersect_for_separated_cells() -> None:
    left = Cell.from_origin_size(0, 0.0, 0.0, 2.0, 2.0)
    right = Cell.from_origin_size(1, 3.0, 0.0, 2.0, 2.0)
    assert not rectangles_intersect(left, right)


def test_adjacent_cells_share_vertical_wall() -> None:
    left = Cell.from_origin_size(0, 0.0, 0.0, 2.0, 4.0)
    right = Cell.from_origin_size(1, 2.0, 0.0, 2.0, 4.0)

    assert are_adjacent(left, right)
    assert overlap_length(left, right) == pytest.approx(4.0)

    wall = get_shared_wall(left, right)
    assert wall == SharedWall(
        orientation="vertical",
        fixed_coord=2.0,
        span_start=0.0,
        span_end=4.0,
    )


def test_adjacent_cells_share_horizontal_wall() -> None:
    bottom = Cell.from_origin_size(0, 0.0, 0.0, 4.0, 2.0)
    top = Cell.from_origin_size(1, 0.0, 2.0, 4.0, 2.0)

    assert are_adjacent(bottom, top)
    assert overlap_length(bottom, top) == pytest.approx(4.0)

    wall = get_shared_wall(bottom, top)
    assert wall == SharedWall(
        orientation="horizontal",
        fixed_coord=2.0,
        span_start=0.0,
        span_end=4.0,
    )


def test_partial_edge_overlap_is_adjacent_with_shorter_wall() -> None:
    left = Cell.from_origin_size(0, 0.0, 0.0, 2.0, 4.0)
    right = Cell.from_origin_size(1, 2.0, 1.0, 2.0, 2.0)

    assert are_adjacent(left, right)
    assert overlap_length(left, right) == pytest.approx(2.0)

    wall = get_shared_wall(left, right)
    assert wall == SharedWall(
        orientation="vertical",
        fixed_coord=2.0,
        span_start=1.0,
        span_end=3.0,
    )


def test_corner_touching_cells_are_not_adjacent() -> None:
    bottom_left = Cell.from_origin_size(0, 0.0, 0.0, 2.0, 2.0)
    top_right = Cell.from_origin_size(1, 2.0, 2.0, 2.0, 2.0)

    assert not are_adjacent(bottom_left, top_right)
    assert overlap_length(bottom_left, top_right) == 0.0
    assert get_shared_wall(bottom_left, top_right) is None


def test_separated_cells_are_not_adjacent() -> None:
    left = Cell.from_origin_size(0, 0.0, 0.0, 2.0, 2.0)
    right = Cell.from_origin_size(1, 3.0, 0.0, 2.0, 2.0)

    assert not are_adjacent(left, right)
    assert overlap_length(left, right) == 0.0
    assert get_shared_wall(left, right) is None


def test_overlapping_cells_are_not_adjacent() -> None:
    base = Cell.from_origin_size(0, 0.0, 0.0, 4.0, 4.0)
    overlap = Cell.from_origin_size(1, 2.0, 2.0, 2.0, 2.0)

    assert not are_adjacent(base, overlap)
    assert overlap_length(base, overlap) == 0.0
    assert get_shared_wall(base, overlap) is None

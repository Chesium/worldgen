from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

EPS = 1e-9


@dataclass(frozen=True)
class Cell:
    """Axis-aligned rectangular partition cell."""

    id: int
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    def __post_init__(self) -> None:
        if self.x_min >= self.x_max:
            raise ValueError(
                f"Cell {self.id}: x_min ({self.x_min}) must be < x_max ({self.x_max})"
            )
        if self.y_min >= self.y_max:
            raise ValueError(
                f"Cell {self.id}: y_min ({self.y_min}) must be < y_max ({self.y_max})"
            )

    @classmethod
    def from_origin_size(
        cls,
        cell_id: int,
        x: float,
        y: float,
        width: float,
        height: float,
    ) -> Cell:
        return cls(
            id=cell_id,
            x_min=x,
            y_min=y,
            x_max=x + width,
            y_max=y + height,
        )

    @property
    def width(self) -> float:
        return self.x_max - self.x_min

    @property
    def height(self) -> float:
        return self.y_max - self.y_min


@dataclass(frozen=True)
class SharedWall:
    """Wall segment shared by two adjacent cells."""

    orientation: Literal["vertical", "horizontal"]
    fixed_coord: float
    span_start: float
    span_end: float

    @property
    def length(self) -> float:
        return self.span_end - self.span_start


def rectangles_intersect(a: Cell, b: Cell, eps: float = EPS) -> bool:
    """Return True when two cells overlap with positive area."""
    if a.x_max <= b.x_min + eps or b.x_max <= a.x_min + eps:
        return False
    if a.y_max <= b.y_min + eps or b.y_max <= a.y_min + eps:
        return False
    return True


def are_adjacent(a: Cell, b: Cell, eps: float = EPS) -> bool:
    """Return True when two cells share a wall segment with positive length."""
    return get_shared_wall(a, b, eps=eps) is not None


def overlap_length(a: Cell, b: Cell, eps: float = EPS) -> float:
    """Return the length of the shared wall between adjacent cells, else 0."""
    shared = get_shared_wall(a, b, eps=eps)
    if shared is None:
        return 0.0
    return shared.length


def get_shared_wall(a: Cell, b: Cell, eps: float = EPS) -> SharedWall | None:
    """Return shared wall geometry for adjacent cells, else None."""
    if rectangles_intersect(a, b, eps=eps):
        return None

    vertical = _vertical_shared_wall(a, b, eps)
    horizontal = _horizontal_shared_wall(a, b, eps)

    if vertical is not None and horizontal is not None:
        # Corner contact only; not a shared wall segment.
        return None
    return vertical if vertical is not None else horizontal


def _vertical_shared_wall(a: Cell, b: Cell, eps: float) -> SharedWall | None:
    if _approx_equal(a.x_max, b.x_min, eps):
        fixed_coord = a.x_max
    elif _approx_equal(b.x_max, a.x_min, eps):
        fixed_coord = b.x_max
    else:
        return None

    span_start = max(a.y_min, b.y_min)
    span_end = min(a.y_max, b.y_max)
    if span_end - span_start <= eps:
        return None

    return SharedWall(
        orientation="vertical",
        fixed_coord=fixed_coord,
        span_start=span_start,
        span_end=span_end,
    )


def _horizontal_shared_wall(a: Cell, b: Cell, eps: float) -> SharedWall | None:
    if _approx_equal(a.y_max, b.y_min, eps):
        fixed_coord = a.y_max
    elif _approx_equal(b.y_max, a.y_min, eps):
        fixed_coord = b.y_max
    else:
        return None

    span_start = max(a.x_min, b.x_min)
    span_end = min(a.x_max, b.x_max)
    if span_end - span_start <= eps:
        return None

    return SharedWall(
        orientation="horizontal",
        fixed_coord=fixed_coord,
        span_start=span_start,
        span_end=span_end,
    )


def _approx_equal(a: float, b: float, eps: float) -> bool:
    return abs(a - b) <= eps


Room = Cell

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

from shapely.geometry import Polygon

EPS = 1e-9

Vec2 = tuple[float, float]


@dataclass(frozen=True)
class Edge:
    """A directed boundary edge of a cell polygon."""

    a: Vec2
    b: Vec2
    index: int

    @property
    def length(self) -> float:
        return math.hypot(self.b[0] - self.a[0], self.b[1] - self.a[1])


@dataclass(frozen=True)
class Cell:
    """Partition cell.

    A cell is an axis-aligned rectangle by default (described by its bounding
    box). When ``vertices`` is provided the cell is a general convex polygon and
    ``x_min``/``y_min``/``x_max``/``y_max`` describe its bounding box.
    """

    id: int
    x_min: float
    y_min: float
    x_max: float
    y_max: float
    vertices: tuple[Vec2, ...] | None = None

    def __post_init__(self) -> None:
        if self.x_min >= self.x_max:
            raise ValueError(
                f"Cell {self.id}: x_min ({self.x_min}) must be < x_max ({self.x_max})"
            )
        if self.y_min >= self.y_max:
            raise ValueError(
                f"Cell {self.id}: y_min ({self.y_min}) must be < y_max ({self.y_max})"
            )
        if self.vertices is not None and len(self.vertices) < 3:
            raise ValueError(f"Cell {self.id}: polygon needs at least 3 vertices")

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

    @classmethod
    def from_polygon(cls, cell_id: int, vertices: tuple[Vec2, ...]) -> Cell:
        if len(vertices) < 3:
            raise ValueError(f"Cell {cell_id}: polygon needs at least 3 vertices")
        ordered = _orient_ccw(tuple(vertices))
        xs = [point[0] for point in ordered]
        ys = [point[1] for point in ordered]
        return cls(
            id=cell_id,
            x_min=min(xs),
            y_min=min(ys),
            x_max=max(xs),
            y_max=max(ys),
            vertices=ordered,
        )

    @property
    def width(self) -> float:
        return self.x_max - self.x_min

    @property
    def height(self) -> float:
        return self.y_max - self.y_min

    @property
    def is_rectangle(self) -> bool:
        return self.vertices is None

    @property
    def polygon_vertices(self) -> tuple[Vec2, ...]:
        if self.vertices is not None:
            return self.vertices
        return (
            (self.x_min, self.y_min),
            (self.x_max, self.y_min),
            (self.x_max, self.y_max),
            (self.x_min, self.y_max),
        )

    @property
    def edges(self) -> tuple[Edge, ...]:
        verts = self.polygon_vertices
        count = len(verts)
        return tuple(
            Edge(a=verts[index], b=verts[(index + 1) % count], index=index)
            for index in range(count)
        )

    @property
    def polygon(self) -> Polygon:
        return Polygon(self.polygon_vertices)

    @property
    def area(self) -> float:
        return abs(_shoelace(self.polygon_vertices))

    @property
    def centroid(self) -> Vec2:
        if self.vertices is None:
            return (
                (self.x_min + self.x_max) / 2.0,
                (self.y_min + self.y_max) / 2.0,
            )
        return _polygon_centroid(self.polygon_vertices)


@dataclass(frozen=True)
class Rect:
    """Axis-aligned rectangle without identity or positive-area assertions."""

    x_min: float
    y_min: float
    x_max: float
    y_max: float

    @property
    def width(self) -> float:
        return self.x_max - self.x_min

    @property
    def height(self) -> float:
        return self.y_max - self.y_min

    @property
    def area(self) -> float:
        return self.width * self.height

    @property
    def center(self) -> tuple[float, float]:
        return (
            (self.x_min + self.x_max) / 2.0,
            (self.y_min + self.y_max) / 2.0,
        )


class SharedWall:
    """A wall segment shared by two adjacent cells.

    Canonically represented by its two endpoints ``p1`` and ``p2`` (sorted
    lexicographically). Axis-aligned convenience accessors (``orientation``,
    ``fixed_coord``, ``span_start``, ``span_end``) are provided so the
    rectangular code paths and existing callers keep working.
    """

    __slots__ = ("p1", "p2")

    def __init__(
        self,
        orientation: Literal["vertical", "horizontal"] | None = None,
        fixed_coord: float | None = None,
        span_start: float | None = None,
        span_end: float | None = None,
        *,
        p1: Vec2 | None = None,
        p2: Vec2 | None = None,
    ) -> None:
        if p1 is not None and p2 is not None:
            point_a, point_b = p1, p2
        elif orientation is not None:
            if fixed_coord is None or span_start is None or span_end is None:
                raise ValueError("Axis-aligned SharedWall requires span bounds")
            if orientation == "vertical":
                point_a = (fixed_coord, span_start)
                point_b = (fixed_coord, span_end)
            else:
                point_a = (span_start, fixed_coord)
                point_b = (span_end, fixed_coord)
        else:
            raise ValueError("SharedWall requires endpoints or axis description")

        if point_b < point_a:
            point_a, point_b = point_b, point_a
        object.__setattr__(self, "p1", point_a)
        object.__setattr__(self, "p2", point_b)

    @classmethod
    def from_axis(
        cls,
        orientation: Literal["vertical", "horizontal"],
        fixed_coord: float,
        span_start: float,
        span_end: float,
    ) -> SharedWall:
        return cls(
            orientation=orientation,
            fixed_coord=fixed_coord,
            span_start=span_start,
            span_end=span_end,
        )

    @property
    def length(self) -> float:
        return math.hypot(self.p2[0] - self.p1[0], self.p2[1] - self.p1[1])

    @property
    def orientation(self) -> str:
        if abs(self.p1[0] - self.p2[0]) <= EPS:
            return "vertical"
        if abs(self.p1[1] - self.p2[1]) <= EPS:
            return "horizontal"
        return "diagonal"

    @property
    def fixed_coord(self) -> float:
        orientation = self.orientation
        if orientation == "vertical":
            return self.p1[0]
        if orientation == "horizontal":
            return self.p1[1]
        raise ValueError("fixed_coord undefined for diagonal wall")

    @property
    def span_start(self) -> float:
        orientation = self.orientation
        if orientation == "vertical":
            return min(self.p1[1], self.p2[1])
        if orientation == "horizontal":
            return min(self.p1[0], self.p2[0])
        raise ValueError("span_start undefined for diagonal wall")

    @property
    def span_end(self) -> float:
        orientation = self.orientation
        if orientation == "vertical":
            return max(self.p1[1], self.p2[1])
        if orientation == "horizontal":
            return max(self.p1[0], self.p2[0])
        raise ValueError("span_end undefined for diagonal wall")

    @property
    def midpoint(self) -> Vec2:
        return (
            (self.p1[0] + self.p2[0]) / 2.0,
            (self.p1[1] + self.p2[1]) / 2.0,
        )

    @property
    def direction(self) -> Vec2:
        length = self.length
        if length <= EPS:
            return (0.0, 0.0)
        return (
            (self.p2[0] - self.p1[0]) / length,
            (self.p2[1] - self.p1[1]) / length,
        )

    @property
    def normal(self) -> Vec2:
        dx, dy = self.direction
        return (-dy, dx)

    def point_at(self, t: float) -> Vec2:
        return (
            self.p1[0] + t * (self.p2[0] - self.p1[0]),
            self.p1[1] + t * (self.p2[1] - self.p1[1]),
        )

    def point_at_arc_length(self, distance: float) -> Vec2:
        length = self.length
        if length <= EPS:
            return self.p1
        return self.point_at(distance / length)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, SharedWall):
            return NotImplemented
        return self.p1 == other.p1 and self.p2 == other.p2

    def __hash__(self) -> int:
        return hash((self.p1, self.p2))

    def __repr__(self) -> str:
        return f"SharedWall(p1={self.p1}, p2={self.p2})"


def rectangles_intersect(a: Cell, b: Cell, eps: float = EPS) -> bool:
    """Return True when two cells' bounding boxes overlap with positive area."""
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
    if a.is_rectangle and b.is_rectangle:
        return _rectangle_shared_wall(a, b, eps)
    return _polygon_shared_wall(a, b, eps)


def _rectangle_shared_wall(a: Cell, b: Cell, eps: float) -> SharedWall | None:
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

    return SharedWall.from_axis(
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

    return SharedWall.from_axis(
        orientation="horizontal",
        fixed_coord=fixed_coord,
        span_start=span_start,
        span_end=span_end,
    )


def _polygon_shared_wall(a: Cell, b: Cell, eps: float) -> SharedWall | None:
    poly_a = a.polygon
    poly_b = b.polygon
    if poly_a.overlaps(poly_b) and poly_a.intersection(poly_b).area > eps:
        return None

    shared = poly_a.boundary.intersection(poly_b.boundary)
    if shared.is_empty:
        return None

    segment = _longest_segment(shared)
    if segment is None:
        return None
    p1, p2 = segment
    if math.hypot(p2[0] - p1[0], p2[1] - p1[1]) <= eps:
        return None
    return SharedWall(p1=p1, p2=p2)


def _longest_segment(geometry) -> tuple[Vec2, Vec2] | None:
    best: tuple[Vec2, Vec2] | None = None
    best_length = 0.0
    for line in _iter_lines(geometry):
        coords = list(line.coords)
        for start, end in zip(coords, coords[1:]):
            length = math.hypot(end[0] - start[0], end[1] - start[1])
            if length > best_length:
                best_length = length
                best = ((start[0], start[1]), (end[0], end[1]))
    return best


def _iter_lines(geometry):
    geom_type = geometry.geom_type
    if geom_type == "LineString":
        yield geometry
    elif geom_type in {"MultiLineString", "GeometryCollection"}:
        for part in geometry.geoms:
            if part.geom_type == "LineString" and not part.is_empty:
                yield part


def _approx_equal(a: float, b: float, eps: float) -> bool:
    return abs(a - b) <= eps


def _shoelace(vertices: tuple[Vec2, ...]) -> float:
    total = 0.0
    count = len(vertices)
    for index in range(count):
        x1, y1 = vertices[index]
        x2, y2 = vertices[(index + 1) % count]
        total += x1 * y2 - x2 * y1
    return total / 2.0


def _polygon_centroid(vertices: tuple[Vec2, ...]) -> Vec2:
    area = _shoelace(vertices)
    if abs(area) <= EPS:
        xs = [point[0] for point in vertices]
        ys = [point[1] for point in vertices]
        return (sum(xs) / len(xs), sum(ys) / len(ys))
    cx = 0.0
    cy = 0.0
    count = len(vertices)
    for index in range(count):
        x1, y1 = vertices[index]
        x2, y2 = vertices[(index + 1) % count]
        cross = x1 * y2 - x2 * y1
        cx += (x1 + x2) * cross
        cy += (y1 + y2) * cross
    factor = 1.0 / (6.0 * area)
    return (cx * factor, cy * factor)


def _orient_ccw(vertices: tuple[Vec2, ...]) -> tuple[Vec2, ...]:
    if _shoelace(vertices) < 0:
        return tuple(reversed(vertices))
    return vertices


Room = Cell

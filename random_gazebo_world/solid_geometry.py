from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Iterator

import numpy as np
from shapely.geometry import Point, Polygon
from shapely.geometry import box as shapely_box
from shapely.geometry.base import BaseGeometry

from random_gazebo_world.geometry import EPS, Rect
from random_gazebo_world.topology import CellRole

if TYPE_CHECKING:
    from random_gazebo_world.geometry import Cell
    from random_gazebo_world.walls import WallLayout


class SolidProvenance(str, Enum):
    UNUSED_CELL = "unused_cell"
    PASSAGE_LEFTOVER = "passage_leftover"


class SolidShape(str, Enum):
    AXIS_ALIGNED_RECT = "axis_aligned_rect"
    ORTHOGONAL = "orthogonal"
    GENERAL = "general"


@dataclass(frozen=True)
class TaggedSolid:
    polygon: Polygon
    provenance: SolidProvenance
    shape: SolidShape
    cell_id: int | None = None


def collect_tagged_solids(wall_layout: WallLayout) -> tuple[TaggedSolid, ...]:
    """Return solid fill polygons with source and shape metadata."""
    layout = wall_layout.opening_layout.applied_layout
    solids: list[TaggedSolid] = []
    for cell in layout.partition.cells:
        if layout.role_for(cell.id) is not CellRole.UNUSED:
            continue
        polygon = cell.polygon
        solids.append(
            TaggedSolid(
                polygon=polygon,
                provenance=SolidProvenance.UNUSED_CELL,
                shape=_shape_for_unused_cell(cell, polygon),
                cell_id=cell.id,
            )
        )

    if wall_layout.passage_geometry is not None:
        for cell_geometry in wall_layout.passage_geometry.cells:
            for polygon in cell_geometry.solids:
                solids.append(
                    TaggedSolid(
                        polygon=polygon,
                        provenance=SolidProvenance.PASSAGE_LEFTOVER,
                        shape=polygon_shape(polygon),
                        cell_id=cell_geometry.cell_id,
                    )
                )
    return tuple(solids)


def iter_polygons(geometry: BaseGeometry) -> Iterator[Polygon]:
    if geometry.is_empty:
        return
    if geometry.geom_type == "Polygon":
        polygon = geometry
        if polygon.area > EPS:
            yield polygon
    elif geometry.geom_type in {"MultiPolygon", "GeometryCollection"}:
        for part in geometry.geoms:
            if part.geom_type == "Polygon" and part.area > EPS:
                yield part


def is_axis_aligned_rectangle(polygon: Polygon, eps: float = EPS) -> bool:
    if polygon.is_empty or len(polygon.interiors) > 0:
        return False
    coords = _open_ring_coords(polygon.exterior.coords)
    if len(coords) != 4:
        return False
    xs = {_rounded(value, eps) for value, _ in coords}
    ys = {_rounded(value, eps) for _, value in coords}
    if len(xs) != 2 or len(ys) != 2:
        return False
    min_x, min_y, max_x, max_y = polygon.bounds
    return abs(polygon.area - (max_x - min_x) * (max_y - min_y)) <= eps


def is_orthogonal_polygon(polygon: Polygon, eps: float = EPS) -> bool:
    if polygon.is_empty:
        return False
    rings = [polygon.exterior, *polygon.interiors]
    for ring in rings:
        coords = list(ring.coords)
        for (ax, ay), (bx, by) in zip(coords, coords[1:], strict=False):
            if abs(ax - bx) > eps and abs(ay - by) > eps:
                return False
    return True


def polygon_shape(polygon: Polygon) -> SolidShape:
    if is_axis_aligned_rectangle(polygon):
        return SolidShape.AXIS_ALIGNED_RECT
    if is_orthogonal_polygon(polygon):
        return SolidShape.ORTHOGONAL
    return SolidShape.GENERAL


def decompose_orthogonal_polygon(polygon: Polygon) -> tuple[Rect, ...]:
    """Decompose an orthogonal polygon into exact grid-induced rectangles."""
    if not is_orthogonal_polygon(polygon):
        raise ValueError("Only orthogonal polygons can be decomposed into rectangles")

    xs = _unique_ring_values(polygon, axis=0)
    ys = _unique_ring_values(polygon, axis=1)
    if len(xs) < 2 or len(ys) < 2:
        return ()

    mask = np.zeros((len(ys) - 1, len(xs) - 1), dtype=bool)
    for row in range(len(ys) - 1):
        for col in range(len(xs) - 1):
            rect = shapely_box(xs[col], ys[row], xs[col + 1], ys[row + 1])
            if rect.area <= EPS:
                continue
            if polygon.covers(Point(rect.centroid.x, rect.centroid.y)):
                mask[row, col] = polygon.intersection(rect).area >= rect.area - 1e-7

    rectangles: list[Rect] = []
    for r0, c0, r1, c1 in _greedy_rectangles(mask):
        rectangles.append(Rect(x_min=xs[c0], y_min=ys[r0], x_max=xs[c1 + 1], y_max=ys[r1 + 1]))
    return tuple(rectangles)


def rect_from_polygon_bounds(polygon: Polygon) -> Rect:
    min_x, min_y, max_x, max_y = polygon.bounds
    return Rect(x_min=min_x, y_min=min_y, x_max=max_x, y_max=max_y)


def _shape_for_unused_cell(cell: Cell, polygon: Polygon) -> SolidShape:
    if cell.is_rectangle and is_axis_aligned_rectangle(polygon):
        return SolidShape.AXIS_ALIGNED_RECT
    return polygon_shape(polygon)


def _unique_ring_values(polygon: Polygon, *, axis: int) -> list[float]:
    values: list[float] = []
    for ring in [polygon.exterior, *polygon.interiors]:
        for coord in ring.coords:
            values.append(float(coord[axis]))
    return sorted(set(_rounded(value, EPS) for value in values))


def _open_ring_coords(coords) -> list[tuple[float, float]]:
    points = [(float(x), float(y)) for x, y in coords]
    if len(points) >= 2 and points[0] == points[-1]:
        return points[:-1]
    return points


def _rounded(value: float, eps: float) -> float:
    if eps <= 0:
        return value
    digits = max(0, int(abs(math.log10(eps))))
    return round(value, digits)


def _greedy_rectangles(mask: np.ndarray) -> list[tuple[int, int, int, int]]:
    remaining = mask.copy()
    height, width = remaining.shape
    rects: list[tuple[int, int, int, int]] = []
    flat = remaining.reshape(-1)
    while True:
        idx = int(np.argmax(flat))
        if not flat[idx]:
            break
        r0, c0 = divmod(idx, width)
        c1 = c0
        while c1 + 1 < width and remaining[r0, c1 + 1]:
            c1 += 1
        r1 = r0
        while r1 + 1 < height and remaining[r1 + 1, c0 : c1 + 1].all():
            r1 += 1
        remaining[r0 : r1 + 1, c0 : c1 + 1] = False
        rects.append((r0, c0, r1, c1))
    return rects

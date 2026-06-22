from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
from typing import TYPE_CHECKING, Literal

from shapely.geometry import LineString, Point, box
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union
from shapely.prepared import prep

from random_gazebo_world.config import Config
from random_gazebo_world.geometry import EPS, Cell, Rect
from random_gazebo_world.openings import Opening, OpeningLayout

if TYPE_CHECKING:
    from random_gazebo_world.topology import AppliedLayout

PortKind = Literal["vertical", "horizontal"]


class PassageGeometryError(RuntimeError):
    """Raised when passage corridor geometry cannot be built or is invalid."""


@dataclass(frozen=True)
class Port:
    """Connection point of an opening on a passage cell edge."""

    x: float
    y: float
    kind: PortKind
    width: float


@dataclass(frozen=True)
class PassageCellGeometry:
    cell_id: int
    corridor: tuple[Rect, ...]
    solids: tuple[Rect, ...]


@dataclass(frozen=True)
class PassageGeometryLayout:
    opening_layout: OpeningLayout
    cells: tuple[PassageCellGeometry, ...]

    @property
    def solids(self) -> tuple[Rect, ...]:
        return tuple(rect for cell in self.cells for rect in cell.solids)

    @property
    def corridors(self) -> tuple[Rect, ...]:
        return tuple(rect for cell in self.cells for rect in cell.corridor)

    def corridor_for(self, cell_id: int) -> tuple[Rect, ...]:
        for cell in self.cells:
            if cell.cell_id == cell_id:
                return cell.corridor
        return ()


def generate_passage_geometry(
    opening_layout: OpeningLayout,
    config: Config,
) -> PassageGeometryLayout:
    layout = opening_layout.applied_layout
    cells_by_id = {cell.id: cell for cell in layout.partition.cells}
    openings_by_cell = _group_openings_by_passage_cell(opening_layout, layout)

    cell_geometries: list[PassageCellGeometry] = []
    for cell_id in sorted(openings_by_cell):
        cell = cells_by_id[cell_id]
        openings = openings_by_cell[cell_id]
        cell_geometries.append(_build_cell_geometry(cell, openings))

    geometry = PassageGeometryLayout(
        opening_layout=opening_layout,
        cells=tuple(cell_geometries),
    )
    validate_passage_geometry(geometry, config)
    return geometry


def _build_cell_geometry(
    cell: Cell,
    openings: tuple[Opening, ...],
) -> PassageCellGeometry:
    if len(openings) < 2:
        raise PassageGeometryError(
            f"Passage cell {cell.id} has {len(openings)} openings, needs at least 2"
        )

    ports = [_opening_port(cell, opening) for opening in openings]
    cell_box = box(cell.x_min, cell.y_min, cell.x_max, cell.y_max)

    strips: list[BaseGeometry] = []
    for port_a, port_b in combinations(ports, 2):
        polyline = _route_polyline(port_a, port_b, cell)
        width = min(port_a.width, port_b.width)
        strip = LineString(polyline).buffer(
            width / 2.0, cap_style="flat", join_style="mitre"
        )
        strips.append(strip)

    corridor_geom = unary_union(strips).intersection(cell_box)
    if corridor_geom.is_empty or corridor_geom.area <= EPS:
        raise PassageGeometryError(
            f"Passage cell {cell.id} produced an empty corridor"
        )

    leftover_geom = cell_box.difference(corridor_geom)

    xs, ys = _grid_lines(corridor_geom, cell)
    corridor_rects = _geometry_to_rects(corridor_geom, xs, ys)
    solid_rects = _geometry_to_rects(leftover_geom, xs, ys)

    return PassageCellGeometry(
        cell_id=cell.id,
        corridor=tuple(corridor_rects),
        solids=tuple(solid_rects),
    )


def validate_passage_geometry(
    layout: PassageGeometryLayout,
    config: Config,
) -> None:
    cells_by_id = {
        cell.id: cell
        for cell in layout.opening_layout.applied_layout.partition.cells
    }
    openings_by_cell = _group_openings_by_passage_cell(
        layout.opening_layout, layout.opening_layout.applied_layout
    )

    for cell_geometry in layout.cells:
        cell = cells_by_id[cell_geometry.cell_id]
        cell_box = box(cell.x_min, cell.y_min, cell.x_max, cell.y_max)
        corridor_geom = unary_union(
            [box(r.x_min, r.y_min, r.x_max, r.y_max) for r in cell_geometry.corridor]
        )

        if corridor_geom.is_empty:
            raise PassageGeometryError(
                f"Passage cell {cell.id} corridor is empty"
            )
        if corridor_geom.geom_type != "Polygon":
            raise PassageGeometryError(
                f"Passage cell {cell.id} corridor is not a single connected region"
            )

        for opening in openings_by_cell[cell_geometry.cell_id]:
            port = _opening_port(cell, opening)
            if corridor_geom.distance(Point(port.x, port.y)) > 1e-6:
                raise PassageGeometryError(
                    f"Passage cell {cell.id} corridor does not reach opening "
                    f"{opening.cell_a_id}-{opening.cell_b_id}"
                )

        corridor_area = sum(rect.area for rect in cell_geometry.corridor)
        solid_area = sum(rect.area for rect in cell_geometry.solids)
        if abs(corridor_area + solid_area - cell_box.area) > 1e-6:
            raise PassageGeometryError(
                f"Passage cell {cell.id} corridor and solids do not tile the cell"
            )

        for rect in cell_geometry.solids:
            if (
                rect.x_min < cell.x_min - EPS
                or rect.y_min < cell.y_min - EPS
                or rect.x_max > cell.x_max + EPS
                or rect.y_max > cell.y_max + EPS
            ):
                raise PassageGeometryError(
                    f"Passage cell {cell.id} solid box lies outside the cell"
                )


def _group_openings_by_passage_cell(
    opening_layout: OpeningLayout,
    layout: AppliedLayout,
) -> dict[int, tuple[Opening, ...]]:
    grouped: dict[int, list[Opening]] = {}
    for opening in opening_layout.openings:
        for cell_id in (opening.cell_a_id, opening.cell_b_id):
            if cell_id in layout.passage_cell_ids:
                grouped.setdefault(cell_id, []).append(opening)
    return {cell_id: tuple(items) for cell_id, items in grouped.items()}


def _opening_port(cell: Cell, opening: Opening) -> Port:
    wall = opening.shared_wall
    center = opening.center
    if wall.orientation == "vertical":
        if abs(wall.fixed_coord - cell.x_min) <= EPS:
            x = cell.x_min
        elif abs(wall.fixed_coord - cell.x_max) <= EPS:
            x = cell.x_max
        else:
            raise PassageGeometryError(
                f"Opening {opening.cell_a_id}-{opening.cell_b_id} not on a "
                f"vertical edge of cell {cell.id}"
            )
        return Port(x=x, y=center, kind="vertical", width=opening.width)

    if abs(wall.fixed_coord - cell.y_min) <= EPS:
        y = cell.y_min
    elif abs(wall.fixed_coord - cell.y_max) <= EPS:
        y = cell.y_max
    else:
        raise PassageGeometryError(
            f"Opening {opening.cell_a_id}-{opening.cell_b_id} not on a "
            f"horizontal edge of cell {cell.id}"
        )
    return Port(x=center, y=y, kind="horizontal", width=opening.width)


def _route_polyline(
    port_a: Port,
    port_b: Port,
    cell: Cell,
) -> list[tuple[float, float]]:
    mid_x = (cell.x_min + cell.x_max) / 2.0
    mid_y = (cell.y_min + cell.y_max) / 2.0

    if port_a.kind == "vertical" and port_b.kind == "vertical":
        if abs(port_a.y - port_b.y) <= EPS:
            return [(port_a.x, port_a.y), (port_b.x, port_b.y)]
        return [
            (port_a.x, port_a.y),
            (mid_x, port_a.y),
            (mid_x, port_b.y),
            (port_b.x, port_b.y),
        ]

    if port_a.kind == "horizontal" and port_b.kind == "horizontal":
        if abs(port_a.x - port_b.x) <= EPS:
            return [(port_a.x, port_a.y), (port_b.x, port_b.y)]
        return [
            (port_a.x, port_a.y),
            (port_a.x, mid_y),
            (port_b.x, mid_y),
            (port_b.x, port_b.y),
        ]

    vertical = port_a if port_a.kind == "vertical" else port_b
    horizontal = port_b if port_a.kind == "vertical" else port_a
    corner = (horizontal.x, vertical.y)
    return [
        (vertical.x, vertical.y),
        corner,
        (horizontal.x, horizontal.y),
    ]


def _grid_lines(
    corridor_geom: BaseGeometry,
    cell: Cell,
) -> tuple[list[float], list[float]]:
    xs = {cell.x_min, cell.x_max}
    ys = {cell.y_min, cell.y_max}

    for polygon in _iter_polygons(corridor_geom):
        for ring in [polygon.exterior, *polygon.interiors]:
            for x, y in ring.coords:
                xs.add(_clamp(x, cell.x_min, cell.x_max))
                ys.add(_clamp(y, cell.y_min, cell.y_max))

    return _unique_sorted(xs), _unique_sorted(ys)


def _geometry_to_rects(
    geom: BaseGeometry,
    xs: list[float],
    ys: list[float],
) -> list[Rect]:
    if geom.is_empty:
        return []

    prepared = prep(geom)
    n_cols = len(xs) - 1
    n_rows = len(ys) - 1
    present = [[False] * n_rows for _ in range(n_cols)]

    for i in range(n_cols):
        cx = (xs[i] + xs[i + 1]) / 2.0
        for j in range(n_rows):
            cy = (ys[j] + ys[j + 1]) / 2.0
            if prepared.contains(Point(cx, cy)):
                present[i][j] = True

    used = [[False] * n_rows for _ in range(n_cols)]
    rects: list[Rect] = []

    for i in range(n_cols):
        for j in range(n_rows):
            if not present[i][j] or used[i][j]:
                continue

            i2 = i
            while i2 + 1 < n_cols and present[i2 + 1][j] and not used[i2 + 1][j]:
                i2 += 1

            j2 = j
            expand = True
            while expand and j2 + 1 < n_rows:
                for ii in range(i, i2 + 1):
                    if not present[ii][j2 + 1] or used[ii][j2 + 1]:
                        expand = False
                        break
                if expand:
                    j2 += 1

            for ii in range(i, i2 + 1):
                for jj in range(j, j2 + 1):
                    used[ii][jj] = True

            rects.append(Rect(xs[i], ys[j], xs[i2 + 1], ys[j2 + 1]))

    return rects


def _iter_polygons(geom: BaseGeometry):
    if geom.is_empty:
        return
    if geom.geom_type == "Polygon":
        yield geom
    elif geom.geom_type in {"MultiPolygon", "GeometryCollection"}:
        for part in geom.geoms:
            if part.geom_type == "Polygon" and not part.is_empty:
                yield part


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _unique_sorted(values: set[float]) -> list[float]:
    ordered = sorted(values)
    result: list[float] = []
    for value in ordered:
        if not result or value - result[-1] > EPS:
            result.append(value)
    return result

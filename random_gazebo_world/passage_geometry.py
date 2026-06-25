from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
from typing import TYPE_CHECKING, Literal

from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from random_gazebo_world.config import Config
from random_gazebo_world.geometry import EPS, Cell
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
    width: float
    kind: PortKind | None = None


@dataclass(frozen=True)
class PassageCellGeometry:
    cell_id: int
    corridor: BaseGeometry
    solids: tuple[Polygon, ...]


@dataclass(frozen=True)
class PassageGeometryLayout:
    opening_layout: OpeningLayout
    cells: tuple[PassageCellGeometry, ...]

    @property
    def solids(self) -> tuple[Polygon, ...]:
        return tuple(solid for cell in self.cells for solid in cell.solids)

    @property
    def corridors(self) -> tuple[BaseGeometry, ...]:
        return tuple(cell.corridor for cell in self.cells)

    def corridor_for(self, cell_id: int) -> BaseGeometry | None:
        for cell in self.cells:
            if cell.cell_id == cell_id:
                return cell.corridor
        return None


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
        cell_geometries.append(_build_cell_geometry(cell, openings, config))

    geometry = PassageGeometryLayout(
        opening_layout=opening_layout,
        cells=tuple(cell_geometries),
    )
    validate_passage_geometry(geometry, config)
    return geometry


def _build_cell_geometry(
    cell: Cell,
    openings: tuple[Opening, ...],
    config: Config,
) -> PassageCellGeometry:
    if len(openings) < 2:
        raise PassageGeometryError(
            f"Passage cell {cell.id} has {len(openings)} openings, needs at least 2"
        )

    cell_polygon = cell.polygon
    if config.passage_geometry_mode == "legacy_orthogonal":
        ports = [_legacy_opening_port(cell, opening) for opening in openings]
        corridor = _build_legacy_orthogonal_corridor(cell, ports)
    else:
        ports = [_opening_port(cell, opening) for opening in openings]
        corridor = _build_curved_corridor(cell, ports)

    if corridor.is_empty or corridor.area <= EPS:
        raise PassageGeometryError(
            f"Passage cell {cell.id} produced an empty corridor"
        )

    leftover = cell_polygon.difference(corridor)
    solids = tuple(_iter_polygons(leftover))

    return PassageCellGeometry(
        cell_id=cell.id,
        corridor=corridor,
        solids=solids,
    )


def _build_curved_corridor(cell: Cell, ports: tuple[Port, ...] | list[Port]) -> BaseGeometry:
    cell_polygon = cell.polygon
    centroid = cell.centroid

    strips: list[BaseGeometry] = []
    for port in ports:
        polyline = LineString([(port.x, port.y), centroid])
        strips.append(
            polyline.buffer(port.width / 2.0, cap_style="round", join_style="round")
        )

    return unary_union(strips).intersection(cell_polygon)


def _build_legacy_orthogonal_corridor(
    cell: Cell,
    ports: tuple[Port, ...] | list[Port],
) -> BaseGeometry:
    cell_polygon = cell.polygon

    strips: list[BaseGeometry] = []
    for port_a, port_b in combinations(ports, 2):
        polyline = _route_polyline(port_a, port_b, cell)
        width = min(port_a.width, port_b.width)
        strips.append(
            LineString(polyline).buffer(
                width / 2.0, cap_style="flat", join_style="mitre"
            )
        )

    return unary_union(strips).intersection(cell_polygon)


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
        corridor = cell_geometry.corridor

        if corridor.is_empty:
            raise PassageGeometryError(f"Passage cell {cell.id} corridor is empty")
        if corridor.geom_type != "Polygon":
            raise PassageGeometryError(
                f"Passage cell {cell.id} corridor is not a single connected region"
            )

        for opening in openings_by_cell[cell_geometry.cell_id]:
            port = _opening_port(cell, opening)
            if corridor.distance(Point(port.x, port.y)) > 1e-6:
                raise PassageGeometryError(
                    f"Passage cell {cell.id} corridor does not reach opening "
                    f"{opening.cell_a_id}-{opening.cell_b_id}"
                )

        solid_area = sum(solid.area for solid in cell_geometry.solids)
        if abs(corridor.area + solid_area - cell.area) > 1e-4:
            raise PassageGeometryError(
                f"Passage cell {cell.id} corridor and solids do not tile the cell"
            )

        cell_polygon = cell.polygon
        for solid in cell_geometry.solids:
            if solid.difference(cell_polygon).area > 1e-6:
                raise PassageGeometryError(
                    f"Passage cell {cell.id} solid lies outside the cell"
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
    x, y = opening.midpoint
    return Port(x=x, y=y, width=opening.width)


def _legacy_opening_port(cell: Cell, opening: Opening) -> Port:
    wall = opening.shared_wall
    mx, my = opening.midpoint
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
        return Port(x=x, y=my, kind="vertical", width=opening.width)

    if abs(wall.fixed_coord - cell.y_min) <= EPS:
        y = cell.y_min
    elif abs(wall.fixed_coord - cell.y_max) <= EPS:
        y = cell.y_max
    else:
        raise PassageGeometryError(
            f"Opening {opening.cell_a_id}-{opening.cell_b_id} not on a "
            f"horizontal edge of cell {cell.id}"
        )
    return Port(x=mx, y=y, kind="horizontal", width=opening.width)


def _route_polyline(
    port_a: Port,
    port_b: Port,
    cell: Cell,
) -> list[tuple[float, float]]:
    if port_a.kind is None or port_b.kind is None:
        raise PassageGeometryError("Legacy routing requires axis-aligned port kinds")

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


def _iter_polygons(geom: BaseGeometry):
    if geom.is_empty:
        return
    if geom.geom_type == "Polygon":
        if geom.area > EPS:
            yield geom
    elif geom.geom_type in {"MultiPolygon", "GeometryCollection"}:
        for part in geom.geoms:
            if part.geom_type == "Polygon" and part.area > EPS:
                yield part

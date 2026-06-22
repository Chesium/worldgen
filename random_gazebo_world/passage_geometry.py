from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from random_gazebo_world.config import Config
from random_gazebo_world.geometry import EPS, Cell
from random_gazebo_world.openings import Opening, OpeningLayout

if TYPE_CHECKING:
    from random_gazebo_world.topology import AppliedLayout


class PassageGeometryError(RuntimeError):
    """Raised when passage corridor geometry cannot be built or is invalid."""


@dataclass(frozen=True)
class Port:
    """Connection point of an opening on a passage cell edge."""

    x: float
    y: float
    width: float


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
    cell_polygon = cell.polygon
    centroid = cell.centroid

    strips: list[BaseGeometry] = []
    for port in ports:
        polyline = LineString([(port.x, port.y), centroid])
        strips.append(
            polyline.buffer(port.width / 2.0, cap_style="round", join_style="round")
        )

    corridor = unary_union(strips).intersection(cell_polygon)
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

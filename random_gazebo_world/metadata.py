from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from shapely.geometry import Polygon
from shapely.geometry.base import BaseGeometry

from random_gazebo_world.config import Config
from random_gazebo_world.geometry import Cell, SharedWall, Vec2
from random_gazebo_world.openings import Opening, OpeningLayout
from random_gazebo_world.partition import Partition
from random_gazebo_world.passage_geometry import PassageGeometryLayout
from random_gazebo_world.topology import (
    AppliedLayout,
    CandidateConnection,
    ConnectionType,
    SelectedRoomGraph,
)
from random_gazebo_world.walls import WallLayout, WallSegment

SCHEMA_VERSION = 2


class MetadataError(RuntimeError):
    """Raised when layout/metadata serialization fails."""


@dataclass(frozen=True)
class LayoutDocument:
    partition: Partition
    cell_roles: tuple[tuple[int, str], ...]
    room_cell_ids: frozenset[int]
    passage_cell_ids: frozenset[int]
    connections: tuple[CandidateConnection, ...]
    openings: tuple[Opening, ...]
    wall_segments: tuple[WallSegment, ...]
    passage_corridors: tuple[BaseGeometry, ...] = ()
    passage_solids: tuple[Polygon, ...] = ()
    unused_solids: tuple[Polygon, ...] = ()


def build_layout_document(
    applied_layout: AppliedLayout,
    opening_layout: OpeningLayout,
    wall_layout: WallLayout,
    passage_geometry: PassageGeometryLayout | None = None,
) -> LayoutDocument:
    cell_roles = tuple(
        (cell.id, applied_layout.role_for(cell.id).value)
        for cell in applied_layout.partition.cells
    )
    corridors = passage_geometry.corridors if passage_geometry is not None else ()
    solids = passage_geometry.solids if passage_geometry is not None else ()
    return LayoutDocument(
        partition=applied_layout.partition,
        cell_roles=cell_roles,
        room_cell_ids=applied_layout.room_selection.room_cell_ids,
        passage_cell_ids=applied_layout.passage_cell_ids,
        connections=applied_layout.selected_graph.connections,
        openings=opening_layout.openings,
        wall_segments=wall_layout.segments,
        passage_corridors=corridors,
        passage_solids=solids,
        unused_solids=wall_layout.unused_solids,
    )


def export_layout_json(path: Path, document: LayoutDocument) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(layout_document_to_dict(document), handle, indent=2, sort_keys=True)
        handle.write("\n")


def load_layout_json(path: Path) -> LayoutDocument:
    with path.open(encoding="utf-8") as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise MetadataError(f"Layout root must be an object: {path}")
    return layout_document_from_dict(payload)


def export_metadata_json(
    path: Path,
    config: Config,
    document: LayoutDocument,
    selected_graph: SelectedRoomGraph,
) -> None:
    payload = {
        "seed": config.random_seed,
        "config": config_to_dict(config),
        "counts": {
            "cells": len(document.partition.cells),
            "rooms": len(document.room_cell_ids),
            "passage_cells": len(document.passage_cell_ids),
            "connections": len(document.connections),
            "openings": len(document.openings),
            "wall_segments": len(document.wall_segments),
            "passage_corridors": len(document.passage_corridors),
            "passage_solids": len(document.passage_solids),
            "unused_solids": len(document.unused_solids),
        },
        "generation_stats": {
            "gate_connections": sum(
                1
                for connection in document.connections
                if connection.connection_type is ConnectionType.GATE
            ),
            "passage_connections": sum(
                1
                for connection in document.connections
                if connection.connection_type is ConnectionType.PASSAGE
            ),
            "loop_connections": len(selected_graph.loop_connections),
            "spanning_tree_connections": len(selected_graph.spanning_tree_connections),
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def load_metadata_json(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise MetadataError(f"Metadata root must be an object: {path}")
    return payload


def layout_document_to_dict(document: LayoutDocument) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "world": {
            "width": document.partition.world_width,
            "height": document.partition.world_height,
        },
        "cells": [
            {
                "id": cell.id,
                "x_min": cell.x_min,
                "y_min": cell.y_min,
                "x_max": cell.x_max,
                "y_max": cell.y_max,
                "vertices": [list(point) for point in cell.polygon_vertices],
                "role": role,
            }
            for cell, role in zip(document.partition.cells, _roles_by_cell_id(document), strict=True)
        ],
        "room_cell_ids": sorted(document.room_cell_ids),
        "passage_cell_ids": sorted(document.passage_cell_ids),
        "connections": [
            connection_to_dict(connection) for connection in document.connections
        ],
        "openings": [opening_to_dict(opening) for opening in document.openings],
        "wall_segments": [
            wall_segment_to_dict(segment) for segment in document.wall_segments
        ],
        "passage_corridors": [
            polygon_to_dict(geom) for geom in document.passage_corridors
        ],
        "passage_solids": [polygon_to_dict(geom) for geom in document.passage_solids],
        "unused_solids": [polygon_to_dict(geom) for geom in document.unused_solids],
    }


def layout_document_from_dict(payload: dict[str, Any]) -> LayoutDocument:
    world = payload.get("world")
    cells_payload = payload.get("cells")
    if not isinstance(world, dict) or not isinstance(cells_payload, list):
        raise MetadataError("Layout must contain world and cells")

    cells: list[Cell] = []
    cell_roles: list[tuple[int, str]] = []
    for item in cells_payload:
        if not isinstance(item, dict):
            raise MetadataError("Each cell entry must be an object")
        cell = _cell_from_dict(item)
        cells.append(cell)
        cell_roles.append((cell.id, str(item["role"])))

    partition = Partition(
        cells=tuple(cells),
        world_width=float(world["width"]),
        world_height=float(world["height"]),
    )
    return LayoutDocument(
        partition=partition,
        cell_roles=tuple(cell_roles),
        room_cell_ids=frozenset(int(value) for value in payload.get("room_cell_ids", [])),
        passage_cell_ids=frozenset(
            int(value) for value in payload.get("passage_cell_ids", [])
        ),
        connections=tuple(
            connection_from_dict(item)
            for item in payload.get("connections", [])
        ),
        openings=tuple(opening_from_dict(item) for item in payload.get("openings", [])),
        wall_segments=tuple(
            wall_segment_from_dict(item) for item in payload.get("wall_segments", [])
        ),
        passage_corridors=tuple(
            polygon_from_dict(item) for item in payload.get("passage_corridors", [])
        ),
        passage_solids=tuple(
            polygon_from_dict(item) for item in payload.get("passage_solids", [])
        ),
        unused_solids=tuple(
            polygon_from_dict(item) for item in payload.get("unused_solids", [])
        ),
    )


def _cell_from_dict(item: dict[str, Any]) -> Cell:
    cell_id = int(item["id"])
    raw_vertices = item.get("vertices")
    if raw_vertices:
        vertices = tuple((float(x), float(y)) for x, y in raw_vertices)
        if _is_axis_aligned_rectangle(vertices):
            return Cell(
                id=cell_id,
                x_min=float(item["x_min"]),
                y_min=float(item["y_min"]),
                x_max=float(item["x_max"]),
                y_max=float(item["y_max"]),
            )
        return Cell.from_polygon(cell_id, vertices)
    return Cell(
        id=cell_id,
        x_min=float(item["x_min"]),
        y_min=float(item["y_min"]),
        x_max=float(item["x_max"]),
        y_max=float(item["y_max"]),
    )


def _is_axis_aligned_rectangle(vertices: tuple[Vec2, ...]) -> bool:
    if len(vertices) != 4:
        return False
    xs = {round(point[0], 9) for point in vertices}
    ys = {round(point[1], 9) for point in vertices}
    return len(xs) == 2 and len(ys) == 2


def config_to_dict(config: Config) -> dict[str, Any]:
    return asdict(config)


def config_from_dict(payload: dict[str, Any]) -> Config:
    config = Config(**payload)
    config.validate()
    return config


def connection_to_dict(connection: CandidateConnection) -> dict[str, Any]:
    return {
        "room_a_id": connection.room_a_id,
        "room_b_id": connection.room_b_id,
        "connection_type": connection.connection_type.value,
        "path_cell_ids": list(connection.path_cell_ids),
        "shared_wall": (
            shared_wall_to_dict(connection.shared_wall)
            if connection.shared_wall is not None
            else None
        ),
    }


def connection_from_dict(payload: dict[str, Any]) -> CandidateConnection:
    shared_wall_payload = payload.get("shared_wall")
    return CandidateConnection(
        room_a_id=int(payload["room_a_id"]),
        room_b_id=int(payload["room_b_id"]),
        connection_type=ConnectionType(str(payload["connection_type"])),
        shared_wall=(
            shared_wall_from_dict(shared_wall_payload)
            if shared_wall_payload is not None
            else None
        ),
        path_cell_ids=tuple(int(value) for value in payload.get("path_cell_ids", [])),
    )


def opening_to_dict(opening: Opening) -> dict[str, Any]:
    return {
        "cell_a_id": opening.cell_a_id,
        "cell_b_id": opening.cell_b_id,
        "kind": opening.kind,
        "width": opening.width,
        "span_start": opening.span_start,
        "span_end": opening.span_end,
        "shared_wall": shared_wall_to_dict(opening.shared_wall),
    }


def opening_from_dict(payload: dict[str, Any]) -> Opening:
    return Opening(
        cell_a_id=int(payload["cell_a_id"]),
        cell_b_id=int(payload["cell_b_id"]),
        shared_wall=shared_wall_from_dict(payload["shared_wall"]),
        kind=str(payload["kind"]),
        width=float(payload["width"]),
        span_start=float(payload["span_start"]),
        span_end=float(payload["span_end"]),
    )


def polygon_to_dict(geometry: BaseGeometry) -> dict[str, Any]:
    if geometry.is_empty or geometry.geom_type != "Polygon":
        return {"vertices": []}
    coords = list(geometry.exterior.coords)[:-1]
    return {"vertices": [[float(x), float(y)] for x, y in coords]}


def polygon_from_dict(payload: dict[str, Any]) -> Polygon:
    vertices = payload.get("vertices", [])
    return Polygon([(float(x), float(y)) for x, y in vertices])


def wall_segment_to_dict(segment: WallSegment) -> dict[str, Any]:
    return {
        "p1": list(segment.p1),
        "p2": list(segment.p2),
    }


def wall_segment_from_dict(payload: dict[str, Any]) -> WallSegment:
    return WallSegment(
        p1=(float(payload["p1"][0]), float(payload["p1"][1])),
        p2=(float(payload["p2"][0]), float(payload["p2"][1])),
    )


def shared_wall_to_dict(shared_wall: SharedWall) -> dict[str, Any]:
    return {
        "p1": list(shared_wall.p1),
        "p2": list(shared_wall.p2),
    }


def shared_wall_from_dict(payload: dict[str, Any]) -> SharedWall:
    return SharedWall(
        p1=(float(payload["p1"][0]), float(payload["p1"][1])),
        p2=(float(payload["p2"][0]), float(payload["p2"][1])),
    )


def _roles_by_cell_id(document: LayoutDocument) -> list[str]:
    role_map = dict(document.cell_roles)
    return [role_map[cell.id] for cell in document.partition.cells]

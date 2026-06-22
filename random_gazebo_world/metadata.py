from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from random_gazebo_world.config import Config
from random_gazebo_world.geometry import Cell, SharedWall
from random_gazebo_world.openings import Opening, OpeningLayout
from random_gazebo_world.partition import Partition
from random_gazebo_world.topology import (
    AppliedLayout,
    CandidateConnection,
    ConnectionType,
    SelectedRoomGraph,
)
from random_gazebo_world.walls import WallLayout, WallSegment


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


def build_layout_document(
    applied_layout: AppliedLayout,
    opening_layout: OpeningLayout,
    wall_layout: WallLayout,
) -> LayoutDocument:
    cell_roles = tuple(
        (cell.id, applied_layout.role_for(cell.id).value)
        for cell in applied_layout.partition.cells
    )
    return LayoutDocument(
        partition=applied_layout.partition,
        cell_roles=cell_roles,
        room_cell_ids=applied_layout.room_selection.room_cell_ids,
        passage_cell_ids=applied_layout.passage_cell_ids,
        connections=applied_layout.selected_graph.connections,
        openings=opening_layout.openings,
        wall_segments=wall_layout.segments,
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
        cell = Cell(
            id=int(item["id"]),
            x_min=float(item["x_min"]),
            y_min=float(item["y_min"]),
            x_max=float(item["x_max"]),
            y_max=float(item["y_max"]),
        )
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
    )


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


def wall_segment_to_dict(segment: WallSegment) -> dict[str, Any]:
    return {
        "orientation": segment.orientation,
        "fixed_coord": segment.fixed_coord,
        "span_start": segment.span_start,
        "span_end": segment.span_end,
    }


def wall_segment_from_dict(payload: dict[str, Any]) -> WallSegment:
    return WallSegment(
        orientation=str(payload["orientation"]),
        fixed_coord=float(payload["fixed_coord"]),
        span_start=float(payload["span_start"]),
        span_end=float(payload["span_end"]),
    )


def shared_wall_to_dict(shared_wall: SharedWall) -> dict[str, Any]:
    return {
        "orientation": shared_wall.orientation,
        "fixed_coord": shared_wall.fixed_coord,
        "span_start": shared_wall.span_start,
        "span_end": shared_wall.span_end,
    }


def shared_wall_from_dict(payload: dict[str, Any]) -> SharedWall:
    return SharedWall(
        orientation=str(payload["orientation"]),
        fixed_coord=float(payload["fixed_coord"]),
        span_start=float(payload["span_start"]),
        span_end=float(payload["span_end"]),
    )


def _roles_by_cell_id(document: LayoutDocument) -> list[str]:
    role_map = dict(document.cell_roles)
    return [role_map[cell.id] for cell in document.partition.cells]

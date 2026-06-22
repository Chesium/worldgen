from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from random_gazebo_world.adjacency import AdjacencyGraph
from random_gazebo_world.config import Config
from random_gazebo_world.geometry import EPS, Rect, SharedWall
from random_gazebo_world.openings import Opening, OpeningLayout
from random_gazebo_world.passage_geometry import PassageGeometryLayout
from random_gazebo_world.topology import CellRole


class WallGenerationError(RuntimeError):
    """Raised when wall segments are invalid."""


@dataclass(frozen=True)
class WallSegment:
    orientation: Literal["vertical", "horizontal"]
    fixed_coord: float
    span_start: float
    span_end: float

    @property
    def length(self) -> float:
        return self.span_end - self.span_start


@dataclass(frozen=True)
class WallLayout:
    opening_layout: OpeningLayout
    segments: tuple[WallSegment, ...]
    passage_geometry: PassageGeometryLayout | None = None
    unused_solids: tuple[Rect, ...] = ()


def generate_walls(
    opening_layout: OpeningLayout,
    adjacency: AdjacencyGraph,
    config: Config,
    passage_geometry: PassageGeometryLayout | None = None,
) -> WallLayout:
    layout = opening_layout.applied_layout
    openings_by_pair = _group_openings_by_cell_pair(opening_layout.openings)
    segments: list[WallSegment] = []

    for edge in adjacency.edges:
        left_id = edge.cell_a_id
        right_id = edge.cell_b_id
        role_left = layout.role_for(left_id)
        role_right = layout.role_for(right_id)

        if not _should_generate_interior_wall(role_left, role_right):
            continue

        pair = (min(left_id, right_id), max(left_id, right_id))
        openings = openings_by_pair.get(pair, ())
        segments.extend(
            _wall_segments_from_shared_wall(
                edge.shared_wall,
                openings,
                config.wall_thickness,
            )
        )

    for cell in layout.partition.cells:
        if layout.role_for(cell.id) == CellRole.UNUSED:
            continue
        segments.extend(
            _exterior_wall_segments(
                cell,
                layout.partition.world_width,
                layout.partition.world_height,
                config.wall_thickness,
            )
        )

    unused_solids = _unused_cell_solids(layout)

    wall_layout = WallLayout(
        opening_layout=opening_layout,
        segments=tuple(segments),
        passage_geometry=passage_geometry,
        unused_solids=unused_solids,
    )
    validate_wall_layout(wall_layout, config)
    return wall_layout


def validate_wall_layout(wall_layout: WallLayout, config: Config) -> None:
    min_length = config.wall_thickness

    for segment in wall_layout.segments:
        if segment.length + EPS < min_length:
            raise WallGenerationError(
                f"Wall segment length {segment.length} below threshold {min_length}"
            )

    for opening in wall_layout.opening_layout.openings:
        for segment in wall_layout.segments:
            if _segment_overlaps_opening(segment, opening):
                raise WallGenerationError(
                    f"Wall segment overlaps opening "
                    f"{opening.cell_a_id}-{opening.cell_b_id}"
                )


def wall_segment_line(
    segment: WallSegment,
) -> tuple[tuple[float, float], tuple[float, float]]:
    if segment.orientation == "vertical":
        return (
            (segment.fixed_coord, segment.span_start),
            (segment.fixed_coord, segment.span_end),
        )
    return (
        (segment.span_start, segment.fixed_coord),
        (segment.span_end, segment.fixed_coord),
    )


def _should_generate_interior_wall(role_left: CellRole, role_right: CellRole) -> bool:
    if role_left == CellRole.UNUSED or role_right == CellRole.UNUSED:
        return False
    if role_left == CellRole.PASSAGE and role_right == CellRole.PASSAGE:
        return False
    return True


def _unused_cell_solids(layout) -> tuple[Rect, ...]:
    return tuple(
        Rect(cell.x_min, cell.y_min, cell.x_max, cell.y_max)
        for cell in layout.partition.cells
        if layout.role_for(cell.id) == CellRole.UNUSED
    )


def _group_openings_by_cell_pair(
    openings: tuple[Opening, ...],
) -> dict[tuple[int, int], tuple[Opening, ...]]:
    grouped: dict[tuple[int, int], list[Opening]] = {}
    for opening in openings:
        pair = (opening.cell_a_id, opening.cell_b_id)
        grouped.setdefault(pair, []).append(opening)
    return {pair: tuple(items) for pair, items in grouped.items()}


def _wall_segments_from_shared_wall(
    shared_wall: SharedWall,
    openings: tuple[Opening, ...],
    min_length: float,
) -> list[WallSegment]:
    relevant_openings = [
        opening
        for opening in openings
        if _opening_on_wall(opening, shared_wall)
    ]
    intervals = [(shared_wall.span_start, shared_wall.span_end)]

    for opening in sorted(relevant_openings, key=lambda item: item.span_start):
        next_intervals: list[tuple[float, float]] = []
        for start, end in intervals:
            if opening.span_end <= start + EPS or opening.span_start >= end - EPS:
                next_intervals.append((start, end))
                continue
            if start + EPS < opening.span_start:
                next_intervals.append((start, opening.span_start))
            if opening.span_end + EPS < end:
                next_intervals.append((opening.span_end, end))
        intervals = next_intervals

    segments: list[WallSegment] = []
    for start, end in intervals:
        if end - start + EPS < min_length:
            continue
        segments.append(
            WallSegment(
                orientation=shared_wall.orientation,
                fixed_coord=shared_wall.fixed_coord,
                span_start=start,
                span_end=end,
            )
        )
    return segments


def _opening_on_wall(opening: Opening, shared_wall: SharedWall) -> bool:
    if opening.shared_wall.orientation != shared_wall.orientation:
        return False
    if abs(opening.shared_wall.fixed_coord - shared_wall.fixed_coord) > EPS:
        return False
    return (
        opening.span_start >= shared_wall.span_start - EPS
        and opening.span_end <= shared_wall.span_end + EPS
    )


def _exterior_wall_segments(
    cell,
    world_width: float,
    world_height: float,
    min_length: float,
) -> list[WallSegment]:
    segments: list[WallSegment] = []

    if abs(cell.x_min) <= EPS:
        segments.extend(
            _maybe_segment("vertical", cell.x_min, cell.y_min, cell.y_max, min_length)
        )
    if abs(cell.x_max - world_width) <= EPS:
        segments.extend(
            _maybe_segment("vertical", cell.x_max, cell.y_min, cell.y_max, min_length)
        )
    if abs(cell.y_min) <= EPS:
        segments.extend(
            _maybe_segment("horizontal", cell.y_min, cell.x_min, cell.x_max, min_length)
        )
    if abs(cell.y_max - world_height) <= EPS:
        segments.extend(
            _maybe_segment("horizontal", cell.y_max, cell.x_min, cell.x_max, min_length)
        )

    return segments


def _maybe_segment(
    orientation: Literal["vertical", "horizontal"],
    fixed_coord: float,
    span_start: float,
    span_end: float,
    min_length: float,
) -> list[WallSegment]:
    if span_end - span_start + EPS < min_length:
        return []
    return [
        WallSegment(
            orientation=orientation,
            fixed_coord=fixed_coord,
            span_start=span_start,
            span_end=span_end,
        )
    ]


def _segment_overlaps_opening(segment: WallSegment, opening: Opening) -> bool:
    if segment.orientation != opening.shared_wall.orientation:
        return False
    if abs(segment.fixed_coord - opening.shared_wall.fixed_coord) > EPS:
        return False

    overlap_start = max(segment.span_start, opening.span_start)
    overlap_end = min(segment.span_end, opening.span_end)
    return overlap_end - overlap_start > EPS

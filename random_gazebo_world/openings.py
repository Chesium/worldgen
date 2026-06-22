from __future__ import annotations

import random
from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal

from random_gazebo_world.config import Config
from random_gazebo_world.geometry import EPS, SharedWall

if TYPE_CHECKING:
    from random_gazebo_world.topology import AppliedLayout

OpeningKind = Literal["gate", "passage"]


class OpeningError(RuntimeError):
    """Raised when an opening cannot be placed on a shared wall."""


@dataclass(frozen=True)
class LogicalOpening:
    """Placeholder opening between two adjacent cells along a selected connection."""

    cell_a_id: int
    cell_b_id: int
    shared_wall: SharedWall
    kind: OpeningKind

    def __post_init__(self) -> None:
        if self.cell_a_id > self.cell_b_id:
            raise ValueError("cell_a_id must be <= cell_b_id for stable ordering")


@dataclass(frozen=True)
class Opening:
    """Concrete doorway opening placed on a shared wall segment.

    ``span_start`` and ``span_end`` are arc-length offsets along the shared wall
    measured from ``shared_wall.p1`` (both in ``[0, shared_wall.length]``).
    """

    cell_a_id: int
    cell_b_id: int
    shared_wall: SharedWall
    kind: OpeningKind
    width: float
    span_start: float
    span_end: float

    @property
    def center(self) -> float:
        return (self.span_start + self.span_end) / 2.0

    @property
    def midpoint(self) -> tuple[float, float]:
        return self.shared_wall.point_at_arc_length(self.center)

    def endpoints(self) -> tuple[tuple[float, float], tuple[float, float]]:
        return (
            self.shared_wall.point_at_arc_length(self.span_start),
            self.shared_wall.point_at_arc_length(self.span_end),
        )


@dataclass(frozen=True)
class OpeningLayout:
    applied_layout: AppliedLayout
    openings: tuple[Opening, ...]


def generate_openings(
    applied_layout: AppliedLayout,
    config: Config,
    rng: random.Random,
) -> OpeningLayout:
    openings: list[Opening] = []

    for logical_opening in applied_layout.logical_openings:
        if logical_opening.kind == "gate":
            width_min = config.gate_width_min
            width_max = config.gate_width_max
        else:
            width_min = config.passage_width_min
            width_max = config.passage_width_max

        openings.append(
            place_opening(logical_opening, width_min, width_max, rng)
        )

    opening_layout = OpeningLayout(
        applied_layout=applied_layout,
        openings=tuple(openings),
    )
    validate_openings(opening_layout, config)
    return opening_layout


def place_opening(
    logical_opening: LogicalOpening,
    width_min: float,
    width_max: float,
    rng: random.Random,
) -> Opening:
    wall = logical_opening.shared_wall
    length = wall.length
    if length + EPS < width_min:
        raise OpeningError(
            f"Shared wall {logical_opening.cell_a_id}-{logical_opening.cell_b_id} "
            f"length {length} is shorter than minimum width {width_min}"
        )

    max_width = min(width_max, length)
    width = rng.uniform(width_min, max_width)

    min_center = width / 2.0
    max_center = length - width / 2.0
    if min_center > max_center + EPS:
        raise OpeningError(
            f"Cannot fit opening width {width} on shared wall "
            f"{logical_opening.cell_a_id}-{logical_opening.cell_b_id}"
        )

    center = rng.uniform(min_center, max_center)
    span_start = center - width / 2.0
    span_end = center + width / 2.0

    return Opening(
        cell_a_id=logical_opening.cell_a_id,
        cell_b_id=logical_opening.cell_b_id,
        shared_wall=wall,
        kind=logical_opening.kind,
        width=width,
        span_start=span_start,
        span_end=span_end,
    )


def validate_openings(opening_layout: OpeningLayout, config: Config) -> None:
    for opening in opening_layout.openings:
        if opening.kind == "gate":
            width_min = config.gate_width_min
            width_max = config.gate_width_max
        else:
            width_min = config.passage_width_min
            width_max = config.passage_width_max

        if opening.width + EPS < width_min or opening.width > width_max + EPS:
            raise OpeningError(
                f"Opening {opening.cell_a_id}-{opening.cell_b_id} width "
                f"{opening.width} outside [{width_min}, {width_max}]"
            )

        wall = opening.shared_wall
        if opening.span_start < -EPS:
            raise OpeningError(
                f"Opening {opening.cell_a_id}-{opening.cell_b_id} starts before wall"
            )
        if opening.span_end > wall.length + EPS:
            raise OpeningError(
                f"Opening {opening.cell_a_id}-{opening.cell_b_id} ends after wall"
            )
        if opening.span_end - opening.span_start + EPS < opening.width:
            raise OpeningError(
                f"Opening {opening.cell_a_id}-{opening.cell_b_id} span shorter than width"
            )


def opening_line(opening: Opening) -> tuple[tuple[float, float], tuple[float, float]]:
    return opening.endpoints()

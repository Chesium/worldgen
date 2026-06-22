from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from random_gazebo_world.geometry import SharedWall

OpeningKind = Literal["gate", "passage"]


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

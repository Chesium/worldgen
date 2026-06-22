from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from random_gazebo_world.adjacency import AdjacencyGraph
from random_gazebo_world.geometry import SharedWall
from random_gazebo_world.partition import Partition
from random_gazebo_world.topology import RoomSelection


def _setup_axes(
    ax: plt.Axes,
    world_width: float,
    world_height: float,
    title: str,
) -> None:
    ax.set_xlim(0.0, world_width)
    ax.set_ylim(0.0, world_height)
    ax.set_aspect("equal")
    ax.set_title(title)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")


def _save_figure(fig: plt.Figure, output_base: Path) -> None:
    output_base.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_base.with_suffix(".png"), dpi=150)
    fig.savefig(output_base.with_suffix(".svg"))
    plt.close(fig)


def render_partition(
    partition: Partition,
    output_base: Path,
    title: str = "BSP Partition",
) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))
    colors = plt.get_cmap("tab20")

    for index, cell in enumerate(partition.cells):
        color = colors(index % 20)
        ax.add_patch(
            Rectangle(
                (cell.x_min, cell.y_min),
                cell.width,
                cell.height,
                facecolor=color,
                edgecolor="black",
                linewidth=1.0,
                alpha=0.65,
            )
        )
        ax.text(
            cell.x_min + cell.width / 2.0,
            cell.y_min + cell.height / 2.0,
            str(cell.id),
            ha="center",
            va="center",
            fontsize=9,
            color="black",
            weight="bold",
        )

    _setup_axes(ax, partition.world_width, partition.world_height, title)
    fig.tight_layout()
    _save_figure(fig, output_base)


def _shared_wall_line(shared_wall: SharedWall) -> tuple[tuple[float, float], tuple[float, float]]:
    if shared_wall.orientation == "vertical":
        return (
            (shared_wall.fixed_coord, shared_wall.span_start),
            (shared_wall.fixed_coord, shared_wall.span_end),
        )
    return (
        (shared_wall.span_start, shared_wall.fixed_coord),
        (shared_wall.span_end, shared_wall.fixed_coord),
    )


def render_adjacency_graph(
    partition: Partition,
    adjacency: AdjacencyGraph,
    output_base: Path,
    title: str = "Cell Adjacency Graph",
) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))
    colors = plt.get_cmap("Pastel1")

    for index, cell in enumerate(partition.cells):
        color = colors(index % 9)
        ax.add_patch(
            Rectangle(
                (cell.x_min, cell.y_min),
                cell.width,
                cell.height,
                facecolor=color,
                edgecolor="black",
                linewidth=0.8,
                alpha=0.8,
            )
        )
        ax.text(
            cell.x_min + cell.width / 2.0,
            cell.y_min + cell.height / 2.0,
            str(cell.id),
            ha="center",
            va="center",
            fontsize=9,
            color="black",
            weight="bold",
        )

    for edge in adjacency.edges:
        start, end = _shared_wall_line(edge.shared_wall)
        ax.plot(
            [start[0], end[0]],
            [start[1], end[1]],
            color="crimson",
            linewidth=3.0,
            solid_capstyle="round",
            zorder=3,
        )

        cell_a = adjacency.cell_by_id(edge.cell_a_id)
        cell_b = adjacency.cell_by_id(edge.cell_b_id)
        ax.plot(
            [
                cell_a.x_min + cell_a.width / 2.0,
                cell_b.x_min + cell_b.width / 2.0,
            ],
            [
                cell_a.y_min + cell_a.height / 2.0,
                cell_b.y_min + cell_b.height / 2.0,
            ],
            color="navy",
            linewidth=1.0,
            linestyle="--",
            alpha=0.7,
            zorder=2,
        )

    _setup_axes(ax, partition.world_width, partition.world_height, title)
    fig.tight_layout()
    _save_figure(fig, output_base)


def render_selected_rooms(
    partition: Partition,
    selection: RoomSelection,
    output_base: Path,
    title: str = "Selected Rooms",
) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))

    for cell in selection.unused_cells():
        ax.add_patch(
            Rectangle(
                (cell.x_min, cell.y_min),
                cell.width,
                cell.height,
                facecolor="#d9d9d9",
                edgecolor="#666666",
                linewidth=1.0,
                alpha=0.9,
            )
        )
        ax.text(
            cell.x_min + cell.width / 2.0,
            cell.y_min + cell.height / 2.0,
            f"{cell.id}\nunused",
            ha="center",
            va="center",
            fontsize=8,
            color="#444444",
        )

    for cell in selection.room_cells():
        ax.add_patch(
            Rectangle(
                (cell.x_min, cell.y_min),
                cell.width,
                cell.height,
                facecolor="#7bd389",
                edgecolor="#1f7a3a",
                linewidth=1.5,
                alpha=0.9,
            )
        )
        ax.text(
            cell.x_min + cell.width / 2.0,
            cell.y_min + cell.height / 2.0,
            f"{cell.id}\nroom",
            ha="center",
            va="center",
            fontsize=9,
            color="#12351f",
            weight="bold",
        )

    _setup_axes(ax, partition.world_width, partition.world_height, title)
    fig.tight_layout()
    _save_figure(fig, output_base)

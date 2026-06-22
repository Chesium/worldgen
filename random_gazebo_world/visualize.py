from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from random_gazebo_world.adjacency import AdjacencyGraph
from random_gazebo_world.geometry import SharedWall
from random_gazebo_world.openings import OpeningLayout, opening_line
from random_gazebo_world.partition import Partition
from random_gazebo_world.topology import (
    AppliedLayout,
    CandidateConnections,
    ConnectionType,
    RoomSelection,
    SelectedRoomGraph,
)
from random_gazebo_world.walls import WallLayout, wall_segment_line


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


def _cell_centroid(cell) -> tuple[float, float]:
    return (
        cell.x_min + cell.width / 2.0,
        cell.y_min + cell.height / 2.0,
    )


def render_candidate_connections(
    partition: Partition,
    room_selection: RoomSelection,
    candidates: CandidateConnections,
    output_base: Path,
    title: str = "Candidate Connections",
) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))
    cells_by_id = {cell.id: cell for cell in partition.cells}

    for cell in room_selection.unused_cells():
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

    for cell in room_selection.room_cells():
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
            str(cell.id),
            ha="center",
            va="center",
            fontsize=9,
            color="#12351f",
            weight="bold",
        )

    for connection in candidates.connections:
        if connection.connection_type == ConnectionType.GATE:
            assert connection.shared_wall is not None
            start, end = _shared_wall_line(connection.shared_wall)
            ax.plot(
                [start[0], end[0]],
                [start[1], end[1]],
                color="#d4a017",
                linewidth=4.0,
                solid_capstyle="round",
                zorder=4,
            )
            continue

        path_cells = [cells_by_id[cell_id] for cell_id in connection.path_cell_ids]
        xs, ys = zip(*(_cell_centroid(cell) for cell in path_cells))
        ax.plot(
            xs,
            ys,
            color="#7b2cbf",
            linewidth=2.5,
            linestyle="-",
            marker="o",
            markersize=5,
            zorder=4,
        )

    _setup_axes(ax, partition.world_width, partition.world_height, title)
    fig.tight_layout()
    _save_figure(fig, output_base)


def _draw_room_layout_base(
    ax: plt.Axes,
    room_selection: RoomSelection,
) -> None:
    for cell in room_selection.unused_cells():
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

    for cell in room_selection.room_cells():
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
            str(cell.id),
            ha="center",
            va="center",
            fontsize=9,
            color="#12351f",
            weight="bold",
        )


def _draw_connection(
    ax: plt.Axes,
    partition: Partition,
    connection,
    *,
    color: str,
    linewidth: float,
    linestyle: str,
) -> None:
    cells_by_id = {cell.id: cell for cell in partition.cells}
    if connection.connection_type == ConnectionType.GATE:
        assert connection.shared_wall is not None
        start, end = _shared_wall_line(connection.shared_wall)
        ax.plot(
            [start[0], end[0]],
            [start[1], end[1]],
            color=color,
            linewidth=linewidth,
            linestyle=linestyle,
            solid_capstyle="round",
            zorder=4,
        )
        return

    path_cells = [cells_by_id[cell_id] for cell_id in connection.path_cell_ids]
    xs, ys = zip(*(_cell_centroid(cell) for cell in path_cells))
    ax.plot(
        xs,
        ys,
        color=color,
        linewidth=linewidth,
        linestyle=linestyle,
        marker="o",
        markersize=5,
        zorder=4,
    )


def render_selected_room_graph(
    partition: Partition,
    selected: SelectedRoomGraph,
    output_base: Path,
    title: str = "Selected Room Graph",
) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))
    _draw_room_layout_base(ax, selected.room_selection)

    loop_pairs = selected.loop_connection_pairs
    for connection in selected.connections:
        pair = (connection.room_a_id, connection.room_b_id)
        is_loop = pair in loop_pairs
        _draw_connection(
            ax,
            partition,
            connection,
            color="#ff7f0e" if is_loop else "#1f4e79",
            linewidth=3.5 if is_loop else 3.0,
            linestyle="--" if is_loop else "-",
        )

    _setup_axes(ax, partition.world_width, partition.world_height, title)
    fig.tight_layout()
    _save_figure(fig, output_base)


def render_passage_cells(
    layout: AppliedLayout,
    output_base: Path,
    title: str = "Passage Cells",
) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))
    partition = layout.partition

    for cell in partition.cells:
        role = layout.role_for(cell.id)
        if role.value == "room":
            facecolor = "#7bd389"
            edgecolor = "#1f7a3a"
            label = f"{cell.id}\nroom"
            text_color = "#12351f"
            weight = "bold"
        elif role.value == "passage":
            facecolor = "#8ecae6"
            edgecolor = "#219ebc"
            label = f"{cell.id}\npassage"
            text_color = "#023047"
            weight = "bold"
        else:
            facecolor = "#d9d9d9"
            edgecolor = "#666666"
            label = f"{cell.id}\nunused"
            text_color = "#444444"
            weight = "normal"

        ax.add_patch(
            Rectangle(
                (cell.x_min, cell.y_min),
                cell.width,
                cell.height,
                facecolor=facecolor,
                edgecolor=edgecolor,
                linewidth=1.2,
                alpha=0.9,
            )
        )
        ax.text(
            cell.x_min + cell.width / 2.0,
            cell.y_min + cell.height / 2.0,
            label,
            ha="center",
            va="center",
            fontsize=8,
            color=text_color,
            weight=weight,
        )

    _setup_axes(ax, partition.world_width, partition.world_height, title)
    fig.tight_layout()
    _save_figure(fig, output_base)


def render_openings(
    opening_layout: OpeningLayout,
    output_base: Path,
    title: str = "Openings",
) -> None:
    layout = opening_layout.applied_layout
    partition = layout.partition
    fig, ax = plt.subplots(figsize=(8, 8))

    for cell in partition.cells:
        role = layout.role_for(cell.id)
        if role.value == "room":
            facecolor = "#7bd389"
            edgecolor = "#1f7a3a"
        elif role.value == "passage":
            facecolor = "#8ecae6"
            edgecolor = "#219ebc"
        else:
            facecolor = "#d9d9d9"
            edgecolor = "#666666"

        ax.add_patch(
            Rectangle(
                (cell.x_min, cell.y_min),
                cell.width,
                cell.height,
                facecolor=facecolor,
                edgecolor=edgecolor,
                linewidth=1.0,
                alpha=0.85,
            )
        )

    for logical_opening in layout.logical_openings:
        start, end = _shared_wall_line(logical_opening.shared_wall)
        ax.plot(
            [start[0], end[0]],
            [start[1], end[1]],
            color="#bbbbbb",
            linewidth=2.0,
            solid_capstyle="butt",
            zorder=2,
        )

    for opening in opening_layout.openings:
        start, end = opening_line(opening)
        color = "#d4a017" if opening.kind == "gate" else "#7b2cbf"
        ax.plot(
            [start[0], end[0]],
            [start[1], end[1]],
            color=color,
            linewidth=5.0,
            solid_capstyle="round",
            zorder=4,
        )

    _setup_axes(ax, partition.world_width, partition.world_height, title)
    fig.tight_layout()
    _save_figure(fig, output_base)


def render_passage_geometry(
    wall_layout: WallLayout,
    output_base: Path,
    title: str = "Passage Geometry",
) -> None:
    layout = wall_layout.opening_layout.applied_layout
    partition = layout.partition
    passage_geometry = wall_layout.passage_geometry
    fig, ax = plt.subplots(figsize=(8, 8))

    for cell in partition.cells:
        role = layout.role_for(cell.id)
        if role.value == "room":
            facecolor = "#eef8f0"
        elif role.value == "passage":
            facecolor = "#fbeeee"
        else:
            facecolor = "#f5f5f5"

        ax.add_patch(
            Rectangle(
                (cell.x_min, cell.y_min),
                cell.width,
                cell.height,
                facecolor=facecolor,
                edgecolor="#cccccc",
                linewidth=0.5,
                alpha=0.9,
            )
        )

    if passage_geometry is not None:
        for cell_geometry in passage_geometry.cells:
            for rect in cell_geometry.solids:
                ax.add_patch(
                    Rectangle(
                        (rect.x_min, rect.y_min),
                        rect.width,
                        rect.height,
                        facecolor="#444444",
                        edgecolor="#222222",
                        linewidth=0.5,
                        alpha=0.85,
                        zorder=3,
                    )
                )
            for rect in cell_geometry.corridor:
                ax.add_patch(
                    Rectangle(
                        (rect.x_min, rect.y_min),
                        rect.width,
                        rect.height,
                        facecolor="#8ecae6",
                        edgecolor="#219ebc",
                        linewidth=0.5,
                        alpha=0.95,
                        zorder=4,
                    )
                )

    for segment in wall_layout.segments:
        start, end = wall_segment_line(segment)
        ax.plot(
            [start[0], end[0]],
            [start[1], end[1]],
            color="#111111",
            linewidth=2.0,
            solid_capstyle="butt",
            zorder=5,
        )

    _setup_axes(ax, partition.world_width, partition.world_height, title)
    fig.tight_layout()
    _save_figure(fig, output_base)


def render_wall_segments(
    wall_layout: WallLayout,
    output_base: Path,
    title: str = "Wall Segments",
) -> None:
    layout = wall_layout.opening_layout.applied_layout
    partition = layout.partition
    fig, ax = plt.subplots(figsize=(8, 8))

    for cell in partition.cells:
        role = layout.role_for(cell.id)
        if role.value == "room":
            facecolor = "#eef8f0"
        elif role.value == "passage":
            facecolor = "#eef7fb"
        else:
            facecolor = "#f5f5f5"

        ax.add_patch(
            Rectangle(
                (cell.x_min, cell.y_min),
                cell.width,
                cell.height,
                facecolor=facecolor,
                edgecolor="#cccccc",
                linewidth=0.5,
                alpha=0.9,
            )
        )

    for segment in wall_layout.segments:
        start, end = wall_segment_line(segment)
        ax.plot(
            [start[0], end[0]],
            [start[1], end[1]],
            color="#111111",
            linewidth=3.0,
            solid_capstyle="butt",
            zorder=5,
        )

    _setup_axes(ax, partition.world_width, partition.world_height, title)
    fig.tight_layout()
    _save_figure(fig, output_base)


def render_final_floorplan(
    wall_layout: WallLayout,
    output_base: Path,
    title: str = "Final Floor Plan",
) -> None:
    opening_layout = wall_layout.opening_layout
    layout = opening_layout.applied_layout
    partition = layout.partition
    passage_geometry = wall_layout.passage_geometry
    cells_by_id = {cell.id: cell for cell in partition.cells}
    fig, ax = plt.subplots(figsize=(8, 8))

    for cell in partition.cells:
        role = layout.role_for(cell.id)
        if role.value == "room":
            facecolor = "#7bd389"
            edgecolor = "#1f7a3a"
            label = str(cell.id)
        elif role.value == "passage":
            facecolor = "#ececec" if passage_geometry is not None else "#8ecae6"
            edgecolor = "#aaaaaa" if passage_geometry is not None else "#219ebc"
            label = str(cell.id)
        else:
            facecolor = "#ececec"
            edgecolor = "#aaaaaa"
            label = ""

        ax.add_patch(
            Rectangle(
                (cell.x_min, cell.y_min),
                cell.width,
                cell.height,
                facecolor=facecolor,
                edgecolor=edgecolor,
                linewidth=1.0,
                alpha=0.95,
            )
        )
        if label:
            ax.text(
                cell.x_min + cell.width / 2.0,
                cell.y_min + cell.height / 2.0,
                label,
                ha="center",
                va="center",
                fontsize=9,
                color="#12351f",
                weight="bold",
            )

    if passage_geometry is not None:
        for cell_geometry in passage_geometry.cells:
            for rect in cell_geometry.corridor:
                ax.add_patch(
                    Rectangle(
                        (rect.x_min, rect.y_min),
                        rect.width,
                        rect.height,
                        facecolor="#8ecae6",
                        edgecolor="none",
                        alpha=0.95,
                        zorder=2,
                    )
                )

    for segment in wall_layout.segments:
        start, end = wall_segment_line(segment)
        ax.plot(
            [start[0], end[0]],
            [start[1], end[1]],
            color="#111111",
            linewidth=3.0,
            solid_capstyle="butt",
            zorder=4,
        )

    for opening in opening_layout.openings:
        start, end = opening_line(opening)
        color = "#d4a017" if opening.kind == "gate" else "#7b2cbf"
        ax.plot(
            [start[0], end[0]],
            [start[1], end[1]],
            color=color,
            linewidth=4.5,
            solid_capstyle="round",
            zorder=5,
        )

    for connection in layout.selected_graph.connections:
        if connection.connection_type != ConnectionType.PASSAGE:
            continue
        path_cells = [cells_by_id[cell_id] for cell_id in connection.path_cell_ids]
        xs, ys = zip(*(_cell_centroid(cell) for cell in path_cells))
        ax.plot(
            xs,
            ys,
            color="#5a189a",
            linewidth=1.5,
            linestyle="--",
            alpha=0.8,
            zorder=3,
        )

    _setup_axes(ax, partition.world_width, partition.world_height, title)
    fig.tight_layout()
    _save_figure(fig, output_base)

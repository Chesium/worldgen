from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from random_gazebo_world.partition import Partition


def render_partition(
    partition: Partition,
    output_base: Path,
    title: str = "BSP Partition",
) -> None:
    output_base.parent.mkdir(parents=True, exist_ok=True)

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

    ax.set_xlim(0.0, partition.world_width)
    ax.set_ylim(0.0, partition.world_height)
    ax.set_aspect("equal")
    ax.set_title(title)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    fig.tight_layout()

    png_path = output_base.with_suffix(".png")
    svg_path = output_base.with_suffix(".svg")
    fig.savefig(png_path, dpi=150)
    fig.savefig(svg_path)
    plt.close(fig)

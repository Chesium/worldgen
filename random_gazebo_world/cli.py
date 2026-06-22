from __future__ import annotations

import argparse
import random
from pathlib import Path

from random_gazebo_world.adjacency import build_adjacency_graph
from random_gazebo_world.config import Config, load_config
from random_gazebo_world.partition import generate_partition
from random_gazebo_world.rng import create_seeded_rng
from random_gazebo_world.topology import select_rooms
from random_gazebo_world.visualize import (
    render_adjacency_graph,
    render_partition,
    render_selected_rooms,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="random_gazebo_world",
        description="Generate procedural Gazebo worlds from rectangular room layouts.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    generate = subparsers.add_parser("generate", help="Generate a world layout.")
    generate.add_argument(
        "--config",
        required=True,
        type=Path,
        help="Path to YAML config file.",
    )
    generate.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed override. Defaults to config random_seed.",
    )
    generate.add_argument(
        "--out",
        required=True,
        type=Path,
        help="Output directory for generated world artifacts.",
    )
    return parser


def generate_world(config: Config, out_dir: Path) -> random.Random:
    out_dir.mkdir(parents=True, exist_ok=True)
    debug_dir = out_dir / "debug"
    debug_dir.mkdir(parents=True, exist_ok=True)

    rng = create_seeded_rng(config.random_seed)
    partition = generate_partition(config, rng)
    adjacency = build_adjacency_graph(partition)
    room_selection = select_rooms(partition, config, rng)
    render_partition(partition, debug_dir / "01_partition")
    render_selected_rooms(partition, room_selection, debug_dir / "02_selected_rooms")
    render_adjacency_graph(partition, adjacency, debug_dir / "03_cell_adjacency_graph")
    return rng


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.command == "generate":
        config = load_config(args.config)
        if args.seed is not None:
            config = config.with_seed(args.seed)
        generate_world(config, args.out)
        return 0

    parser.error(f"Unknown command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())

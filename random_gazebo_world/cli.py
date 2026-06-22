from __future__ import annotations

import argparse
from pathlib import Path

from random_gazebo_world.config import load_config
from random_gazebo_world.pipeline import generate_valid_world, write_world_outputs


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


def generate_world(config, out_dir: Path) -> None:
    world = generate_valid_world(config)
    write_world_outputs(world, out_dir)


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

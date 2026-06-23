#!/usr/bin/env python3
"""Augment a generated ``world.sdf`` so it is simulatable by Gazebo + Nav2.

The world generator emits a static SDF with geometry and lights but no Gazebo
system plugins, and declares ``<physics type="ignored">``. Without the physics,
sensors, IMU, scene-broadcaster and user-commands systems the spawned TurtleBot3
has no dynamics and publishes no sensor data, so Nav2 cannot localise or move it.

This script injects the same system plugins used by Nav2's stock world
(``nav2_minimal_tb3_sim/worlds/tb3_sandbox.sdf.xacro``) and switches the physics
engine to ``ode``.

For legacy generator output it can also rewrite "solid" fill regions exported as
``<polyline>`` geometry. Gazebo cannot extrude the non-convex/keyhole-shaped
polygons -- it logs ``Unable to extrude mesh`` and drops them. Modern generator
output exports those solids as boxes or meshes directly, so this conversion path
is only a compatibility fallback.

It depends only on numpy (available in both the uv venv and a sourced ROS
environment); the orchestrator imports it inside the venv.

Usage:
    python augment_world.py INPUT.sdf [--output OUTPUT.sdf]
                            [--render-engine ogre2] [--no-scene-broadcaster]
                            [--solid-resolution 0.1] [--no-convert-polylines]
"""
from __future__ import annotations

import argparse
import json
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

# Visual material applied to the rasterized solid boxes (muted grey, matches the
# generator's wall material closely enough for debugging in RViz/the GUI).
_SOLID_MATERIAL: tuple[tuple[str, str], ...] = (
    ("ambient", "0.2 0.2 0.22 1"),
    ("diffuse", "0.45 0.45 0.5 1"),
    ("specular", "0.03 0.03 0.03 1"),
)

# (filename, plugin class name, list of (child_tag, child_text)) for each system.
_SYSTEM_PLUGINS: tuple[tuple[str, str, tuple[tuple[str, str], ...]], ...] = (
    ("gz-sim-physics-system", "gz::sim::systems::Physics", ()),
    ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands", ()),
    ("gz-sim-imu-system", "gz::sim::systems::Imu", ()),
)

_SCENE_BROADCASTER = (
    "gz-sim-scene-broadcaster-system",
    "gz::sim::systems::SceneBroadcaster",
    (),
)


# region agent log
def _debug_log(
    run_id: str,
    hypothesis_id: str,
    location: str,
    message: str,
    data: dict,
) -> None:
    payload = {
        "sessionId": "fa55d2",
        "runId": run_id,
        "hypothesisId": hypothesis_id,
        "location": location,
        "message": message,
        "data": data,
        "timestamp": int(time.time() * 1000),
    }
    try:
        with Path("/home/chesium/worldgen/.cursor/debug-fa55d2.log").open(
            "a", encoding="utf-8"
        ) as handle:
            handle.write(json.dumps(payload, sort_keys=True) + "\n")
    except OSError:
        pass
# endregion agent log


def _make_plugin(
    filename: str, name: str, children: tuple[tuple[str, str], ...]
) -> ET.Element:
    plugin = ET.Element("plugin", filename=filename, name=name)
    for tag, text in children:
        child = ET.SubElement(plugin, tag)
        child.text = text
    return plugin


def augment_world_sdf(
    input_path: Path,
    output_path: Path,
    *,
    render_engine: str = "ogre",
    include_scene_broadcaster: bool = True,
    max_step_size: float = 0.003,
    solid_resolution: float = 0.1,
    convert_legacy_polylines: bool = True,
) -> Path:
    tree = ET.parse(input_path)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"No <world> element found in {input_path}")

    # region agent log
    _debug_log(
        "post-render-fix",
        "N5",
        "demo/scripts/augment_world.py:augment_world_sdf",
        "augmenting world with render engine",
        {
            "input_path": str(input_path),
            "output_path": str(output_path),
            "render_engine": render_engine,
            "convert_legacy_polylines": convert_legacy_polylines,
        },
    )
    # endregion agent log

    if convert_legacy_polylines:
        _convert_solid_polylines(world, solid_resolution)

    plugins: list[ET.Element] = [
        _make_plugin(filename, name, children)
        for filename, name, children in _SYSTEM_PLUGINS
    ]
    # Sensors system needs an explicit render engine for the lidar/camera.
    plugins.append(
        _make_plugin(
            "gz-sim-sensors-system",
            "gz::sim::systems::Sensors",
            (("render_engine", render_engine),),
        )
    )
    if include_scene_broadcaster:
        plugins.append(_make_plugin(*_SCENE_BROADCASTER))

    # Skip plugins that are already present (idempotent re-runs).
    existing = {
        plugin.get("filename")
        for plugin in world.findall("plugin")
        if plugin.get("filename")
    }
    to_insert = [p for p in plugins if p.get("filename") not in existing]
    for offset, plugin in enumerate(to_insert):
        world.insert(offset, plugin)

    _fix_physics(world, max_step_size)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return output_path


def _convert_solid_polylines(world: ET.Element, resolution: float) -> int:
    """Replace every ``<polyline>`` solid with rasterized ``<box>`` primitives.

    Returns the number of box pairs (collision+visual) emitted. Idempotent: if the
    polylines were already converted on a previous run there is nothing to do.
    """
    link = world.find("./model[@name='walls']/link")
    if link is None:
        return 0

    polygons: list[np.ndarray] = []
    heights: list[float] = []
    to_remove: list[ET.Element] = []
    for element in list(link):
        if element.tag not in ("collision", "visual"):
            continue
        poly = element.find("./geometry/polyline")
        if poly is None:
            continue
        to_remove.append(element)
        # Only build geometry from the collision copies to avoid duplicates;
        # visual polylines describe the same polygon.
        if element.tag != "collision":
            continue
        pts = [
            tuple(float(v) for v in (p.text or "").split())
            for p in poly.findall("point")
        ]
        pts = [p for p in pts if len(p) == 2]
        if len(pts) < 3:
            continue
        height_el = poly.find("height")
        height = float(height_el.text) if height_el is not None and height_el.text else 1.0
        polygons.append(np.asarray(pts, dtype=float))
        heights.append(height)

    if not to_remove:
        return 0
    for element in to_remove:
        link.remove(element)
    if not polygons:
        return 0

    height = max(heights)
    boxes = _rasterize_polygons_to_boxes(polygons, resolution)
    for index, (cx, cy, sx, sy) in enumerate(boxes):
        _append_solid_box(link, index, cx, cy, sx, sy, height)
    return len(boxes)


def _rasterize_polygons_to_boxes(
    polygons: list[np.ndarray], resolution: float
) -> list[tuple[float, float, float, float]]:
    """Rasterize polygons onto a shared grid, then greedily cover the filled
    cells with maximal axis-aligned rectangles. Returns (cx, cy, sx, sy) boxes.
    """
    all_pts = np.concatenate(polygons, axis=0)
    xmin, ymin = all_pts.min(axis=0)
    xmax, ymax = all_pts.max(axis=0)
    nx = max(1, int(np.ceil((xmax - xmin) / resolution)))
    ny = max(1, int(np.ceil((ymax - ymin) / resolution)))
    cx = xmin + (np.arange(nx) + 0.5) * resolution
    cy = ymin + (np.arange(ny) + 0.5) * resolution
    grid_x, grid_y = np.meshgrid(cx, cy)  # shape (ny, nx)

    mask = np.zeros((ny, nx), dtype=bool)
    for polygon in polygons:
        mask |= _points_in_polygon(grid_x, grid_y, polygon)
    if not mask.any():
        return []

    boxes: list[tuple[float, float, float, float]] = []
    for r0, c0, r1, c1 in _greedy_rectangles(mask):
        width = (c1 - c0 + 1) * resolution
        height = (r1 - r0 + 1) * resolution
        center_x = xmin + (c0 + (c1 - c0 + 1) / 2.0) * resolution
        center_y = ymin + (r0 + (r1 - r0 + 1) / 2.0) * resolution
        boxes.append((center_x, center_y, width, height))
    return boxes


def _points_in_polygon(
    px: np.ndarray, py: np.ndarray, polygon: np.ndarray
) -> np.ndarray:
    """Vectorized even-odd ray casting for an arbitrary simple polygon."""
    inside = np.zeros(px.shape, dtype=bool)
    x1 = polygon[:, 0]
    y1 = polygon[:, 1]
    x2 = np.roll(x1, -1)
    y2 = np.roll(y1, -1)
    for ax, ay, bx, by in zip(x1, y1, x2, y2):
        if ay == by:
            continue
        crosses = (ay > py) != (by > py)
        x_cross = ax + (py - ay) * (bx - ax) / (by - ay)
        inside ^= crosses & (px < x_cross)
    return inside


def _greedy_rectangles(mask: np.ndarray) -> list[tuple[int, int, int, int]]:
    """Greedily cover True cells with maximal rectangles (top-left first)."""
    remaining = mask.copy()
    height, width = remaining.shape
    rects: list[tuple[int, int, int, int]] = []
    flat = remaining.reshape(-1)
    while True:
        idx = int(np.argmax(flat))
        if not flat[idx]:
            break
        r0, c0 = divmod(idx, width)
        c1 = c0
        while c1 + 1 < width and remaining[r0, c1 + 1]:
            c1 += 1
        r1 = r0
        while r1 + 1 < height and remaining[r1 + 1, c0:c1 + 1].all():
            r1 += 1
        remaining[r0:r1 + 1, c0:c1 + 1] = False
        rects.append((r0, c0, r1, c1))
    return rects


def _append_solid_box(
    link: ET.Element,
    index: int,
    center_x: float,
    center_y: float,
    size_x: float,
    size_y: float,
    height: float,
) -> None:
    pose_text = f"{center_x:.6f} {center_y:.6f} {height / 2.0:.6f} 0 0 0"
    size_text = f"{size_x:.6f} {size_y:.6f} {height:.6f}"
    for kind in ("collision", "visual"):
        element = ET.SubElement(link, kind, name=f"solid_box_{index}_{kind}")
        pose = ET.SubElement(element, "pose")
        pose.text = pose_text
        geometry = ET.SubElement(element, "geometry")
        box = ET.SubElement(geometry, "box")
        size = ET.SubElement(box, "size")
        size.text = size_text
        if kind == "visual":
            material = ET.SubElement(element, "material")
            for tag, text in _SOLID_MATERIAL:
                child = ET.SubElement(material, tag)
                child.text = text


def _fix_physics(world: ET.Element, max_step_size: float) -> None:
    physics = world.find("physics")
    if physics is None:
        physics = ET.SubElement(world, "physics", name="default_physics")
    physics.set("type", "ode")
    step = physics.find("max_step_size")
    if step is None:
        step = ET.SubElement(physics, "max_step_size")
    step.text = str(max_step_size)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="Path to generated world.sdf")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output path (default: world_nav.sdf next to the input).",
    )
    parser.add_argument("--render-engine", default="ogre")
    parser.add_argument(
        "--no-scene-broadcaster",
        action="store_true",
        help="Omit the scene broadcaster system (slightly lighter, no GUI/RViz scene).",
    )
    parser.add_argument(
        "--solid-resolution",
        type=float,
        default=0.1,
        help="Cell size (m) used to rasterize polyline solids into collision/visual "
             "boxes. Smaller is more faithful but emits more boxes.",
    )
    parser.add_argument(
        "--no-convert-polylines",
        action="store_true",
        help="Skip legacy solid polyline conversion and only inject sim systems.",
    )
    args = parser.parse_args(argv)

    output = args.output or args.input.with_name("world_nav.sdf")
    result = augment_world_sdf(
        args.input,
        output,
        render_engine=args.render_engine,
        include_scene_broadcaster=not args.no_scene_broadcaster,
        solid_resolution=args.solid_resolution,
        convert_legacy_polylines=not args.no_convert_polylines,
    )
    print(f"Wrote augmented world: {result}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

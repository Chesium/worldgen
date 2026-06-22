from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from xml.dom import minidom

from random_gazebo_world.config import Config
from random_gazebo_world.geometry import Rect
from random_gazebo_world.walls import WallLayout, WallSegment


class SdfExportError(RuntimeError):
    """Raised when SDF export or validation fails."""


@dataclass(frozen=True)
class WallBox:
    name: str
    center_x: float
    center_y: float
    center_z: float
    size_x: float
    size_y: float
    size_z: float


def export_world_sdf(
    wall_layout: WallLayout,
    config: Config,
    output_path: Path,
) -> Path:
    boxes = _all_boxes(wall_layout, config)
    tree = _build_sdf_tree(boxes)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    _write_pretty_xml(tree, output_path)
    validate_world_sdf(output_path, wall_layout, config)
    return output_path


def _solid_rects(wall_layout: WallLayout) -> tuple[Rect, ...]:
    rects: list[Rect] = []
    if wall_layout.passage_geometry is not None:
        rects.extend(wall_layout.passage_geometry.solids)
    rects.extend(wall_layout.unused_solids)
    return tuple(rects)


def _all_boxes(wall_layout: WallLayout, config: Config) -> list[WallBox]:
    boxes = [
        wall_segment_to_box(segment, config.wall_height, config.wall_thickness, index)
        for index, segment in enumerate(wall_layout.segments)
    ]
    offset = len(boxes)
    boxes.extend(
        rect_to_box(rect, config.wall_height, offset + index)
        for index, rect in enumerate(_solid_rects(wall_layout))
    )
    return boxes


def rect_to_box(rect: Rect, wall_height: float, index: int) -> WallBox:
    center_x, center_y = rect.center
    return WallBox(
        name=f"wall_{index}",
        center_x=center_x,
        center_y=center_y,
        center_z=wall_height / 2.0,
        size_x=rect.width,
        size_y=rect.height,
        size_z=wall_height,
    )


def wall_segment_to_box(
    segment: WallSegment,
    wall_height: float,
    wall_thickness: float,
    index: int,
) -> WallBox:
    center_z = wall_height / 2.0
    length = segment.length

    if segment.orientation == "vertical":
        return WallBox(
            name=f"wall_{index}",
            center_x=segment.fixed_coord,
            center_y=(segment.span_start + segment.span_end) / 2.0,
            center_z=center_z,
            size_x=wall_thickness,
            size_y=length,
            size_z=wall_height,
        )

    return WallBox(
        name=f"wall_{index}",
        center_x=(segment.span_start + segment.span_end) / 2.0,
        center_y=segment.fixed_coord,
        center_z=center_z,
        size_x=length,
        size_y=wall_thickness,
        size_z=wall_height,
    )


def validate_world_sdf(
    sdf_path: Path,
    wall_layout: WallLayout,
    config: Config,
) -> None:
    if not sdf_path.is_file():
        raise SdfExportError(f"SDF file not found: {sdf_path}")

    tree = ET.parse(sdf_path)
    root = tree.getroot()
    if root.tag != "sdf":
        raise SdfExportError("Root element must be <sdf>")

    world = root.find("world")
    if world is None:
        raise SdfExportError("SDF must contain a <world> element")

    model = world.find("model")
    if model is None:
        raise SdfExportError("SDF world must contain a <model> element")
    if model.get("static") != "true":
        raise SdfExportError("Wall model must be static")

    link = model.find("link")
    if link is None:
        raise SdfExportError("Wall model must contain a <link> element")

    collisions = link.findall("collision")
    visuals = link.findall("visual")
    expected_boxes = _all_boxes(wall_layout, config)
    expected = len(expected_boxes)
    if len(collisions) != expected or len(visuals) != expected:
        raise SdfExportError(
            f"Expected {expected} wall collision/visual pairs, got "
            f"{len(collisions)}/{len(visuals)}"
        )

    for index, expected_box in enumerate(expected_boxes):
        collision = collisions[index]
        pose = collision.find("pose")
        size = collision.find("./geometry/box/size")
        if pose is None or size is None:
            raise SdfExportError(f"Wall {index} collision missing pose or box size")

        actual_pose = _parse_pose(pose.text or "")
        actual_size = _parse_size(size.text or "")
        expected_pose = (
            expected_box.center_x,
            expected_box.center_y,
            expected_box.center_z,
        )
        expected_size = (
            expected_box.size_x,
            expected_box.size_y,
            expected_box.size_z,
        )
        if not _approx_tuple(actual_pose, expected_pose):
            raise SdfExportError(f"Wall {index} pose mismatch")
        if not _approx_tuple(actual_size, expected_size):
            raise SdfExportError(f"Wall {index} size mismatch")


def _build_sdf_tree(boxes: list[WallBox]) -> ET.ElementTree:
    sdf = ET.Element("sdf", version="1.8")
    world = ET.SubElement(sdf, "world", name="generated_world")
    model = ET.SubElement(world, "model", name="walls")
    model.set("static", "true")
    link = ET.SubElement(model, "link", name="walls_link")

    for box in boxes:
        _append_box(link, f"{box.name}_collision", box, kind="collision")
        _append_box(link, f"{box.name}_visual", box, kind="visual")

    return ET.ElementTree(sdf)


def _append_box(
    link: ET.Element,
    name: str,
    box: WallBox,
    *,
    kind: str,
) -> None:
    element = ET.SubElement(link, kind, name=name)
    pose = ET.SubElement(element, "pose")
    pose.text = _format_pose(box.center_x, box.center_y, box.center_z)

    geometry = ET.SubElement(element, "geometry")
    box_geometry = ET.SubElement(geometry, "box")
    size = ET.SubElement(box_geometry, "size")
    size.text = _format_size(box.size_x, box.size_y, box.size_z)

    if kind == "visual":
        material = ET.SubElement(element, "material")
        ambient = ET.SubElement(material, "ambient")
        ambient.text = "0.6 0.6 0.6 1"
        diffuse = ET.SubElement(material, "diffuse")
        diffuse.text = "0.6 0.6 0.6 1"


def _write_pretty_xml(tree: ET.ElementTree, output_path: Path) -> None:
    xml_bytes = ET.tostring(tree.getroot(), encoding="utf-8")
    pretty = minidom.parseString(xml_bytes).toprettyxml(indent="  ")
    output_path.write_text(pretty, encoding="utf-8")


def _format_pose(x: float, y: float, z: float) -> str:
    return f"{x:.6f} {y:.6f} {z:.6f} 0 0 0"


def _format_size(x: float, y: float, z: float) -> str:
    return f"{x:.6f} {y:.6f} {z:.6f}"


def _parse_pose(text: str) -> tuple[float, float, float]:
    parts = text.split()
    if len(parts) < 3:
        raise SdfExportError(f"Invalid pose: {text!r}")
    return float(parts[0]), float(parts[1]), float(parts[2])


def _parse_size(text: str) -> tuple[float, float, float]:
    parts = text.split()
    if len(parts) != 3:
        raise SdfExportError(f"Invalid size: {text!r}")
    return float(parts[0]), float(parts[1]), float(parts[2])


def _approx_tuple(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
    eps: float = 1e-6,
) -> bool:
    return all(abs(a - b) <= eps for a, b in zip(left, right, strict=True))

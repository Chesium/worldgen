from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from xml.dom import minidom

from shapely.geometry import Polygon

from random_gazebo_world.config import Config
from random_gazebo_world.walls import WallLayout, WallSegment


class SdfExportError(RuntimeError):
    """Raised when SDF export or validation fails."""


SCENE_AMBIENT = "0.25 0.25 0.25 1"
SCENE_BACKGROUND = "0.550000012 0.600000024 0.649999976 1"
GRAVITY = "0 0 -9.8000000000000007"
MAGNETIC_FIELD = "5.5644999999999998e-06 2.2875799999999999e-05 -4.2388400000000002e-05"

VISUAL_MATERIAL = {
    "lighting": "true",
    "ambient": "0.219999999 0.25 0.270000011 1",
    "diffuse": "0.8 0.8 0.8 1",
    "specular": "0.0299999993 0.0299999993 0.0299999993 1",
    "shininess": "8",
    "emissive": "0 0 0 1",
    "double_sided": "true",
    "metalness": "0.0",
    "roughness": "0.85",
}

SUN_LIGHT = {
    "name": "sun",
    "pose": "0 0 10 0 0 0",
    "cast_shadows": "true",
    "intensity": "1",
    "direction": "-0.1 0.14 -0.81999999999999995",
    "diffuse": "1 0.959999979 0.860000014 1",
    "specular": "0.100000001 0.100000001 0.100000001 1",
    "range": "1000",
    "linear": "0.0099999997764825821",
    "constant": "0.89999997615814209",
    "quadratic": "0.0010000000474974513",
}

FILL_LIGHT = {
    "name": "fill_light",
    "pose": "0 0 0 0 0 0",
    "cast_shadows": "false",
    "intensity": "1",
    "direction": "0.45000000000000001 -0.65000000000000002 -0.59999999999999998",
    "diffuse": "0.25 0.319999993 0.400000006 1",
    "specular": "0 0 0 1",
    "range": "10",
    "linear": "1",
    "constant": "1",
    "quadratic": "0",
}


@dataclass(frozen=True)
class WallBox:
    name: str
    center_x: float
    center_y: float
    center_z: float
    size_x: float
    size_y: float
    size_z: float
    yaw: float = 0.0


@dataclass(frozen=True)
class SolidPolyline:
    name: str
    points: tuple[tuple[float, float], ...]
    height: float


def export_world_sdf(
    wall_layout: WallLayout,
    config: Config,
    output_path: Path,
) -> Path:
    boxes = _wall_boxes(wall_layout, config)
    polylines = _solid_polylines(wall_layout, config)
    tree = _build_sdf_tree(boxes, polylines, config)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    _write_pretty_xml(tree, output_path)
    validate_world_sdf(output_path, wall_layout, config)
    return output_path


def ground_box(config: Config) -> WallBox:
    thickness = config.ground_thickness
    return WallBox(
        name="ground",
        center_x=config.world_width / 2.0,
        center_y=config.world_height / 2.0,
        center_z=-thickness / 2.0,
        size_x=config.world_width,
        size_y=config.world_height,
        size_z=thickness,
    )


def _solid_polygons(wall_layout: WallLayout) -> tuple[Polygon, ...]:
    polygons: list[Polygon] = []
    if wall_layout.passage_geometry is not None:
        polygons.extend(wall_layout.passage_geometry.solids)
    polygons.extend(wall_layout.unused_solids)
    return tuple(polygons)


def _wall_boxes(wall_layout: WallLayout, config: Config) -> list[WallBox]:
    return [
        wall_segment_to_box(segment, config.wall_height, config.wall_thickness, index)
        for index, segment in enumerate(wall_layout.segments)
    ]


def _solid_polylines(wall_layout: WallLayout, config: Config) -> list[SolidPolyline]:
    polylines: list[SolidPolyline] = []
    for index, polygon in enumerate(_solid_polygons(wall_layout)):
        points = _polygon_points(polygon)
        if points is None:
            continue
        polylines.append(
            SolidPolyline(name=f"solid_{index}", points=points, height=config.wall_height)
        )
    return polylines


def _polygon_points(polygon: Polygon) -> tuple[tuple[float, float], ...] | None:
    if polygon.is_empty:
        return None
    coords = list(polygon.exterior.coords)
    if len(coords) < 4:
        return None
    return tuple((float(x), float(y)) for x, y in coords[:-1])


def wall_segment_to_box(
    segment: WallSegment,
    wall_height: float,
    wall_thickness: float,
    index: int,
) -> WallBox:
    center_z = wall_height / 2.0
    length = segment.length
    orientation = segment.orientation

    if orientation == "vertical":
        return WallBox(
            name=f"wall_{index}",
            center_x=segment.fixed_coord,
            center_y=(segment.span_start + segment.span_end) / 2.0,
            center_z=center_z,
            size_x=wall_thickness,
            size_y=length,
            size_z=wall_height,
        )

    if orientation == "horizontal":
        return WallBox(
            name=f"wall_{index}",
            center_x=(segment.span_start + segment.span_end) / 2.0,
            center_y=segment.fixed_coord,
            center_z=center_z,
            size_x=length,
            size_y=wall_thickness,
            size_z=wall_height,
        )

    center_x, center_y = segment.midpoint
    return WallBox(
        name=f"wall_{index}",
        center_x=center_x,
        center_y=center_y,
        center_z=center_z,
        size_x=length,
        size_y=wall_thickness,
        size_z=wall_height,
        yaw=segment.yaw,
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

    models = world.findall("model")
    model = next((item for item in models if item.get("name") == "walls"), None)
    if model is None:
        raise SdfExportError("SDF world must contain a walls model")
    if model.get("static") != "true":
        raise SdfExportError("Wall model must be static")

    link = model.find("link")
    if link is None:
        raise SdfExportError("Wall model must contain a <link> element")

    expected_boxes = _wall_boxes(wall_layout, config)
    expected_polylines = _solid_polylines(wall_layout, config)

    box_collisions = [
        item for item in link.findall("collision")
        if item.find("./geometry/box") is not None
    ]
    box_visuals = [
        item for item in link.findall("visual")
        if item.find("./geometry/box") is not None
    ]
    polyline_collisions = [
        item for item in link.findall("collision")
        if item.find("./geometry/polyline") is not None
    ]
    polyline_visuals = [
        item for item in link.findall("visual")
        if item.find("./geometry/polyline") is not None
    ]

    if len(box_collisions) != len(expected_boxes) or len(box_visuals) != len(
        expected_boxes
    ):
        raise SdfExportError(
            f"Expected {len(expected_boxes)} wall box collision/visual pairs, got "
            f"{len(box_collisions)}/{len(box_visuals)}"
        )
    if len(polyline_collisions) != len(expected_polylines) or len(
        polyline_visuals
    ) != len(expected_polylines):
        raise SdfExportError(
            f"Expected {len(expected_polylines)} solid polyline collision/visual pairs, "
            f"got {len(polyline_collisions)}/{len(polyline_visuals)}"
        )

    for index, expected_box in enumerate(expected_boxes):
        collision = box_collisions[index]
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

    _validate_ground_model(world, config)


def _validate_ground_model(world: ET.Element, config: Config) -> None:
    models = world.findall("model")
    ground_model = next(
        (model for model in models if model.get("name") == "ground"), None
    )
    if ground_model is None:
        raise SdfExportError("SDF world must contain a ground model")
    if ground_model.get("static") != "true":
        raise SdfExportError("Ground model must be static")

    link = ground_model.find("link")
    if link is None:
        raise SdfExportError("Ground model must contain a link")

    expected = ground_box(config)
    collision = link.find("collision")
    visual = link.find("visual")
    if collision is None or visual is None:
        raise SdfExportError("Ground model must contain collision and visual")

    collision_pose = collision.find("pose")
    collision_size = collision.find("./geometry/box/size")
    if collision_pose is None or collision_size is None:
        raise SdfExportError("Ground collision missing pose or box size")

    actual_pose = _parse_pose(collision_pose.text or "")
    actual_size = _parse_size(collision_size.text or "")
    expected_pose = (expected.center_x, expected.center_y, expected.center_z)
    expected_size = (expected.size_x, expected.size_y, expected.size_z)
    if not _approx_tuple(actual_pose, expected_pose):
        raise SdfExportError("Ground pose mismatch")
    if not _approx_tuple(actual_size, expected_size):
        raise SdfExportError("Ground size mismatch")


def _build_sdf_tree(
    boxes: list[WallBox],
    polylines: list[SolidPolyline],
    config: Config,
) -> ET.ElementTree:
    sdf = ET.Element("sdf", version="1.10")
    world = ET.SubElement(sdf, "world", name="generated_world")
    _append_world_environment(world)
    _append_ground_model(world, config)
    model = ET.SubElement(world, "model", name="walls")
    model.set("static", "true")
    link = ET.SubElement(model, "link", name="walls_link")

    for box in boxes:
        _append_box(link, f"{box.name}_collision", box, kind="collision")
        _append_box(link, f"{box.name}_visual", box, kind="visual")

    for polyline in polylines:
        _append_polyline(link, f"{polyline.name}_collision", polyline, kind="collision")
        _append_polyline(link, f"{polyline.name}_visual", polyline, kind="visual")

    _append_directional_light(world, SUN_LIGHT)
    _append_directional_light(world, FILL_LIGHT)
    return ET.ElementTree(sdf)


def _append_polyline(
    link: ET.Element,
    name: str,
    polyline: SolidPolyline,
    *,
    kind: str,
) -> None:
    element = ET.SubElement(link, kind, name=name)
    pose = ET.SubElement(element, "pose")
    pose.text = "0 0 0 0 0 0"

    geometry = ET.SubElement(element, "geometry")
    poly = ET.SubElement(geometry, "polyline")
    for x, y in polyline.points:
        point = ET.SubElement(poly, "point")
        point.text = f"{x:.6f} {y:.6f}"
    height = ET.SubElement(poly, "height")
    height.text = f"{polyline.height:.6f}"

    if kind == "visual":
        _append_visual_material(element)


def _append_ground_model(world: ET.Element, config: Config) -> None:
    box = ground_box(config)
    model = ET.SubElement(world, "model", name="ground")
    model.set("static", "true")
    link = ET.SubElement(model, "link", name="ground_link")
    _append_box(link, "ground_collision", box, kind="collision")
    _append_box(link, "ground_visual", box, kind="visual")


def _append_world_environment(world: ET.Element) -> None:
    physics = ET.SubElement(world, "physics", name="default_physics", type="ignored")
    max_step_size = ET.SubElement(physics, "max_step_size")
    max_step_size.text = "0.001"
    real_time_factor = ET.SubElement(physics, "real_time_factor")
    real_time_factor.text = "1"
    real_time_update_rate = ET.SubElement(physics, "real_time_update_rate")
    real_time_update_rate.text = "1000"

    scene = ET.SubElement(world, "scene")
    ambient = ET.SubElement(scene, "ambient")
    ambient.text = SCENE_AMBIENT
    background = ET.SubElement(scene, "background")
    background.text = SCENE_BACKGROUND
    shadows = ET.SubElement(scene, "shadows")
    shadows.text = "true"

    gravity = ET.SubElement(world, "gravity")
    gravity.text = GRAVITY
    magnetic_field = ET.SubElement(world, "magnetic_field")
    magnetic_field.text = MAGNETIC_FIELD
    ET.SubElement(world, "atmosphere", type="adiabatic")


def _append_directional_light(world: ET.Element, settings: dict[str, str]) -> None:
    light = ET.SubElement(world, "light", name=settings["name"], type="directional")
    pose = ET.SubElement(light, "pose")
    pose.text = settings["pose"]
    cast_shadows = ET.SubElement(light, "cast_shadows")
    cast_shadows.text = settings["cast_shadows"]
    intensity = ET.SubElement(light, "intensity")
    intensity.text = settings["intensity"]
    direction = ET.SubElement(light, "direction")
    direction.text = settings["direction"]
    diffuse = ET.SubElement(light, "diffuse")
    diffuse.text = settings["diffuse"]
    specular = ET.SubElement(light, "specular")
    specular.text = settings["specular"]

    attenuation = ET.SubElement(light, "attenuation")
    light_range = ET.SubElement(attenuation, "range")
    light_range.text = settings["range"]
    linear = ET.SubElement(attenuation, "linear")
    linear.text = settings["linear"]
    constant = ET.SubElement(attenuation, "constant")
    constant.text = settings["constant"]
    quadratic = ET.SubElement(attenuation, "quadratic")
    quadratic.text = settings["quadratic"]

    spot = ET.SubElement(light, "spot")
    inner_angle = ET.SubElement(spot, "inner_angle")
    inner_angle.text = "0"
    outer_angle = ET.SubElement(spot, "outer_angle")
    outer_angle.text = "0"
    falloff = ET.SubElement(spot, "falloff")
    falloff.text = "0"


def _append_visual_material(visual: ET.Element) -> None:
    material = ET.SubElement(visual, "material")
    lighting = ET.SubElement(material, "lighting")
    lighting.text = VISUAL_MATERIAL["lighting"]
    ambient = ET.SubElement(material, "ambient")
    ambient.text = VISUAL_MATERIAL["ambient"]
    diffuse = ET.SubElement(material, "diffuse")
    diffuse.text = VISUAL_MATERIAL["diffuse"]
    specular = ET.SubElement(material, "specular")
    specular.text = VISUAL_MATERIAL["specular"]
    shininess = ET.SubElement(material, "shininess")
    shininess.text = VISUAL_MATERIAL["shininess"]
    emissive = ET.SubElement(material, "emissive")
    emissive.text = VISUAL_MATERIAL["emissive"]
    double_sided = ET.SubElement(material, "double_sided")
    double_sided.text = VISUAL_MATERIAL["double_sided"]
    pbr = ET.SubElement(material, "pbr")
    metal = ET.SubElement(pbr, "metal")
    metalness = ET.SubElement(metal, "metalness")
    metalness.text = VISUAL_MATERIAL["metalness"]
    roughness = ET.SubElement(metal, "roughness")
    roughness.text = VISUAL_MATERIAL["roughness"]


def _append_box(
    link: ET.Element,
    name: str,
    box: WallBox,
    *,
    kind: str,
) -> None:
    element = ET.SubElement(link, kind, name=name)
    pose = ET.SubElement(element, "pose")
    pose.text = _format_pose(box.center_x, box.center_y, box.center_z, box.yaw)

    geometry = ET.SubElement(element, "geometry")
    box_geometry = ET.SubElement(geometry, "box")
    size = ET.SubElement(box_geometry, "size")
    size.text = _format_size(box.size_x, box.size_y, box.size_z)

    if kind == "visual":
        _append_visual_material(element)


def _write_pretty_xml(tree: ET.ElementTree, output_path: Path) -> None:
    xml_bytes = ET.tostring(tree.getroot(), encoding="utf-8")
    pretty = minidom.parseString(xml_bytes).toprettyxml(indent="  ")
    output_path.write_text(pretty, encoding="utf-8")


def _format_pose(x: float, y: float, z: float, yaw: float = 0.0) -> str:
    return f"{x:.6f} {y:.6f} {z:.6f} 0 0 {yaw:.6f}"


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

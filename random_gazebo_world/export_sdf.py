from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Literal
from xml.dom import minidom

from shapely.geometry import Polygon
from shapely.ops import triangulate

from random_gazebo_world.config import Config
from random_gazebo_world.geometry import EPS, Rect
from random_gazebo_world.solid_geometry import (
    SolidShape,
    collect_tagged_solids,
    decompose_orthogonal_polygon,
    rect_from_polygon_bounds,
)
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


@dataclass(frozen=True)
class SolidMesh:
    name: str
    uri: str


@dataclass(frozen=True)
class SolidGeometryPlan:
    boxes: tuple[WallBox, ...]
    polylines: tuple[SolidPolyline, ...]
    meshes: tuple[SolidMesh, ...]


SolidExportMode = Literal["polyline", "hybrid"]


def export_world_sdf(
    wall_layout: WallLayout,
    config: Config,
    output_path: Path,
    *,
    solid_export_mode: SolidExportMode = "hybrid",
) -> Path:
    boxes = _wall_boxes(wall_layout, config)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    solid_plan = _plan_solid_geometry(
        wall_layout,
        config,
        output_path=output_path,
        mode=solid_export_mode,
    )
    tree = _build_sdf_tree(boxes, solid_plan, config)
    _write_pretty_xml(tree, output_path)
    validate_world_sdf(
        output_path,
        wall_layout,
        config,
        solid_export_mode=solid_export_mode,
    )
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


def _plan_solid_geometry(
    wall_layout: WallLayout,
    config: Config,
    *,
    output_path: Path,
    mode: SolidExportMode,
) -> SolidGeometryPlan:
    if mode == "polyline":
        return SolidGeometryPlan(
            boxes=(),
            polylines=tuple(_solid_polylines(wall_layout, config)),
            meshes=(),
        )
    if mode != "hybrid":
        raise SdfExportError(f"Unsupported solid export mode: {mode!r}")

    boxes: list[WallBox] = []
    meshes: list[SolidMesh] = []
    mesh_dir = output_path.parent / "meshes"
    tagged_solids = collect_tagged_solids(wall_layout)
    for solid_index, solid in enumerate(tagged_solids):
        if solid.shape is SolidShape.AXIS_ALIGNED_RECT:
            rects = (rect_from_polygon_bounds(solid.polygon),)
        elif solid.shape is SolidShape.ORTHOGONAL:
            rects = decompose_orthogonal_polygon(solid.polygon)
        else:
            rects = ()

        if rects:
            for rect_index, rect in enumerate(rects):
                boxes.append(
                    _solid_rect_to_box(
                        rect,
                        name=f"solid_{solid_index}_rect_{rect_index}",
                        height=config.wall_height,
                    )
                )
            continue

        mesh_path = mesh_dir / f"solid_{solid_index}.obj"
        _write_solid_mesh(solid.polygon, mesh_path, height=config.wall_height)
        meshes.append(SolidMesh(name=f"solid_{solid_index}", uri=_mesh_uri(mesh_path)))

    return SolidGeometryPlan(boxes=tuple(boxes), polylines=(), meshes=tuple(meshes))


def _solid_rect_to_box(rect: Rect, *, name: str, height: float) -> WallBox:
    center_x, center_y = rect.center
    return WallBox(
        name=name,
        center_x=center_x,
        center_y=center_y,
        center_z=height / 2.0,
        size_x=rect.width,
        size_y=rect.height,
        size_z=height,
    )


def _polygon_points(polygon: Polygon) -> tuple[tuple[float, float], ...] | None:
    if polygon.is_empty:
        return None
    coords = list(polygon.exterior.coords)
    if len(coords) < 4:
        return None
    return tuple((float(x), float(y)) for x, y in coords[:-1])


def _write_solid_mesh(polygon: Polygon, output_path: Path, *, height: float) -> None:
    triangles = [
        triangle
        for triangle in triangulate(polygon)
        if triangle.area > EPS and polygon.covers(triangle)
    ]
    triangle_area = sum(triangle.area for triangle in triangles)
    if abs(triangle_area - polygon.area) > 1e-6:
        raise SdfExportError(
            f"Could not triangulate solid polygon for mesh export: "
            f"area {triangle_area:.6f} != {polygon.area:.6f}"
        )

    vertices: list[tuple[float, float, float]] = []
    normals: list[tuple[float, float, float]] = []
    faces: list[tuple[int, ...]] = []

    def append_face(points: tuple[tuple[float, float, float], ...]) -> None:
        normal = _face_normal(points)
        indices: list[int] = []
        for x, y, z in points:
            vertices.append((round(float(x), 9), round(float(y), 9), round(float(z), 9)))
            normals.append(normal)
            indices.append(len(vertices))
        faces.append(tuple(indices))

    for triangle in triangles:
        coords = list(triangle.exterior.coords)[:3]
        append_face(tuple((x, y, height) for x, y in coords))
        append_face(tuple((x, y, 0.0) for x, y in reversed(coords)))

    for ring in [polygon.exterior, *polygon.interiors]:
        coords = list(ring.coords)
        for (ax, ay), (bx, by) in zip(coords, coords[1:], strict=False):
            append_face(
                (
                    (ax, ay, 0.0),
                    (bx, by, 0.0),
                    (bx, by, height),
                    (ax, ay, height),
                )
            )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    lines = ["# Generated by random_gazebo_world\n"]
    for x, y, z in vertices:
        lines.append(f"v {x:.9f} {y:.9f} {z:.9f}\n")
    for nx, ny, nz in normals:
        lines.append(f"vn {nx:.9f} {ny:.9f} {nz:.9f}\n")
    for face in faces:
        lines.append("f " + " ".join(f"{index}//{index}" for index in face) + "\n")
    output_path.write_text("".join(lines), encoding="utf-8")


def _face_normal(
    points: tuple[tuple[float, float, float], ...],
) -> tuple[float, float, float]:
    if len(points) < 3:
        return (0.0, 0.0, 1.0)
    ax, ay, az = points[0]
    bx, by, bz = points[1]
    cx, cy, cz = points[2]
    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    length = (nx * nx + ny * ny + nz * nz) ** 0.5
    if length <= EPS:
        return (0.0, 0.0, 1.0)
    return (nx / length, ny / length, nz / length)


def _mesh_uri(path: Path) -> str:
    return f"file://{path.resolve().as_posix()}"


def _model_is_static(model: ET.Element) -> bool:
    return model.get("static") == "true" or model.findtext("static") == "true"


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
    *,
    solid_export_mode: SolidExportMode = "hybrid",
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
    if not _model_is_static(model):
        raise SdfExportError("Wall model must be static")

    link = model.find("link")
    if link is None:
        raise SdfExportError("Wall model must contain a <link> element")

    expected_boxes = _wall_boxes(wall_layout, config)

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
    mesh_collisions = [
        item for item in link.findall("collision")
        if item.find("./geometry/mesh") is not None
    ]
    mesh_visuals = [
        item for item in link.findall("visual")
        if item.find("./geometry/mesh") is not None
    ]

    wall_box_collisions = [
        item for item in box_collisions if (item.get("name") or "").startswith("wall_")
    ]
    wall_box_visuals = [
        item for item in box_visuals if (item.get("name") or "").startswith("wall_")
    ]

    if len(wall_box_collisions) != len(expected_boxes) or len(wall_box_visuals) != len(
        expected_boxes
    ):
        raise SdfExportError(
            f"Expected {len(expected_boxes)} wall box collision/visual pairs, got "
            f"{len(wall_box_collisions)}/{len(wall_box_visuals)}"
        )
    _validate_solid_geometry_counts(
        sdf_path,
        wall_layout,
        config,
        solid_export_mode,
        box_collisions,
        box_visuals,
        polyline_collisions,
        polyline_visuals,
        mesh_collisions,
        mesh_visuals,
    )

    for index, expected_box in enumerate(expected_boxes):
        collision = wall_box_collisions[index]
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


def _validate_solid_geometry_counts(
    sdf_path: Path,
    wall_layout: WallLayout,
    config: Config,
    solid_export_mode: SolidExportMode,
    box_collisions: list[ET.Element],
    box_visuals: list[ET.Element],
    polyline_collisions: list[ET.Element],
    polyline_visuals: list[ET.Element],
    mesh_collisions: list[ET.Element],
    mesh_visuals: list[ET.Element],
) -> None:
    if solid_export_mode == "polyline":
        expected_polylines = _solid_polylines(wall_layout, config)
        if len(polyline_collisions) != len(expected_polylines) or len(
            polyline_visuals
        ) != len(expected_polylines):
            raise SdfExportError(
                f"Expected {len(expected_polylines)} solid polyline pairs, got "
                f"{len(polyline_collisions)}/{len(polyline_visuals)}"
            )
        return

    solid_box_collisions = [
        item for item in box_collisions if (item.get("name") or "").startswith("solid_")
    ]
    solid_box_visuals = [
        item for item in box_visuals if (item.get("name") or "").startswith("solid_")
    ]
    if len(solid_box_collisions) != len(solid_box_visuals):
        raise SdfExportError(
            f"Solid box collision/visual mismatch: "
            f"{len(solid_box_collisions)}/{len(solid_box_visuals)}"
        )
    if len(mesh_collisions) != len(mesh_visuals):
        raise SdfExportError(
            f"Solid mesh collision/visual mismatch: {len(mesh_collisions)}/{len(mesh_visuals)}"
        )
    if polyline_collisions or polyline_visuals:
        raise SdfExportError("Hybrid SDF export must not contain solid polylines")

    expected_solids = collect_tagged_solids(wall_layout)
    if len(solid_box_collisions) + len(mesh_collisions) < len(expected_solids):
        raise SdfExportError(
            f"Expected at least {len(expected_solids)} solid geometries, got "
            f"{len(solid_box_collisions) + len(mesh_collisions)}"
        )

    for mesh in mesh_collisions + mesh_visuals:
        uri = mesh.findtext("./geometry/mesh/uri")
        if uri is None or not uri.startswith("file://"):
            raise SdfExportError(f"Solid mesh has invalid URI: {uri!r}")
        mesh_path = Path(uri.removeprefix("file://"))
        if not mesh_path.is_file():
            raise SdfExportError(f"Solid mesh file not found: {mesh_path}")


def _validate_ground_model(world: ET.Element, config: Config) -> None:
    models = world.findall("model")
    ground_model = next(
        (model for model in models if model.get("name") == "ground"), None
    )
    if ground_model is None:
        raise SdfExportError("SDF world must contain a ground model")
    if not _model_is_static(ground_model):
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
    solids: SolidGeometryPlan,
    config: Config,
) -> ET.ElementTree:
    sdf = ET.Element("sdf", version="1.10")
    world = ET.SubElement(sdf, "world", name="generated_world")
    _append_world_environment(world)
    _append_ground_model(world, config)
    model = ET.SubElement(world, "model", name="walls")
    static = ET.SubElement(model, "static")
    static.text = "true"
    link = ET.SubElement(model, "link", name="walls_link")

    for box in boxes:
        _append_box(link, f"{box.name}_collision", box, kind="collision")
        _append_box(link, f"{box.name}_visual", box, kind="visual")

    for box in solids.boxes:
        _append_box(link, f"{box.name}_collision", box, kind="collision")
        _append_box(link, f"{box.name}_visual", box, kind="visual")

    for polyline in solids.polylines:
        _append_polyline(link, f"{polyline.name}_collision", polyline, kind="collision")
        _append_polyline(link, f"{polyline.name}_visual", polyline, kind="visual")

    for mesh in solids.meshes:
        _append_mesh(link, f"{mesh.name}_collision", mesh, kind="collision")
        _append_mesh(link, f"{mesh.name}_visual", mesh, kind="visual")

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


def _append_mesh(
    link: ET.Element,
    name: str,
    mesh: SolidMesh,
    *,
    kind: str,
) -> None:
    element = ET.SubElement(link, kind, name=name)
    pose = ET.SubElement(element, "pose")
    pose.text = "0 0 0 0 0 0"

    geometry = ET.SubElement(element, "geometry")
    mesh_geometry = ET.SubElement(geometry, "mesh")
    uri = ET.SubElement(mesh_geometry, "uri")
    uri.text = mesh.uri

    if kind == "visual":
        _append_visual_material(element)


def _append_ground_model(world: ET.Element, config: Config) -> None:
    box = ground_box(config)
    model = ET.SubElement(world, "model", name="ground")
    static = ET.SubElement(model, "static")
    static.text = "true"
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

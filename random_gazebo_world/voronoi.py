from __future__ import annotations

import random

import numpy as np
from scipy.spatial import Voronoi
from shapely.geometry import Polygon
from shapely.geometry import box as shapely_box

from random_gazebo_world.config import Config
from random_gazebo_world.geometry import EPS, Cell, Vec2
from random_gazebo_world.partition import Partition, PartitionError


def generate_voronoi_partition(config: Config, rng: random.Random) -> Partition:
    world = shapely_box(0.0, 0.0, config.world_width, config.world_height)
    seeds = _sample_seeds(config, rng)

    polygons = _clip_voronoi(seeds, world)
    for _ in range(config.voronoi_lloyd_iterations):
        seeds = np.array([_polygon_centroid(poly) for poly in polygons])
        polygons = _clip_voronoi(seeds, world)

    cells: list[Cell] = []
    for index, polygon in enumerate(polygons):
        vertices = _polygon_to_vertices(polygon)
        if vertices is None:
            raise PartitionError(
                f"Voronoi region {index} is degenerate after clipping"
            )
        cells.append(Cell.from_polygon(index, vertices))

    partition = Partition(
        cells=tuple(cells),
        world_width=config.world_width,
        world_height=config.world_height,
    )
    validate_voronoi_partition(partition, config)
    return partition


def validate_voronoi_partition(partition: Partition, config: Config) -> None:
    if not partition.cells:
        raise PartitionError("Voronoi partition produced no cells")

    total_area = 0.0
    for cell in partition.cells:
        area = cell.area
        if area < config.voronoi_min_cell_area - EPS:
            raise PartitionError(
                f"Cell {cell.id} area {area:.4f} below voronoi_min_cell_area"
            )
        if area > config.voronoi_max_cell_area + EPS:
            raise PartitionError(
                f"Cell {cell.id} area {area:.4f} above voronoi_max_cell_area"
            )
        if (
            cell.x_min < -EPS
            or cell.y_min < -EPS
            or cell.x_max > partition.world_width + EPS
            or cell.y_max > partition.world_height + EPS
        ):
            raise PartitionError(f"Cell {cell.id} extends beyond the world bounds")
        total_area += area

    expected_area = partition.world_width * partition.world_height
    if abs(total_area - expected_area) > 1e-4:
        raise PartitionError(
            f"Voronoi area {total_area:.4f} does not tile world area {expected_area:.4f}"
        )

    for left_index, left in enumerate(partition.cells):
        left_poly = left.polygon
        for right in partition.cells[left_index + 1 :]:
            overlap = left_poly.intersection(right.polygon).area
            if overlap > 1e-4:
                raise PartitionError(
                    f"Cells {left.id} and {right.id} overlap by {overlap:.4f}"
                )


def _sample_seeds(config: Config, rng: random.Random) -> np.ndarray:
    count = config.voronoi_seed_count
    points = [
        (
            rng.uniform(0.0, config.world_width),
            rng.uniform(0.0, config.world_height),
        )
        for _ in range(count)
    ]
    return np.array(points, dtype=float)


def _clip_voronoi(seeds: np.ndarray, world: Polygon) -> list[Polygon]:
    if len(seeds) < 2:
        raise PartitionError("Voronoi partition needs at least 2 seeds")

    regions, vertices = _finite_polygons(seeds, world)
    polygons: list[Polygon] = []
    for region in regions:
        polygon = Polygon(vertices[region])
        clipped = polygon.intersection(world)
        if clipped.is_empty or clipped.geom_type != "Polygon" or clipped.area <= EPS:
            # Pick the largest polygonal piece if the clip split the region.
            clipped = _largest_polygon(clipped)
        if clipped is None or clipped.area <= EPS:
            raise PartitionError("Voronoi region collapsed during clipping")
        polygons.append(clipped)
    return polygons


def _finite_polygons(
    seeds: np.ndarray,
    world: Polygon,
) -> tuple[list[np.ndarray], np.ndarray]:
    try:
        vor = Voronoi(seeds)
    except Exception as exc:  # noqa: BLE001 - scipy/QHull raises a variety of errors
        raise PartitionError(f"Voronoi computation failed: {exc}") from exc

    radius = float(np.ptp(seeds, axis=0).max()) * 4.0 + 1.0
    center = seeds.mean(axis=0)
    new_vertices = vor.vertices.tolist()

    ridges: dict[int, list[tuple[int, int, int]]] = {}
    for (p1, p2), (v1, v2) in zip(vor.ridge_points, vor.ridge_vertices, strict=True):
        ridges.setdefault(p1, []).append((p2, v1, v2))
        ridges.setdefault(p2, []).append((p1, v1, v2))

    new_regions: list[np.ndarray] = []
    for point_index, region_index in enumerate(vor.point_region):
        region = vor.regions[region_index]
        if region and all(v >= 0 for v in region):
            new_regions.append(np.array(region))
            continue

        finite = [v for v in region if v >= 0]
        for other, v1, v2 in ridges.get(point_index, []):
            if v2 < 0:
                v1, v2 = v2, v1
            if v1 >= 0:
                continue
            tangent = seeds[other] - seeds[point_index]
            tangent /= np.linalg.norm(tangent)
            normal = np.array([-tangent[1], tangent[0]])
            midpoint = seeds[[point_index, other]].mean(axis=0)
            direction = np.sign(np.dot(midpoint - center, normal)) * normal
            far_point = vor.vertices[v2] + direction * radius
            finite.append(len(new_vertices))
            new_vertices.append(far_point.tolist())

        finite_points = np.array([new_vertices[v] for v in finite])
        centroid = finite_points.mean(axis=0)
        angles = np.arctan2(
            finite_points[:, 1] - centroid[1],
            finite_points[:, 0] - centroid[0],
        )
        ordered = np.array(finite)[np.argsort(angles)]
        new_regions.append(ordered)

    return new_regions, np.asarray(new_vertices)


def _largest_polygon(geometry) -> Polygon | None:
    if geometry.is_empty:
        return None
    if geometry.geom_type == "Polygon":
        return geometry
    if geometry.geom_type in {"MultiPolygon", "GeometryCollection"}:
        best: Polygon | None = None
        for part in geometry.geoms:
            if part.geom_type != "Polygon" or part.is_empty:
                continue
            if best is None or part.area > best.area:
                best = part
        return best
    return None


def _polygon_centroid(polygon: Polygon) -> tuple[float, float]:
    centroid = polygon.centroid
    return (centroid.x, centroid.y)


def _polygon_to_vertices(polygon: Polygon) -> tuple[Vec2, ...] | None:
    coords = list(polygon.exterior.coords)
    if len(coords) < 2:
        return None
    # Drop the closing duplicate vertex.
    coords = coords[:-1]
    cleaned: list[Vec2] = []
    for x, y in coords:
        point = (float(x), float(y))
        if not cleaned or (
            abs(point[0] - cleaned[-1][0]) > EPS or abs(point[1] - cleaned[-1][1]) > EPS
        ):
            cleaned.append(point)
    if len(cleaned) >= 2 and (
        abs(cleaned[0][0] - cleaned[-1][0]) <= EPS
        and abs(cleaned[0][1] - cleaned[-1][1]) <= EPS
    ):
        cleaned.pop()
    if len(cleaned) < 3:
        return None
    return tuple(cleaned)

"""Provisional bed geometry for offline and MoveIt previews."""

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Tuple

from .geometry import (CirclePlan, Pose, Vector3, _finite_scalar, _finite_vector,
                       compose_pose, generate_circle_plan, inverse_pose,
                       quaternion_from_rpy)


IDENTITY = (0.0, 0.0, 0.0, 1.0)
CORNER_NAMES = ('top_left', 'top_right', 'bottom_right', 'bottom_left')


@dataclass(frozen=True)
class BedPreview:
    corners: Tuple[Vector3, ...]
    surface: Pose
    path: CirclePlan
    start_ee: Pose
    radius: float
    edge_clearance: float
    clearance: float
    layers: int
    layer_step: float
    samples_per_circle: int
    max_fit_residual: float

    def summary(self):
        return {
            'frame': 'KUKA World',
            'preview_only': True,
            'tcp_source': 'Rev F CAD offset',
            'height_source': 'plane fit to estimated nozzle positions; surface Z uncalibrated',
            'corners_world_mm': dict(zip(CORNER_NAMES, ([v * 1000 for v in p] for p in self.corners))),
            'center_on_estimated_plane_world_mm': [v * 1000 for v in self.surface.position],
            'first_tcp_target_world_mm': [v * 1000 for v in self.path.start_tcp.position],
            'bed_normal_world': self.path.plane_normal,
            'radius_mm': self.radius * 1000,
            'minimum_nominal_edge_clearance_mm': self.edge_clearance * 1000,
            'preview_clearance_mm': self.clearance * 1000,
            'layers': self.layers,
            'layer_step_mm': self.layer_step * 1000,
            'samples_per_circle': self.samples_per_circle,
            'max_vertical_fit_residual_mm': self.max_fit_residual * 1000,
            'estimated_z_range_across_circle_mm': 2000 * self.radius * math.hypot(*self.path.plane_normal[:2]),
        }


def fit_surface(corners):
    """Fit z = a*x + b*y + c and keep the fitted plane at the corner mean."""
    center = tuple(sum(point[i] for point in corners) / len(corners) for i in range(3))
    centered = [tuple(value - mean for value, mean in zip(point, center)) for point in corners]
    xx = sum(x*x for x, _, _ in centered)
    xy = sum(x*y for x, y, _ in centered)
    yy = sum(y*y for _, y, _ in centered)
    xz = sum(x*z for x, _, z in centered)
    yz = sum(y*z for _, y, z in centered)
    determinant = xx*yy - xy*xy
    if determinant <= 1e-12:
        raise ValueError('Bed corners must span an XY footprint')
    a = (yy*xz - xy*yz) / determinant
    b = (xx*yz - xy*xz) / determinant
    residual = max(abs(z - a*x - b*y) for x, y, z in centered)
    orientation = quaternion_from_rpy(math.atan2(b, math.hypot(1, a)), -math.atan(a), 0.0)
    return Pose(center, orientation), residual


def edge_clearance(local_corners, radius):
    """Clearance of a full circle centered at bed-local (0, 0) to a convex footprint."""
    edge_distances = []
    for index, (x, y, _) in enumerate(local_corners):
        end_x, end_y, _ = local_corners[(index + 1) % len(local_corners)]
        dx, dy = end_x - x, end_y - y
        length = math.hypot(dx, dy)
        if length < 1e-6:
            raise ValueError('Bed footprint has a duplicate corner')
        center_side = dy*x - dx*y
        if center_side == 0 or any(
            center_side * (dx*(py-y) - dy*(px-x)) <= 0
            for other, (px, py, _) in enumerate(local_corners)
            if other not in (index, (index + 1) % len(local_corners))
        ):
            raise ValueError('Bed corners must be listed in order around a convex footprint')
        edge_distances.append(abs(center_side) / length)
    return min(edge_distances) - radius


def load_bed_preview(config_path):
    data = json.loads(Path(config_path).read_text())
    corners = tuple(tuple(v / 1000 for v in _finite_vector(data['corners_world_mm'][name], 3, name))
                    for name in CORNER_NAMES)
    surface, residual = fit_surface(corners)
    bed_from_world = inverse_pose(surface)
    local_corners = [compose_pose(bed_from_world, Pose(point, IDENTITY)).position for point in corners]
    radius, height, margin, pitch = (
        _finite_scalar(data[name], name) / 1000
        for name in ('radius_mm', 'preview_clearance_mm', 'minimum_edge_margin_mm', 'layer_step_mm')
    )
    if min(radius, height, margin) <= 0:
        raise ValueError('Preview radius, clearance and minimum edge margin must be positive')
    clearance = edge_clearance(local_corners, radius)
    if clearance < margin:
        raise ValueError('Circle does not fit inside the configured bed edge margin')

    a, b, c = (math.radians(v) for v in _finite_vector(data['flange_abc_deg'], 3, 'flange_abc_deg'))
    flange_orientation = quaternion_from_rpy(c, b, a)
    offset = tuple(v / 1000 for v in _finite_vector(data['nozzle_offset_flange_mm'], 3, 'nozzle_offset_flange_mm'))
    bed_orientation = compose_pose(bed_from_world, Pose((0.0, 0.0, 0.0), flange_orientation)).orientation
    local_start_tcp = Pose((radius, 0.0, height), bed_orientation)
    local_start_ee = compose_pose(local_start_tcp, Pose(tuple(-v for v in offset), IDENTITY))
    local_path = generate_circle_plan(local_start_ee, radius=radius, layers=data['layers'], layer_step=pitch,
                                      samples_per_circle=data['samples_per_circle'], tcp_offset_xyz=offset)
    normal = compose_pose(Pose((0.0, 0.0, 0.0), surface.orientation), Pose((0.0, 0.0, 1.0), IDENTITY)).position
    path = CirclePlan(compose_pose(surface, local_path.start_tcp),
                      tuple(compose_pose(surface, pose) for pose in local_path.tcp_waypoints),
                      tuple(compose_pose(surface, pose) for pose in local_path.ee_waypoints), normal)
    return BedPreview(corners, surface, path, compose_pose(surface, local_start_ee), radius, clearance,
                      height, data['layers'], pitch, data['samples_per_circle'], residual)

"""ROS-independent circle paths and rigid transforms, in metres and radians.

Quaternions use ROS ``(x, y, z, w)`` ordering. Plane axes are expressed in
the fixed reference frame of the supplied end-effector pose, not its tool frame.
"""

from dataclasses import dataclass
import math
from numbers import Integral, Real
from typing import Iterable, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


def _finite_scalar(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError("{} must be a finite real number".format(name))
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("{} must be a finite real number".format(name))
    return result


def _finite_vector(values, size: int, name: str) -> tuple:
    try:
        items = tuple(values)
    except TypeError as exc:
        raise ValueError("{} must contain {} numbers".format(name, size)) from exc
    if len(items) != size:
        raise ValueError("{} must contain {} numbers".format(name, size))
    return tuple(_finite_scalar(value, name) for value in items)


@dataclass(frozen=True)
class Pose:
    """Rigid pose; finite nonzero quaternions are normalized on construction."""

    position: Vector3
    orientation: Quaternion

    def __post_init__(self) -> None:
        position = _finite_vector(self.position, 3, "position")
        quaternion = _finite_vector(self.orientation, 4, "orientation")
        # Scaling first also handles finite quaternions with very large values.
        scale = max(abs(value) for value in quaternion)
        if scale == 0.0:
            raise ValueError("orientation quaternion must be nonzero")
        scaled = tuple(value / scale for value in quaternion)
        norm = math.sqrt(sum(value * value for value in scaled))
        normalized = tuple(value / norm for value in scaled)
        object.__setattr__(self, "position", position)
        object.__setattr__(self, "orientation", normalized)


@dataclass(frozen=True)
class CirclePlan:
    """Matched desired TCP and commanded EE poses, excluding the initial pose."""

    start_tcp: Pose
    tcp_waypoints: Tuple[Pose, ...]
    ee_waypoints: Tuple[Pose, ...]
    plane_normal: Vector3


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Return fixed-axis RPY rotation ``Rz(yaw) Ry(pitch) Rx(roll)``."""
    roll = _finite_scalar(roll, "roll")
    pitch = _finite_scalar(pitch, "pitch")
    yaw = _finite_scalar(yaw, "yaw")
    sr, cr = math.sin(roll / 2.0), math.cos(roll / 2.0)
    sp, cp = math.sin(pitch / 2.0), math.cos(pitch / 2.0)
    sy, cy = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate(q: Quaternion, vector: Vector3) -> Vector3:
    x, y, z, w = q
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def compose_pose(a: Pose, b: Pose) -> Pose:
    """Compose ``reference_from_a`` and ``a_from_b`` into ``reference_from_b``."""
    if not isinstance(a, Pose) or not isinstance(b, Pose):
        raise ValueError("compose_pose requires two Pose instances")
    rotated = _rotate(a.orientation, b.position)
    return Pose(
        tuple(a.position[index] + rotated[index] for index in range(3)),
        _multiply(a.orientation, b.orientation),
    )


def inverse_pose(pose: Pose) -> Pose:
    """Invert a rigid pose without changing its reference-frame convention."""
    if not isinstance(pose, Pose):
        raise ValueError("inverse_pose requires a Pose instance")
    x, y, z, w = pose.orientation
    inverse_q = (-x, -y, -z, w)
    return Pose(_rotate(inverse_q, tuple(-v for v in pose.position)), inverse_q)


def generate_circle_plan(
    current_ee_pose: Pose,
    *,
    radius: float = 0.02,
    layers: int = 3,
    layer_step: float = 0.005,
    samples_per_circle: int = 60,
    plane: str = "xy",
    tcp_offset_xyz: Vector3 = (0.0, 0.0, 0.0),
    tcp_offset_rpy: Vector3 = (0.0, 0.0, 0.0)
) -> CirclePlan:
    """Generate closed rings anchored at the current TCP, with fixed attitude.

    ``tcp_offset_xyz/rpy`` describe EE-from-TCP. The initial TCP is the first
    ring's start, so its centre is ``start - radius * u``. Each ring contains
    ``samples_per_circle`` targets, including its exact closing point; the
    initial point is not emitted. Later layers add one normal-direction
    connection target before their ring. For planes xy/yz/xz, (u, v, normal)
    are (+x,+y,+z), (+y,+z,+x), and (+x,+z,-y), respectively. Layer connection
    targets describe travel geometry, not an instruction to extrude.
    """
    if not isinstance(current_ee_pose, Pose):
        raise ValueError("current_ee_pose must be a Pose instance")
    radius = _finite_scalar(radius, "radius")
    layer_step = _finite_scalar(layer_step, "layer_step")
    if radius <= 0.0:
        raise ValueError("radius must be positive")
    if isinstance(layers, bool) or not isinstance(layers, Integral) or layers < 1:
        raise ValueError("layers must be an integer >= 1")
    if (
        isinstance(samples_per_circle, bool)
        or not isinstance(samples_per_circle, Integral)
        or samples_per_circle < 8
    ):
        raise ValueError("samples_per_circle must be an integer >= 8")
    if layer_step < 0.0 or (layers > 1 and layer_step <= 0.0):
        raise ValueError("layer_step must be nonnegative and positive for multiple layers")
    bases = {
        "xy": ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        "yz": ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
        "xz": ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0)),
    }
    if not isinstance(plane, str) or plane not in bases:
        raise ValueError("plane must be one of xy, yz, xz")
    u, v, normal = bases[plane]
    offset_xyz = _finite_vector(tcp_offset_xyz, 3, "tcp_offset_xyz")
    offset_rpy = _finite_vector(tcp_offset_rpy, 3, "tcp_offset_rpy")
    ee_from_tcp = Pose(offset_xyz, quaternion_from_rpy(*offset_rpy))
    tcp_from_ee = inverse_pose(ee_from_tcp)
    start_tcp = compose_pose(current_ee_pose, ee_from_tcp)
    tcp_waypoints = []
    for layer in range(layers):
        layer_start = tuple(
            start_tcp.position[index] + layer * layer_step * normal[index]
            for index in range(3)
        )
        if layer > 0:
            tcp_waypoints.append(Pose(layer_start, start_tcp.orientation))
        for sample in range(1, samples_per_circle + 1):
            if sample == samples_per_circle:
                position = layer_start
            else:
                angle = math.tau * sample / samples_per_circle
                position = tuple(
                    layer_start[index]
                    + radius * (math.cos(angle) - 1.0) * u[index]
                    + radius * math.sin(angle) * v[index]
                    for index in range(3)
                )
            tcp_waypoints.append(Pose(position, start_tcp.orientation))
    ee_waypoints = tuple(compose_pose(pose, tcp_from_ee) for pose in tcp_waypoints)
    return CirclePlan(start_tcp, tuple(tcp_waypoints), ee_waypoints, normal)


def point_distances(poses: Iterable[Pose]) -> Tuple[float, ...]:
    """Return distances between adjacent poses; include the initial pose explicitly."""
    iterator = iter(poses)
    try:
        previous = next(iterator)
    except StopIteration:
        return ()
    if not isinstance(previous, Pose):
        raise ValueError("poses must contain Pose instances")
    distances = []
    for pose in iterator:
        if not isinstance(pose, Pose):
            raise ValueError("poses must contain Pose instances")
        distances.append(math.sqrt(sum(
            (pose.position[index] - previous.position[index]) ** 2
            for index in range(3)
        )))
        previous = pose
    return tuple(distances)


def path_length(poses: Iterable[Pose]) -> float:
    """Return polygonal position-path length, ignoring orientation changes."""
    return math.fsum(point_distances(poses))

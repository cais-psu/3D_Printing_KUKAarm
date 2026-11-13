"""Geometry checks run without ROS or connected hardware."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from kmriiwa_circle_demo.geometry import (
    Pose,
    compose_pose,
    generate_circle_plan,
    inverse_pose,
    path_length,
    point_distances,
    quaternion_from_rpy,
)


IDENTITY = (0.0, 0.0, 0.0, 1.0)


class GeometryTests(unittest.TestCase):
    def assertVectorClose(self, actual, expected, places=12):
        self.assertEqual(len(actual), len(expected))
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=places)

    def assertQuaternionEquivalent(self, actual, expected):
        dot = sum(a * b for a, b in zip(actual, expected))
        self.assertAlmostEqual(abs(dot), 1.0, places=12)

    def test_pose_validates_and_normalizes(self):
        pose = Pose([1, 2, 3], (0, 0, 0, 5))
        self.assertEqual(pose.position, (1.0, 2.0, 3.0))
        self.assertEqual(pose.orientation, IDENTITY)
        large = Pose((0, 0, 0), (1e308, 1e308, 0, 0))
        self.assertAlmostEqual(sum(v * v for v in large.orientation), 1.0)

    def test_composition_rotates_tool_translation(self):
        reference_from_ee = Pose((1, 2, 3), quaternion_from_rpy(0, 0, math.pi / 2))
        ee_from_tcp = Pose((0.1, 0, 0), IDENTITY)
        tcp = compose_pose(reference_from_ee, ee_from_tcp)
        self.assertVectorClose(tcp.position, (1, 2.1, 3))
        self.assertQuaternionEquivalent(tcp.orientation, reference_from_ee.orientation)

    def test_inverse_roundtrip_with_all_rotation_axes(self):
        pose = Pose((0.1, -0.3, 0.7), quaternion_from_rpy(0.4, -0.7, 1.2))
        for result in (compose_pose(pose, inverse_pose(pose)), compose_pose(inverse_pose(pose), pose)):
            self.assertVectorClose(result.position, (0, 0, 0))
            self.assertQuaternionEquivalent(result.orientation, IDENTITY)

    def test_rpy_order(self):
        pose = Pose((0, 0, 0), quaternion_from_rpy(math.pi / 2, 0, math.pi / 2))
        result = compose_pose(pose, Pose((0, 1, 0), IDENTITY))
        self.assertVectorClose(result.position, (0, 0, 1))

    def test_default_count_and_first_target_not_start(self):
        start = Pose((0.4, 0.1, 0.3), IDENTITY)
        plan = generate_circle_plan(start)
        self.assertEqual(len(plan.tcp_waypoints), 182)
        self.assertEqual(len(plan.ee_waypoints), 182)
        self.assertNotEqual(plan.tcp_waypoints[0].position, plan.start_tcp.position)
        self.assertEqual(plan.tcp_waypoints[59].position, plan.start_tcp.position)

    def test_closed_rings_and_layer_connections_all_planes(self):
        start = Pose((0.4, 0.1, 0.3), quaternion_from_rpy(0.2, -0.1, 0.3))
        bases = {
            "xy": ((1, 0, 0), (0, 0, 1)),
            "yz": ((0, 1, 0), (1, 0, 0)),
            "xz": ((1, 0, 0), (0, -1, 0)),
        }
        radius, step, samples = 0.02, 0.005, 16
        for plane, (u, normal) in bases.items():
            with self.subTest(plane=plane):
                plan = generate_circle_plan(start, plane=plane, samples_per_circle=samples)
                self.assertEqual(plan.plane_normal, normal)
                offset = 0
                for layer in range(3):
                    layer_start = tuple(start.position[i] + layer * step * normal[i] for i in range(3))
                    if layer:
                        self.assertVectorClose(plan.tcp_waypoints[offset].position, layer_start)
                        previous = plan.tcp_waypoints[offset - 1]
                        distance = point_distances((previous, plan.tcp_waypoints[offset]))[0]
                        self.assertAlmostEqual(distance, step)
                        offset += 1
                    ring = plan.tcp_waypoints[offset:offset + samples]
                    self.assertEqual(ring[-1].position, layer_start)
                    center = tuple(layer_start[i] - radius * u[i] for i in range(3))
                    for point in ring:
                        delta = tuple(point.position[i] - center[i] for i in range(3))
                        self.assertAlmostEqual(math.sqrt(sum(v * v for v in delta)), radius)
                        self.assertAlmostEqual(sum(delta[i] * normal[i] for i in range(3)), 0.0)
                        self.assertQuaternionEquivalent(point.orientation, start.orientation)
                    offset += samples
                self.assertEqual(offset, len(plan.tcp_waypoints))

    def test_positive_circle_direction_and_xz_normal(self):
        start = Pose((0, 0, 0), IDENTITY)
        cases = {"xy": (-0.02, 0.02, 0), "yz": (0, -0.02, 0.02), "xz": (-0.02, 0, 0.02)}
        for plane, quarter_turn in cases.items():
            plan = generate_circle_plan(start, plane=plane, samples_per_circle=8, layers=1)
            self.assertVectorClose(plan.tcp_waypoints[1].position, quarter_turn)

    def test_tcp_offset_backcalculation_preserves_desired_path(self):
        start = Pose((0.4, 0.1, 0.3), quaternion_from_rpy(0.2, -0.4, 0.5))
        xyz, rpy = (0.03, -0.01, 0.15), (0.1, -0.2, 0.3)
        offset = Pose(xyz, quaternion_from_rpy(*rpy))
        plan = generate_circle_plan(start, tcp_offset_xyz=xyz, tcp_offset_rpy=rpy)
        actual_initial_tcp = compose_pose(start, offset)
        self.assertVectorClose(plan.start_tcp.position, actual_initial_tcp.position)
        self.assertQuaternionEquivalent(plan.start_tcp.orientation, actual_initial_tcp.orientation)
        for desired_tcp, command_ee in zip(plan.tcp_waypoints, plan.ee_waypoints):
            reconstructed = compose_pose(command_ee, offset)
            self.assertVectorClose(reconstructed.position, desired_tcp.position)
            self.assertQuaternionEquivalent(reconstructed.orientation, desired_tcp.orientation)
            self.assertQuaternionEquivalent(command_ee.orientation, start.orientation)
        self.assertVectorClose(plan.ee_waypoints[59].position, start.position)

    def test_path_length_matches_polygon_and_connections(self):
        plan = generate_circle_plan(Pose((0, 0, 0), IDENTITY))
        poses = (plan.start_tcp,) + plan.tcp_waypoints
        expected = 3 * 60 * 2 * 0.02 * math.sin(math.pi / 60) + 2 * 0.005
        self.assertAlmostEqual(path_length(poses), expected, places=12)
        self.assertAlmostEqual(point_distances(poses)[0], 2 * 0.02 * math.sin(math.pi / 60))
        self.assertEqual(path_length([]), 0)
        self.assertEqual(point_distances([plan.start_tcp]), ())

    def test_single_layer_allows_zero_step(self):
        plan = generate_circle_plan(Pose((0, 0, 0), IDENTITY), layers=1, layer_step=0)
        self.assertEqual(len(plan.tcp_waypoints), 60)

    def test_invalid_pose_values(self):
        bad_poses = [
            ((0, 0), IDENTITY),
            ((0, 0, float("nan")), IDENTITY),
            ((0, True, 0), IDENTITY),
            ((0, 0, 0), (0, 0, 0, 0)),
            ((0, 0, 0), (0, 0, 1)),
            ((0, 0, 0), (0, 0, float("inf"), 1)),
        ]
        for position, orientation in bad_poses:
            with self.subTest(position=position, orientation=orientation):
                with self.assertRaises(ValueError):
                    Pose(position, orientation)

    def test_invalid_circle_inputs(self):
        start = Pose((0, 0, 0), IDENTITY)
        bad = [
            {"radius": 0}, {"radius": -0.1}, {"radius": float("inf")},
            {"radius": True}, {"layers": 0}, {"layers": 1.5}, {"layers": True},
            {"samples_per_circle": 7}, {"samples_per_circle": 8.0},
            {"samples_per_circle": True}, {"layer_step": 0},
            {"layer_step": -0.1}, {"layer_step": float("nan")},
            {"plane": "bad"}, {"plane": []},
            {"tcp_offset_xyz": (0, 1)},
            {"tcp_offset_xyz": (0, 0, float("nan"))},
            {"tcp_offset_rpy": (0, float("inf"), 0)},
        ]
        for arguments in bad:
            with self.subTest(arguments=arguments):
                with self.assertRaises(ValueError):
                    generate_circle_plan(start, **arguments)


if __name__ == "__main__":
    unittest.main()

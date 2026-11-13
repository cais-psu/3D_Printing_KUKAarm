"""Validate final MoveIt messages using plain fixtures, without ROS imports."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from kmriiwa_circle_demo.validation import (
    DRIVER_POINT_LIMIT,
    joint_positions,
    positive_number,
    require_start_match,
    validate_trajectory,
)


NAMES = ["kmriiwa_joint_" + str(index) for index in range(1, 8)]


def start_positions():
    return dict(zip(NAMES, [0.0] * 7))


def point(positions=None, stamp=0.1):
    return SimpleNamespace(
        positions=[0.0] * 7 if positions is None else positions,
        time_from_start=SimpleNamespace(to_sec=lambda: stamp),
    )


def trajectory(count=2, names=None):
    return SimpleNamespace(
        joint_trajectory=SimpleNamespace(
            joint_names=list(NAMES if names is None else names),
            points=[point([0.001 * index] * 7, 0.1 * (index + 1))
                    for index in range(count)],
        ),
        multi_dof_joint_trajectory=SimpleNamespace(joint_names=[], points=[]),
    )


class ValidationTests(unittest.TestCase):
    def validate(self, message, fraction=1.0, start=None, max_step=0.25):
        return validate_trajectory(
            message, fraction, start_positions() if start is None else start, max_step)

    def test_valid_message_returns_final_point_count_and_planned_time(self):
        result = self.validate(trajectory(3))
        self.assertEqual(result["trajectory_points"], 3)
        self.assertAlmostEqual(result["planned_duration_s"], 0.3)
        self.assertAlmostEqual(result["largest_joint_step_rad"], 0.001)
        self.assertEqual(result["fraction"], 1.0)

    def test_rejects_partial_and_nonfinite_cartesian_fraction(self):
        for fraction in (0.0, 0.5, 0.99, -1.0, 1.1, float("nan"), float("inf")):
            with self.subTest(fraction=fraction):
                with self.assertRaises(ValueError):
                    self.validate(trajectory(), fraction=fraction)

    def test_driver_limit_is_applied_to_final_interpolated_point_count(self):
        # A small waypoint input may expand during Cartesian interpolation.
        self.assertEqual(DRIVER_POINT_LIMIT, 500)
        self.assertEqual(self.validate(trajectory(500))["trajectory_points"], 500)
        with self.assertRaises(ValueError):
            self.validate(trajectory(501))
        with self.assertRaises(ValueError):
            self.validate(trajectory(0))

    def test_physical_joint_order_is_required_even_when_name_sets_match(self):
        swapped = list(NAMES)
        swapped[0], swapped[1] = swapped[1], swapped[0]
        with self.assertRaises(ValueError):
            self.validate(trajectory(names=swapped))
        for names in (NAMES[:-1], NAMES + ["extra_joint"], NAMES[:-1] + [NAMES[0]]):
            with self.subTest(names=names):
                with self.assertRaises(ValueError):
                    self.validate(trajectory(names=names))

    def test_rejects_raw_two_pi_jump_without_angle_wrapping(self):
        message = trajectory()
        message.joint_trajectory.points[1].positions[4] = 2 * math.pi + 0.001
        with self.assertRaises(ValueError):
            self.validate(message)

    def test_first_target_jump_is_checked_against_measured_start(self):
        message = trajectory(1)
        message.joint_trajectory.points[0].positions[0] = 0.3
        with self.assertRaises(ValueError):
            self.validate(message)

    def test_rejects_invalid_position_dimensions_and_nonfinite_values(self):
        variants = ([0.0] * 6, [0.0] * 8,
                    [0.0] * 6 + [float("nan")],
                    [0.0] * 6 + [float("inf")])
        for positions in variants:
            with self.subTest(positions=positions):
                message = trajectory()
                message.joint_trajectory.points[-1].positions = positions
                with self.assertRaises(ValueError):
                    self.validate(message)

    def test_rejects_nonincreasing_negative_or_nonfinite_timestamps(self):
        for stamps in ((0.2, 0.1), (0.1, 0.1), (-0.1, 0.1),
                       (0.0, 0.0), (0.1, float("nan")),
                       (0.1, float("inf"))):
            with self.subTest(stamps=stamps):
                message = trajectory()
                message.joint_trajectory.points = [point(stamp=stamp) for stamp in stamps]
                with self.assertRaises(ValueError):
                    self.validate(message)
        with self.assertRaises(ValueError):
            message = trajectory(1)
            message.joint_trajectory.points = [point(stamp=0.0)]
            self.validate(message)

    def test_zero_initial_timestamp_is_valid_with_positive_final_duration(self):
        message = trajectory()
        message.joint_trajectory.points = [point(stamp=0), point(stamp=1)]
        self.assertEqual(self.validate(message)["planned_duration_s"], 1)

    def test_rejects_mobile_base_components(self):
        for names, points in ((["base_joint"], []), ([], [object()])):
            with self.subTest(names=names):
                message = trajectory()
                message.multi_dof_joint_trajectory = SimpleNamespace(joint_names=names, points=points)
                with self.assertRaises(ValueError):
                    self.validate(message)

    def test_joint_feedback_reorders_names_and_ignores_valid_extra_joint(self):
        values = [0.1 * index for index in range(7)]
        actual = joint_positions(
            ["extra_joint"] + list(reversed(NAMES)),
            [10.0] + list(reversed(values)), NAMES)
        self.assertEqual(list(actual), NAMES)
        self.assertEqual(list(actual.values()), values)

    def test_joint_feedback_rejects_missing_duplicate_mismatched_or_nan(self):
        variants = ((NAMES[:-1], [0.0] * 6),
                    (NAMES[:-1] + [NAMES[0]], [0.0] * 7),
                    (NAMES, [0.0] * 6),
                    (NAMES, [0.0] * 6 + [float("nan")]),
                    (NAMES, [0.0] * 6 + [float("inf")]))
        for names, positions in variants:
            with self.subTest(names=names, positions=positions):
                with self.assertRaises(ValueError):
                    joint_positions(names, positions, NAMES)

    def test_start_match_accepts_small_drift_and_rejects_large_or_missing(self):
        expected, current = start_positions(), start_positions()
        current[NAMES[4]] = 0.01
        self.assertAlmostEqual(require_start_match(expected, current, 0.02), 0.01)
        current[NAMES[4]] = 0.03
        with self.assertRaises(ValueError):
            require_start_match(expected, current, 0.02)
        current.pop(NAMES[-1])
        with self.assertRaises(ValueError):
            require_start_match(expected, current, 0.02)

    def test_start_match_rejects_nonfinite_values_in_every_joint(self):
        for name in NAMES:
            for bad in (float("nan"), float("inf")):
                with self.subTest(name=name, bad=bad):
                    expected, current = start_positions(), start_positions()
                    current[name] = bad
                    with self.assertRaises(ValueError):
                        require_start_match(expected, current, 0.02)

    def test_positive_controls_reject_zero_negative_boolean_and_nonfinite(self):
        self.assertEqual(positive_number(0.25, "limit"), 0.25)
        for value in (0, -1, True, "0.25", float("nan"), float("inf")):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    positive_number(value, "limit")


if __name__ == "__main__":
    unittest.main()

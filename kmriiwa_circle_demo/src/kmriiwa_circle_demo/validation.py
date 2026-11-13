"""ROS-independent checks on the exact trajectory sent to the old driver."""
import math

DRIVER_POINT_LIMIT = 500


def positive_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise ValueError('%s must be a positive finite number' % name)
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError('%s must be a positive finite number' % name)
    return value


def joint_positions(names, positions, required):
    if len(names) != len(positions) or len(set(names)) != len(names):
        raise ValueError('Joint state has mismatched or duplicate joint names')
    result = dict(zip(names, positions))
    if not set(required).issubset(result):
        raise ValueError('Joint state is missing arm joints')
    if not all(math.isfinite(float(result[n])) for n in required):
        raise ValueError('Joint state contains non-finite arm positions')
    return {n: float(result[n]) for n in required}


def require_start_match(expected, current, tolerance):
    tolerance = positive_number(tolerance, 'max_start_drift')
    if set(expected) != set(current):
        raise ValueError('Current and planned arm joints differ')
    if not expected or not all(math.isfinite(float(values[n]))
                               for values in (expected, current) for n in expected):
        raise ValueError('Current and planned arm positions must be finite and nonempty')
    largest = max(abs(expected[n] - current[n]) for n in expected)
    if not math.isfinite(largest) or largest > tolerance:
        raise ValueError('Arm moved since planning (%.6f rad); replan at the current pose' % largest)
    return largest


def validate_trajectory(trajectory, fraction, start_positions, max_joint_step):
    """Reject partial plans, branch jumps, and the actual SplineJP size overflow.

    Joint differences are raw angles: all iiwa joints have finite limits.
    A modulo-2pi comparison could hide a physically large movement.
    """
    if not math.isfinite(float(fraction)) or not 1.0 - 1e-6 <= fraction <= 1.0 + 1e-6:
        raise ValueError('Cartesian path is incomplete (fraction=%r); nothing will execute' % fraction)
    max_joint_step = positive_number(max_joint_step, 'max_joint_step')
    joint_trajectory = trajectory.joint_trajectory
    names = list(joint_trajectory.joint_names)
    # The original driver ignores joint_names and consumes positions by index.
    if names != list(start_positions):
        raise ValueError('Trajectory joint order must match physical joint_1 ... joint_7')
    points = list(joint_trajectory.points)
    if not points or len(points) > DRIVER_POINT_LIMIT:
        raise ValueError('Trajectory has %d points; old Java driver requires 1..500. '
                         'Reduce samples/layers or increase eef_step, then replan.' % len(points))
    multi = getattr(trajectory, 'multi_dof_joint_trajectory', None)
    if multi is not None and (multi.joint_names or multi.points):
        raise ValueError('This demo supports the seven-joint arm only, no mobile-base trajectory')
    previous = [start_positions[n] for n in names]
    largest = 0.0
    last_time = -1.0
    for index, point in enumerate(points):
        positions = list(point.positions)
        if len(positions) != 7 or not all(math.isfinite(float(x)) for x in positions):
            raise ValueError('Invalid seven-joint positions at trajectory point %d' % index)
        jump = max(abs(float(x) - y) for x, y in zip(positions, previous))
        largest = max(largest, jump)
        if jump > max_joint_step:
            raise ValueError('Joint jump %.6f rad exceeds %.6f at point %d' %
                             (jump, max_joint_step, index))
        stamp = point.time_from_start.to_sec()
        if not math.isfinite(stamp) or stamp < 0 or stamp < last_time or (index and stamp == last_time):
            raise ValueError('Invalid/non-increasing trajectory timing at point %d' % index)
        previous, last_time = positions, stamp
    if last_time <= 0:
        raise ValueError('Trajectory has no positive planned duration')
    return {'trajectory_points': len(points), 'largest_joint_step_rad': largest,
            'planned_duration_s': last_time, 'fraction': float(fraction)}

"""ROS Noetic adapter for the existing KMR MoveIt/FollowJointTrajectory stack."""
import copy
import csv
import datetime
import inspect
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

from .bed_preview import load_bed_preview
from .geometry import Pose, compose_pose, generate_circle_plan, inverse_pose
from .validation import joint_positions, positive_number, require_start_match, validate_trajectory


def cartesian_path(group, waypoints, eef_step):
    """Old/new MoveIt signatures differ; never pass 0.0 as avoid_collisions."""
    options = {'avoid_collisions': True}
    if 'jump_threshold' in inspect.signature(group.compute_cartesian_path).parameters:
        options['jump_threshold'] = 1.5
    return group.compute_cartesian_path(waypoints, eef_step, **options)


class JointMonitor:
    """Wall-clock bounded receipt of complete arm states, independent of /clock."""
    def __init__(self, rospy, names):
        self.rospy, self.names = rospy, names
        self.condition = threading.Condition()
        self.sample = None

    def callback(self, message):
        try:
            positions = joint_positions(message.name, message.position, self.names)
        except (TypeError, ValueError) as error:
            self.rospy.logwarn_throttle(2.0, 'Ignoring joint state: %s', error)
            return
        with self.condition:
            self.sample = (time.monotonic(), positions)
            self.condition.notify_all()

    def wait(self, timeout, max_age, after=0.0):
        deadline = time.monotonic() + timeout
        with self.condition:
            while not self.rospy.is_shutdown():
                now = time.monotonic()
                if self.sample and self.sample[0] > after and now - self.sample[0] <= max_age:
                    return self.sample[0], dict(self.sample[1])
                remaining = deadline - now
                if remaining <= 0:
                    raise ValueError('No fresh complete seven-joint state; no motion was requested')
                self.condition.wait(min(remaining, 0.1))
        raise RuntimeError('ROS shutdown while waiting for joint states')


class StatusMonitor:
    def __init__(self):
        self.lock = threading.Lock()
        self.sample = None

    def callback(self, message):
        with self.lock:
            self.sample = (time.monotonic(), message)

    def require_ready(self, max_age):
        with self.lock:
            if not self.sample or time.monotonic() - self.sample[0] > max_age:
                raise ValueError('No fresh arm RobotStatus; execution refused')
            message = self.sample[1]
        fields = ('motion_enabled', 'axes_mastered', 'axes_gms_referenced',
                  'axes_position_referenced')
        bad = [name for name in fields if not getattr(message, name, False)]
        # Upstream LBRMsgGenerator sets this misleadingly named field to
        # safetyStopSignal != NOSTOP: True means a safety stop IS active.
        if getattr(message, 'safety_state_enabled', None) is not False:
            bad.append('safety stop active or unknown')
        if bad:
            raise ValueError('Arm RobotStatus is not ready: ' + ', '.join(bad))


def as_geometry(message):
    return Pose((message.position.x, message.position.y, message.position.z),
                (message.orientation.x, message.orientation.y,
                 message.orientation.z, message.orientation.w))


def as_ros_pose(pose, PoseMessage):
    message = PoseMessage()
    message.position.x, message.position.y, message.position.z = pose.position
    (message.orientation.x, message.orientation.y,
     message.orientation.z, message.orientation.w) = pose.orientation
    return message


def save_path(path, poses):
    with Path(path).open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(('index', 'x_m', 'y_m', 'z_m', 'qx', 'qy', 'qz', 'qw'))
        for index, pose in enumerate(poses):
            writer.writerow((index,) + tuple(pose.position) + tuple(pose.orientation))


def save_json(path, value):
    path = Path(path)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def run(rospy, moveit_commander):
    from geometry_msgs.msg import Pose as PoseMessage, PoseStamped
    from kmriiwa_msgs.msg import LBRStatus
    from moveit_msgs.msg import DisplayTrajectory, MoveItErrorCodes
    from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
    from nav_msgs.msg import Path as PathMessage
    from sensor_msgs.msg import JointState

    robot_name = rospy.get_param('~robot_name', 'kmriiwa')
    namespace = rospy.get_param('~robot_namespace', '/' + robot_name).rstrip('/')
    group_name = rospy.get_param('~move_group_name', robot_name + '_manipulator')
    description = rospy.get_param('~robot_description', namespace + '/robot_description')
    reference = rospy.get_param('~reference_frame', robot_name + '_link_0')
    ee_link = rospy.get_param('~ee_link', robot_name + '_link_ee')
    execute = rospy.get_param('~execute', False)
    keep_alive = rospy.get_param('~keep_alive', True)
    if not isinstance(execute, bool) or not isinstance(keep_alive, bool):
        raise ValueError('execute and keep_alive must be ROS boolean parameters')
    bed_file = rospy.get_param('~bed_preview_file', '')
    if not isinstance(bed_file, str):
        raise ValueError('bed_preview_file must be a path string')
    if bed_file and execute:
        raise ValueError('The bed preview uses an uncalibrated tool and bed height; execution is disabled')
    if bed_file and (reference != robot_name + '_link_0' or ee_link != robot_name + '_link_ee'):
        raise ValueError('The bed preview assumes World = link_0 and Flange = link_ee')
    bed = load_bed_preview(os.path.expanduser(bed_file)) if bed_file else None
    parameters = {key: rospy.get_param('~' + key, default) for key, default in (
        ('radius', .02), ('layers', 3), ('layer_step', .005), ('samples_per_circle', 60),
        ('plane', 'xy'), ('tcp_offset_xyz', [0., 0., 0.]), ('tcp_offset_rpy', [0., 0., 0.]))}
    controls = {key: positive_number(rospy.get_param('~' + key, default), key)
                for key, default in (
                    ('eef_step', .003), ('max_joint_step', .25), ('max_start_drift', .02),
                    ('max_state_age', 1.), ('state_timeout', 10.), ('server_timeout', 15.),
                    ('planning_velocity_scale', .1), ('planning_acceleration_scale', .1))}
    for key in ('planning_velocity_scale', 'planning_acceleration_scale'):
        if controls[key] > 1:
            raise ValueError(key + ' must be in (0, 1]')
    if bed is None:
        # Fail on bad geometry without contacting any MoveIt service.
        generate_circle_plan(Pose((0., 0., 0.), (0., 0., 0., 1.)), **parameters)
    names = [robot_name + '_joint_' + str(i) for i in range(1, 8)]
    root = Path(os.path.expanduser(rospy.get_param('~output_root', '~/.ros/kmriiwa_circle_demo')))
    root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    prefix = 'bed_circle_preview_' if bed is not None else 'air_circle_'
    directory = root / (prefix + stamp + '_' + str(os.getpid()))
    directory.mkdir()
    report = {'status': 'initializing', 'execute_requested': execute,
              'execution_attempted': False, 'robot_name': robot_name,
              'namespace': namespace, 'group': group_name, 'ee_link': ee_link,
              'reference_frame': reference, 'geometry': parameters, 'controls': controls,
              'ros_master_uri': os.environ.get('ROS_MASTER_URI', ''),
              'hardware_speed_controlled_by_demo': False,
              'tcp_is_identity_placeholder': not any(parameters['tcp_offset_xyz'] + parameters['tcp_offset_rpy'])}
    if bed is not None:
        report.update(geometry=bed.summary(), tcp_is_identity_placeholder=False,
                      frame_registration='provisional World = link_0 and Flange = link_ee',
                      includes_cartesian_approach_in_plan=True)
    save_json(directory / 'report.json', report)
    rospy.loginfo('Air-circle output: %s', directory)
    monitor = JointMonitor(rospy, names)
    status = StatusMonitor()
    subscriptions = [
        rospy.Subscriber(namespace + '/arm/joint_states', JointState, monitor.callback, queue_size=1),
        rospy.Subscriber(namespace + '/arm/state/RobotStatus', LBRStatus, status.callback, queue_size=1)]
    group = None
    try:
        monitor.wait(controls['state_timeout'], controls['max_state_age'])
        robot = moveit_commander.RobotCommander(robot_description=description, ns=namespace)
        group = moveit_commander.MoveGroupCommander(
            group_name, robot_description=description, ns=namespace,
            wait_for_servers=controls['server_timeout'])
        if set(group.get_active_joints()) != set(names):
            raise ValueError('MoveIt group is not the expected physical seven-joint arm')
        group.set_end_effector_link(ee_link)
        group.set_pose_reference_frame(reference)
        service = namespace + '/compute_fk'
        rospy.wait_for_service(service, timeout=controls['server_timeout'])
        # Commander/service initialization may take seconds; capture a NEW
        # sample now instead of relabelling the old sample with a new stamp.
        _, initial_positions = monitor.wait(
            controls['state_timeout'], controls['max_state_age'], after=time.monotonic())
        snapshot = copy.deepcopy(robot.get_current_state())
        positions = dict(zip(snapshot.joint_state.name, snapshot.joint_state.position))
        positions.update(initial_positions)
        snapshot.joint_state.name = list(positions)
        snapshot.joint_state.position = [positions[n] for n in snapshot.joint_state.name]
        snapshot.joint_state.velocity = []
        snapshot.joint_state.effort = []
        snapshot.joint_state.header.stamp = rospy.Time.now()
        group.set_start_state(snapshot)

        request = GetPositionFKRequest()
        request.header.frame_id = reference
        request.header.stamp = rospy.Time(0)
        request.fk_link_names = [ee_link]
        request.robot_state = snapshot
        response = rospy.ServiceProxy(service, GetPositionFK)(request)
        if response.error_code.val != MoveItErrorCodes.SUCCESS or len(response.pose_stamped) != 1:
            raise ValueError('Forward kinematics failed for the exact planning start state')
        if response.pose_stamped[0].header.frame_id.lstrip('/') != reference.lstrip('/'):
            raise ValueError('FK returned a different reference frame')
        initial_ee = as_geometry(response.pose_stamped[0].pose)
        if bed is None:
            circle = generate_circle_plan(initial_ee, **parameters)
            commanded_ee = circle.ee_waypoints
        else:
            circle = bed.path
            commanded_ee = (bed.start_ee,) + circle.ee_waypoints
            ee_from_tcp = compose_pose(inverse_pose(bed.start_ee), circle.start_tcp)
            report['current_tcp_world_m'] = compose_pose(initial_ee, ee_from_tcp).position
            rospy.logwarn('BED PREVIEW ONLY: provisional tool, height and frame registration. '
                          'The displayed joint plan also includes a Cartesian approach from the current pose.')
        if len(commanded_ee) >= 500:
            raise ValueError('Too many geometric targets for the 500-point Java backend; reduce layers/samples')
        save_path(directory / 'desired_tcp_path.csv', (circle.start_tcp,) + circle.tcp_waypoints)
        save_path(directory / 'desired_ee_path.csv', (initial_ee,) + commanded_ee)

        publishers = []
        for topic, poses in (('~tcp_path', (circle.start_tcp,) + circle.tcp_waypoints),
                             ('~ee_path', (initial_ee,) + commanded_ee)):
            publisher = rospy.Publisher(topic, PathMessage, queue_size=1, latch=True)
            message = PathMessage()
            message.header.frame_id, message.header.stamp = reference, rospy.Time.now()
            for pose in poses:
                stamped = PoseStamped()
                stamped.header = copy.deepcopy(message.header)
                stamped.pose = as_ros_pose(pose, PoseMessage)
                message.poses.append(stamped)
            publisher.publish(message)
            publishers.append(publisher)

        waypoints = [as_ros_pose(pose, PoseMessage) for pose in commanded_ee]
        trajectory, fraction = cartesian_path(group, waypoints, controls['eef_step'])
        # The old driver does not honor these times. Retime only keeps MoveIt
        # execution bookkeeping/display internally meaningful, never a speed cap.
        trajectory = group.retime_trajectory(
            snapshot, trajectory, velocity_scaling_factor=controls['planning_velocity_scale'],
            acceleration_scaling_factor=controls['planning_acceleration_scale'],
            algorithm='iterative_time_parameterization')
        checks = validate_trajectory(trajectory, fraction, initial_positions, controls['max_joint_step'])
        display = DisplayTrajectory()
        display.trajectory_start, display.trajectory = snapshot, [trajectory]
        display_publisher = rospy.Publisher('~display_planned_path', DisplayTrajectory, queue_size=1, latch=True)
        display_publisher.publish(display)
        publishers.append(display_publisher)
        save_json(directory / 'joint_trajectory.json', {
            'joint_names': list(trajectory.joint_trajectory.joint_names),
            'points': [{'positions_rad': list(p.positions), 'planned_time_s': p.time_from_start.to_sec()}
                       for p in trajectory.joint_trajectory.points]})
        report.update(checks)
        report.update(status='planned', start_positions_rad=initial_positions,
                      geometric_targets=len(waypoints))
        save_json(directory / 'report.json', report)
        rospy.loginfo('Complete plan: %d geometric targets, %d trajectory points. execute=%s',
                      len(waypoints), checks['trajectory_points'], execute)
        rospy.logwarn('Original Java backend uses relative JOINT speed and ignores planned timing. '
                      'The demo does not impose a physical TCP speed limit.')
        if report['tcp_is_identity_placeholder']:
            rospy.logwarn('TCP offset is zero: this is current link_ee air motion, not a calibrated nozzle path.')
        if execute:
            _, current = monitor.wait(controls['state_timeout'], controls['max_state_age'])
            require_start_match(initial_positions, current, controls['max_start_drift'])
            status.require_ready(controls['max_state_age'])
            report.update(status='executing', execution_attempted=True)
            save_json(directory / 'report.json', report)
            rospy.logwarn('Executing ONE full arm trajectory. Ctrl+C/action cancel is not a verified '
                          'physical stop for this Java driver; use the robot native stop if needed.')
            success = group.execute(trajectory, wait=True)
            if not success:
                raise RuntimeError('MoveIt execution returned failure; physical completion is unknown')
            # Require a new feedback message after the execution result.
            _, final_positions = monitor.wait(
                controls['state_timeout'], controls['max_state_age'], after=time.monotonic())
            target = dict(zip(names, trajectory.joint_trajectory.points[-1].positions))
            require_start_match(target, final_positions, controls['max_start_drift'])
            report.update(status='completed', final_positions_rad=final_positions)
            save_json(directory / 'report.json', report)
            rospy.loginfo('Air-circle execution completed; final arm feedback matches the endpoint.')
        else:
            rospy.loginfo('PLAN ONLY: no execution or stop request sent. Add Path /kmriiwa_circle_demo/tcp_path '
                          'or MotionPlanning display topic /kmriiwa_circle_demo/display_planned_path in RViz.')
        if keep_alive:
            rospy.loginfo('Keeping preview topics alive. Ctrl+C exits the demo.')
            rospy.spin()
        return 0
    except BaseException as error:
        if not report['execution_attempted'] and report['status'] == 'planned' and rospy.is_shutdown():
            return 0
        if report['status'] == 'completed':
            # Closing the RViz preview must not rewrite a verified completion
            # into an unknown run or send another controller stop request.
            if rospy.is_shutdown() or isinstance(error, KeyboardInterrupt):
                return 0
            report['preview_error_after_completion'] = str(error) or type(error).__name__
            save_json(directory / 'report.json', report)
            raise
        report.update(status='execution_unknown' if report['execution_attempted'] else 'rejected',
                      error=str(error) or type(error).__name__)
        save_json(directory / 'report.json', report)
        if report['execution_attempted'] and group is not None:
            # Best effort only. The unchanged original action backend does not
            # implement a physical motion cancel; never claim this stopped it.
            try:
                group.stop()
            except Exception:
                pass
            rospy.logerr('Physical stop is NOT confirmed. Use the robot native stop; do not auto-retry.')
        raise
    finally:
        for subscription in subscriptions:
            subscription.unregister()


def main():
    try:
        import rospy
        import moveit_commander
    except ImportError as error:
        print('ROS Noetic/MoveIt Python environment is required: %s' % error, file=sys.stderr)
        return 2
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('kmriiwa_circle_demo')
    try:
        return run(rospy, moveit_commander)
    except (Exception, KeyboardInterrupt) as error:
        rospy.logerr('Air-circle demo: %s', str(error) or type(error).__name__)
        return 1
    finally:
        moveit_commander.roscpp_shutdown()

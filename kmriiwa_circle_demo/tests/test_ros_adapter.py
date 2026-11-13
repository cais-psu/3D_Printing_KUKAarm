"""Exercise the ROS adapter's real control flow without ROS or hardware.

Messages and MoveIt are deliberately small fakes; these checks verify whether
the adapter sends an execution/stop request and records an honest outcome.
"""

import copy
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from kmriiwa_circle_demo import ros_node


NS = types.SimpleNamespace
NAMES = ["kmriiwa_joint_" + str(index) for index in range(1, 8)]


class Time:
    def __init__(self, seconds=0.0):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds

    @classmethod
    def now(cls):
        return cls(123.0)


def header():
    return NS(frame_id="", stamp=Time())


class PoseMessage:
    def __init__(self):
        self.position = NS(x=0.4, y=0.0, z=0.3)
        self.orientation = NS(x=0.0, y=0.0, z=0.0, w=1.0)


class PoseStamped:
    def __init__(self):
        self.header = header()
        self.pose = PoseMessage()


class PathMessage:
    def __init__(self):
        self.header = header()
        self.poses = []


class FKRequest:
    def __init__(self):
        self.header = header()
        self.fk_link_names = []
        self.robot_state = None


class DisplayTrajectory:
    pass


class Status:
    motion_enabled = True
    axes_mastered = True
    axes_gms_referenced = True
    axes_position_referenced = True
    # The upstream name is misleading: True means a safety stop is active.
    safety_state_enabled = False


def positions(value=0.0):
    return dict.fromkeys(NAMES, value)


def trajectory(values=(0.01, 0.02), names=None):
    points = [NS(positions=[value] * 7, time_from_start=Time(index + 1.0))
              for index, value in enumerate(values)]
    return NS(joint_trajectory=NS(joint_names=list(names or NAMES), points=points),
              multi_dof_joint_trajectory=NS(joint_names=[], points=[]))


def message_modules():
    modules = {}
    specifications = {
        "geometry_msgs.msg": {"Pose": PoseMessage, "PoseStamped": PoseStamped},
        "kmriiwa_msgs.msg": {"LBRStatus": Status},
        "moveit_msgs.msg": {"DisplayTrajectory": DisplayTrajectory,
                            "MoveItErrorCodes": NS(SUCCESS=1)},
        "moveit_msgs.srv": {"GetPositionFK": object, "GetPositionFKRequest": FKRequest},
        "nav_msgs.msg": {"Path": PathMessage},
        "sensor_msgs.msg": {"JointState": object},
    }
    for fullname, members in specifications.items():
        parent_name, child_name = fullname.split(".")
        parent = modules.setdefault(parent_name, types.ModuleType(parent_name))
        child = types.ModuleType(fullname)
        child.__dict__.update(members)
        setattr(parent, child_name, child)
        modules[fullname] = child
    return modules


class FakeROS:
    Time = Time

    def __init__(self, output_root, parameters=None, status_message=None):
        self.parameters = {"~output_root": output_root, "~keep_alive": False}
        self.parameters.update(parameters or {})
        self.events = []
        self.messages = {}
        self.fk_request = None
        self.status_message = Status() if status_message is None else status_message

    def get_param(self, name, default=None):
        return self.parameters.get(name, default)

    def Subscriber(self, topic, message_type, callback, **options):
        self.events.append(("subscribe", topic))
        if topic.endswith("/RobotStatus"):
            callback(self.status_message)
        return NS(unregister=lambda: self.events.append(("unsubscribe", topic)))

    def Publisher(self, topic, message_type, **options):
        def publish(message):
            self.messages[topic] = message
        return NS(publish=publish)

    def wait_for_service(self, name, **options):
        self.events.append(("wait_for_service", name))

    def ServiceProxy(self, name, service_type):
        def fk(request):
            self.fk_request = copy.deepcopy(request)
            stamped = PoseStamped()
            stamped.header.frame_id = request.header.frame_id
            return NS(error_code=NS(val=1), pose_stamped=[stamped])
        return fk

    def spin(self):
        self.events.append(("spin",))

    def is_shutdown(self):
        return False

    def loginfo(self, *args):
        self.events.append(("info", args))

    def logwarn(self, *args):
        self.events.append(("warn", args))

    def logerr(self, *args):
        self.events.append(("error", args))

    def logwarn_throttle(self, *args):
        self.events.append(("warn_throttle", args))


class FakeGroup:
    def __init__(self, plan=None, fraction=1.0, succeeds=True):
        self.plan = plan or trajectory()
        self.fraction = fraction
        self.succeeds = succeeds
        self.execute_calls = []
        self.stop_calls = 0
        self.start_state = None
        self.path_arguments = None

    def get_active_joints(self):
        return list(NAMES)

    def set_end_effector_link(self, link):
        self.ee_link = link

    def set_pose_reference_frame(self, frame):
        self.reference_frame = frame

    def set_start_state(self, snapshot):
        self.start_state = copy.deepcopy(snapshot)

    def compute_cartesian_path(self, waypoints, eef_step, avoid_collisions=True,
                               path_constraints=None):
        self.path_arguments = (waypoints, eef_step, avoid_collisions, path_constraints)
        return self.plan, self.fraction

    def retime_trajectory(self, snapshot, plan, **options):
        self.retime_options = options
        return plan

    def execute(self, plan, wait=True):
        self.execute_calls.append((plan, wait))
        return self.succeeds

    def stop(self):
        self.stop_calls += 1


class FakeMoveIt:
    def __init__(self, group):
        self.group = group
        self.robot_arguments = None
        self.group_arguments = None

    def RobotCommander(self, **options):
        self.robot_arguments = options
        # The arm values must be replaced by the exact observed joint sample.
        state = NS(joint_state=NS(name=list(NAMES), position=[0.15] * 7,
                                 velocity=[], effort=[], header=header()))
        return NS(get_current_state=lambda: copy.deepcopy(state))

    def MoveGroupCommander(self, name, **options):
        self.group_arguments = (name, options)
        return self.group


class CartesianCompatibilityTests(unittest.TestCase):
    def test_noetic_third_argument_is_collision_check_not_jump_threshold(self):
        group = FakeGroup()
        result = ros_node.cartesian_path(group, [PoseMessage()], 0.003)
        self.assertIs(result[0], group.plan)
        self.assertIs(group.path_arguments[2], True)
        self.assertIsNone(group.path_arguments[3])

    def test_old_signature_receives_both_settings_by_keyword(self):
        class OldGroup:
            def compute_cartesian_path(self, waypoints, eef_step, jump_threshold,
                                       avoid_collisions=True):
                return jump_threshold, avoid_collisions

        jump_threshold, avoid_collisions = ros_node.cartesian_path(OldGroup(), [], 0.003)
        self.assertGreater(jump_threshold, 0.0)
        self.assertIs(avoid_collisions, True)


class AdapterFlowTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.module_patch = mock.patch.dict(sys.modules, message_modules())
        self.module_patch.start()
        self.addCleanup(self.module_patch.stop)

    def run_adapter(self, *, parameters=None, group=None, feedback=None,
                    status_message=None, spin_error=None):
        self.ros = FakeROS(self.temporary.name, parameters, status_message)
        if spin_error is not None:
            self.ros.spin = mock.Mock(side_effect=spin_error)
        self.group = group or FakeGroup()
        self.moveit = FakeMoveIt(self.group)
        # The bootstrap observation precedes server initialization. The exact
        # start must come from the new post-initialization observation instead.
        samples = [(0.0, positions(0.12))] + (feedback or [
            (1.0, positions()), (2.0, positions()), (3.0, positions(0.02))])
        with mock.patch.object(ros_node.JointMonitor, "wait", side_effect=samples):
            return ros_node.run(self.ros, self.moveit)

    def report(self):
        reports = list(Path(self.temporary.name).glob("air_circle_*/report.json"))
        self.assertEqual(len(reports), 1)
        return json.loads(reports[0].read_text())

    def assert_no_motion_request(self):
        self.assertEqual(self.group.execute_calls, [])
        self.assertEqual(self.group.stop_calls, 0)

    def test_default_plan_only_publishes_exact_start_and_never_executes_or_stops(self):
        self.assertEqual(self.run_adapter(), 0)
        self.assert_no_motion_request()
        report = self.report()
        self.assertEqual(report["status"], "planned")
        self.assertFalse(report["execute_requested"])
        self.assertFalse(report["execution_attempted"])
        self.assertIs(self.group.path_arguments[2], True)
        self.assertEqual(self.moveit.robot_arguments,
                         {"robot_description": "/kmriiwa/robot_description", "ns": "/kmriiwa"})
        self.assertEqual(self.moveit.group_arguments[0], "kmriiwa_manipulator")
        display = self.ros.messages["~display_planned_path"]
        self.assertEqual(display.trajectory_start.joint_state.position, [0.0] * 7)
        self.assertEqual(self.ros.fk_request.robot_state.joint_state.position, [0.0] * 7)
        self.assertEqual(self.group.start_state.joint_state.position, [0.0] * 7)

    def test_incomplete_path_is_rejected_before_execute_or_stop(self):
        with self.assertRaisesRegex(ValueError, "incomplete"):
            self.run_adapter(parameters={"~execute": True}, group=FakeGroup(fraction=0.99))
        self.assert_no_motion_request()
        self.assertEqual(self.report()["status"], "rejected")

    def test_actual_interpolated_point_overflow_is_rejected(self):
        plan = trajectory([index * 0.0001 for index in range(501)])
        with self.assertRaisesRegex(ValueError, "501 points"):
            self.run_adapter(parameters={"~execute": True}, group=FakeGroup(plan=plan))
        self.assert_no_motion_request()

    def test_joint_branch_jump_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "Joint jump"):
            self.run_adapter(parameters={"~execute": True}, group=FakeGroup(plan=trajectory((0.8,))))
        self.assert_no_motion_request()

    def test_wrong_joint_order_is_rejected_for_index_based_java_backend(self):
        with self.assertRaisesRegex(ValueError, "joint order"):
            self.run_adapter(parameters={"~execute": True},
                             group=FakeGroup(plan=trajectory(names=list(reversed(NAMES)))))
        self.assert_no_motion_request()

    def test_start_drift_rejects_the_original_plan_without_auto_replanning(self):
        with self.assertRaisesRegex(ValueError, "Arm moved since planning"):
            self.run_adapter(parameters={"~execute": True},
                             feedback=[(1.0, positions()), (2.0, positions(0.10))])
        self.assert_no_motion_request()
        self.assertEqual(self.report()["status"], "rejected")

    def test_execute_failure_cannot_be_reported_as_completed(self):
        with self.assertRaisesRegex(RuntimeError, "execution returned failure"):
            self.run_adapter(parameters={"~execute": True}, group=FakeGroup(succeeds=False))
        self.assertEqual(len(self.group.execute_calls), 1)
        self.assertEqual(self.group.stop_calls, 1)
        self.assertEqual(self.report()["status"], "execution_unknown")
        self.assertTrue(self.report()["execution_attempted"])
        self.assertNotIn("final_positions_rad", self.report())

    def test_explicit_execute_requires_matching_post_result_feedback(self):
        self.assertEqual(self.run_adapter(parameters={"~execute": True}), 0)
        self.assertEqual(len(self.group.execute_calls), 1)
        self.assertEqual(self.group.stop_calls, 0)
        self.assertEqual(self.report()["status"], "completed")
        self.assertEqual(self.report()["final_positions_rad"], positions(0.02))

    def test_success_action_without_matching_feedback_remains_unknown(self):
        with self.assertRaisesRegex(ValueError, "Arm moved since planning"):
            self.run_adapter(parameters={"~execute": True},
                             feedback=[(1.0, positions()), (2.0, positions()),
                                       (3.0, positions(0.2))])
        self.assertEqual(len(self.group.execute_calls), 1)
        self.assertEqual(self.report()["status"], "execution_unknown")

    def test_string_execute_cannot_accidentally_enable_motion(self):
        with self.assertRaisesRegex(ValueError, "boolean"):
            self.run_adapter(parameters={"~execute": "false"})
        self.assert_no_motion_request()

    def test_uncalibrated_bed_preview_cannot_execute(self):
        with self.assertRaisesRegex(ValueError, "execution is disabled"):
            self.run_adapter(parameters={"~bed_preview_file": "/unused/bed.json", "~execute": True})
        self.assert_no_motion_request()
        self.assertIsNone(self.moveit.group_arguments)

    def test_bed_preview_includes_start_target_and_only_plans(self):
        config = Path(__file__).resolve().parents[1] / 'config/bed_preview.json'
        self.assertEqual(self.run_adapter(parameters={"~bed_preview_file": str(config)}), 0)
        tcp_path = self.ros.messages['~tcp_path'].poses
        ee_path = self.ros.messages['~ee_path'].poses
        commanded, _, avoid_collisions, _ = self.group.path_arguments
        self.assertEqual(len(tcp_path), 183)
        self.assertEqual(len(ee_path), 184)
        self.assertEqual(len(commanded), 183)
        self.assertAlmostEqual(commanded[0].position.x, ee_path[1].pose.position.x)
        self.assertAlmostEqual(tcp_path[0].pose.position.z, .05191, places=5)
        self.assertIs(avoid_collisions, True)
        self.assert_no_motion_request()

    def test_active_safety_stop_rejects_execution(self):
        stopped = Status()
        stopped.safety_state_enabled = True
        with self.assertRaisesRegex(ValueError, "safety stop active or unknown"):
            self.run_adapter(parameters={"~execute": True}, status_message=stopped)
        self.assert_no_motion_request()
        self.assertEqual(self.report()["status"], "rejected")

    def test_unknown_safety_stop_rejects_execution(self):
        unknown = NS(motion_enabled=True, axes_mastered=True,
                     axes_gms_referenced=True, axes_position_referenced=True)
        with self.assertRaisesRegex(ValueError, "safety stop active or unknown"):
            self.run_adapter(parameters={"~execute": True}, status_message=unknown)
        self.assert_no_motion_request()

    def test_verified_completion_survives_closing_preview(self):
        self.assertEqual(self.run_adapter(parameters={"~execute": True, "~keep_alive": True},
                                          spin_error=KeyboardInterrupt()), 0)
        self.assertEqual(len(self.group.execute_calls), 1)
        self.assertEqual(self.group.stop_calls, 0)
        self.assertEqual(self.report()["status"], "completed")


if __name__ == "__main__":
    unittest.main()

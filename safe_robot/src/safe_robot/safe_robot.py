import rospy
import numpy as np
import pyexotica as exo
from visualization_msgs.msg import Marker
from std_srvs.srv import SetBool, SetBoolResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from diagnostic_msgs.msg import DiagnosticStatus

class RobotSafetyChecker:

    def __init__(self, exotica_xml_filename, link_xlim, link_ylim, link_zlim, safe_links, joint_position_limit_factor, safe_distance):

        # Set variables
        self.q_targ = None
        self.joint_position_limit_indicators = None
        self.safe_links = None
        self.flag = 0

        # Set box limits
        self.link_xlim = link_xlim
        self.link_ylim = link_ylim
        self.link_zlim = link_zlim
        self.safe_links = safe_links

        # Setup exotica
        solver = exo.Setup.load_solver(exotica_xml_filename)
        self.scene = solver.get_problem().get_scene()
        self.kinematic_tree = self.scene.get_kinematic_tree()
        self.base_link_id = self.scene.get_model_link_names()[0]
        self.joint_names = self.scene.get_controlled_joint_names()
        self.ndof = len(self.joint_names)
        self.safe_distance = safe_distance

        # Extract joint position/velocity limits from kinematic tree
        joint_position_limits = joint_position_limit_factor*self.kinematic_tree.get_joint_limits()
        self.joint_position_limits_lower = joint_position_limits[:,0]
        self.joint_position_limits_upper = joint_position_limits[:,1]

    # Private methods

    def _in_joint_position_limits(self):
        self.joint_position_limit_indicators = np.logical_and(self.joint_position_limits_lower <= self.q_targ, self.q_targ <= self.joint_position_limits_upper)
        return self.joint_position_limit_indicators.all()

    def _links_in_limits(self):
        safe_links = {}
        for link_name in self.safe_links:
            link_targ_position = self.scene.fk(link_name).get_translation()
            safe_links[link_name] = {
                'x': self.link_xlim[0] <= link_targ_position[0] <= self.link_xlim[1],
                'y': self.link_ylim[0] <= link_targ_position[1] <= self.link_ylim[1],
                'z': self.link_zlim[0] <= link_targ_position[2] <= self.link_zlim[1],
            }
        self.safe_links = safe_links
        return all(all(link.values()) for link in self.safe_links.values())

    def _is_state_valid(self):
        return self.scene.is_state_valid(safe_distance=self.safe_distance)

    # Public methods

    def set_target_joint_state(self, msg):
        """Set the target joint velocity position."""

        q_targ = np.zeros(self.ndof)
        for idx, name in enumerate(self.joint_names):
            q_targ[idx] = msg.position[msg.name.index(name)]

        self.q_targ = q_targ
        self.scene.update(self.q_targ) # update exotica

    def get_box_limit_marker(self):
        """Returns a ROS marker for visualizing the box limit."""
        marker = Marker()
        marker.header.frame_id = 'safe_robot/' + self.base_link_id
        marker.ns = 'safe_robot'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.link_xlim[1] - self.link_xlim[0]
        marker.scale.y = self.link_ylim[1] - self.link_ylim[0]
        marker.scale.z = self.link_zlim[1] - self.link_zlim[0]
        marker.color.a = 0.5
        marker.color.r = 1.0
        return marker

    def is_safe(self):
        """Returns false when safety checks are violated, true otherwise."""

        # Setup
        is_safe = True
        self.flag = 0  # zero indicates no issues

        # Check position limits
        if not self._in_joint_position_limits():
            is_safe = False
            self.flag = -1
            return is_safe

        # Check links are within limits
        if not self._links_in_limits():
            is_safe = False
            self.flag = -2
            return is_safe
        # Check self-collision
        if self.check_self_collision:
            if not self._is_state_valid():
                is_safe = False
                self.flag = -3
                return is_safe

        return is_safe

    def report(self):
        """Returns an indicator flag and human-readable information."""

        if self.flag == 0:
            info = 'safety checks passed'

        elif self.flag == -1:
            info = f'target violates joint position limits, joints inside position limit:\n'
            for name, indicator in zip(self.joint_names, self.joint_position_limit_indicators):
                info += f'{name}: {indicator}\n'

        elif self.flag == -2:
            info = f'target joint position means some safety links fall outside box limit:\n'
            for link_name, link in self.safe_links.items():
                info += f'{link_name}:\n'
                for dim in ('x', 'y', 'z'):
                    info += f'  %s: %s\n' % (dim, str(link[dim]))
        elif self.flag == -3:
            info = 'robot in self collision'
        else:
            raise AttributeError(f"did not recognize flag {self.flag}!")

        return self.flag, info


class SafetyNode:


    def __init__(self):

        # Setup node
        rospy.init_node('robot_safety_node')
        self.node_name = rospy.get_name()

        # Set variables
        self.main_loop_timer = None
        self.main_loop_running = False

        # Get (some) parameters, others collected after RobotSafetyChecker is initialized
        robot_safety_checker_setup = dict(
            exotica_xml_filename=rospy.get_param('~exotica_xml_filename'),
            link_xlim=[float(n) for n in rospy.get_param('~link_xlim').split(' ')],
            link_ylim=[float(n) for n in rospy.get_param('~link_ylim').split(' ')],
            link_zlim=[float(n) for n in rospy.get_param('~link_zlim').split(' ')],
            safe_links=rospy.get_param('~safe_links').split(' '),
            joint_position_limit_factor=rospy.get_param('~joint_position_limit_factor', 0.95),
            safe_distance=rospy.get_param('~safe_distance', 0.0),
        )

        publish_as_joint_state = rospy.get_param('~publish_as_joint_state', True)

        # Setup publishers
        self.diag_pub = rospy.Publisher('robot_safety/%s/status' % self.node_name[1:], DiagnosticStatus, queue_size=10)
        cmd_topic = 'joint_states/target/command'
        if publish_as_joint_state:
            self.publish = self.publish_joint_state
            self.pub = rospy.Publisher(cmd_topic, JointState, queue_size=10)
        else:
            self.publish = self.publish_float_array
            self.pub = rospy.Publisher(cmd_topic, Float64MultiArray, queue_size=10)

        # Setup robot safety checker
        self.robot_safety_checker = RobotSafetyChecker(**robot_safety_checker_setup)

        # Setup debugging
        self.debug = rospy.get_param('~debug', False)
        if self.debug:
            debug_hz = 20
            self.box_limit_marker = self.robot_safety_checker.get_box_limit_marker()
            self.box_limit_marker_pub = rospy.Publisher('safe_robot/box_limit/marker', Marker, queue_size=10)
            rospy.Timer(rospy.Duration(1.0/float(debug_hz)), self.debug_loop)
        else:
            self.box_limit_marker = None

        # Set joint name order
        self.joint_name_order = rospy.get_param('~joint_name_order', self.robot_safety_checker.joint_names)

        # Start subscribers
        rospy.Subscriber('joint_states/target', JointState, self.callback)

        # Report successful initialization
        msg = f"[{self.node_name}] successfully initialized node, expecting joint states with joint names:\n"
        for name in self.joint_name_order:
            msg += name + '\n'
        rospy.loginfo(msg)
        if self.debug:
            rospy.loginfo("[%s] started node in debug mode", self.node_name)

    def resolve_joint_order(self, q):
        qout = np.zeros(self.robot_safety_checker.ndof)
        for idx, joint_name in enumerate(self.joint_name_order):
            qout[idx] = q[self.robot_safety_checker.joint_names.index(joint_name)]
        return qout

    def publish_float_array(self, q_target):
        self.pub.publish(Float64MultiArray(data=q_target))

    def publish_joint_state(self, q_target):
        msg = JointState(name=self.joint_name_order, position=q_target)
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)

    def publish_report(self):
        flag, info = self.robot_safety_checker.report()
        if self.debug:
            rospy.loginfo('report:\nflag=%d\ninfo:\n%s', flag, info)
        if flag == 0:
            level = DiagnosticStatus.OK
        elif flag > 0:
            level = DiagnosticStatus.WARN
        else:
            level = DiagnosticStatus.ERROR
        self.diag_pub.publish(DiagnosticStatus(level=level, name=str(flag), message=info, hardware_id=self.node_name))

    def callback(self, msg):
        self.robot_safety_checker.set_target_joint_state(msg)
        if self.robot_safety_checker.is_safe():
            if self.robot_safety_checker.flag == 0:
                q_target = self.resolve_joint_order(self.robot_safety_checker.q_targ)
                self.publish(q_target)
        self.publish_report()

    def debug_loop(self, event):
        self.box_limit_marker.header.stamp = rospy.Time.now()
        self.box_limit_marker_pub.publish(self.box_limit_marker)

    def spin(self):
        rospy.spin()

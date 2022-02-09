import rospy
import numpy as np
import pyexotica as exo
from visualization_msgs.msg import Marker
from std_srvs.srv import SetBool, SetBoolResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from diagnostic_msgs.msg import DiagnosticStatus

class RobotSafetyChecker:

    def __init__(self, exotica_xml_filename, link_xlim, link_ylim, link_zlim, safe_links, joint_position_limit_factor, joint_velocity_limit_factor, K, D):

        # Set variables
        self.q_curr = None
        self.qd_curr = None
        self.q_targ = None
        self.qd_targ = None
        self.joint_position_limit_indicators = None
        self.joint_velocity_limit_indicators = None
        self.safe_links = None
        self.flag = 0

        # Set control loop dt and gains
        self.K = K
        self.D = D

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

        # Extract joint position/velocity limits from kinematic tree
        joint_position_limits = joint_position_limit_factor*self.kinematic_tree.get_joint_limits()
        self.joint_position_limits_lower = joint_position_limits[:,0]
        self.joint_position_limits_upper = joint_position_limits[:,1]
        self.joint_velocity_limits_upper = joint_velocity_limit_factor*self.kinematic_tree.get_velocity_limits()
        self.joint_velocity_limits_lower = -self.joint_velocity_limits_upper.copy()

    # Private methods

    def _is_ready(self):
        return (self.q_curr is not None) and (self.qd_curr is not None) and (self.q_targ is not None) and (self.qd_targ is not None)

    def _in_joint_position_limits(self):
        self.joint_position_limit_indicators = np.logical_and(self.joint_position_limits_lower <= self.q_targ, self.q_targ <= self.joint_position_limits_upper)
        return self.joint_position_limit_indicators.all()

    def _in_joint_velocity_limits(self):
        self.joint_velocity_limit_indicators = np.logical_and(self.joint_velocity_limits_lower <= self.qd_targ, self.qd_targ <= self.joint_velocity_limits_upper)
        return self.joint_velocity_limit_indicators.all()

    def _links_in_limits(self):
        safe_links = {}
        for link_name in self.safe_links:
            link_targ_position = scene.fk(link_name).get_translation()
            safe_links[link_name] = {
                'x': self.link_xlim[0] <= link_targ_position[0] <= self.link_xlim[1],
                'y': self.link_ylim[0] <= link_targ_position[1] <= self.link_ylim[1],
                'z': self.link_zlim[0] <= link_targ_position[2] <= self.link_zlim[1],
            }
        self.safe_links = safe_links
        return all(all(link.values()) for link in self.safe_links.values())

    # Public methods

    def set_current_joint_state(self, qc, qdc):
        """Set the current joint position/velocity state."""
        self.q_curr = qc
        self.qd_curr = qdc

    def set_target_joint_state(self, qt):
        """Set the target joint velocity position."""
        self.q_targ = qt
        self.qd_targ = self.K*(self.q_targ - self.q_curr) - D*self.qd_curr
        self.scene.update(self.q_targ) # update exotica

    def box_limit_marker(self):
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

        # Is ready?
        if not self._is_ready():
            self.flag = 1
            return is_safe

        # Check position limits
        if not self._in_joint_position_limits():
            is_safe = False
            self.flag = -1
            return is_safe

        # Check velocity limits
        if not self._in_joint_velocity_limits():
            is_safe = False
            self.flag = -2
            return is_safe

        # Check links are within limits
        if not self._links_in_limits():
            is_safe = False
            self.flag = -3
            return is_safe

        return is_safe

    def report(self):
        """Returns an indicator flag and human-readable information."""

        if self.flag == 0:
            info = 'safety checks passed'

        elif self.flag == 1:
            info = 'current/target joint states not yet set'

        elif self.flag == -1:
            info = f'target violates joint position limits, joints inside position limit:\n'
            for name, indicator in zip(self.joint_names, self.joint_position_limit_indicators):
                info += f'{name}: {indicator}\n'

        elif self.flag == -2:
            info = f'target violates joint velocity limits, joints inside velocity limit:\n'
            for name, indicator in zip(self.joint_names, self.joint_velocity_limit_indicators):
                info += f'{name}: {indicator}\n'

        elif self.flag == -3:
            info = f'target joint position means some safety links fall outside box limit:\n'
            for link_name, link in self.safe_links.items():
                info += f'{link_name}:\n'
                for dim in ('x', 'y', 'z'):
                    info += f'  %s: %s\n' % (dim, str(link[dim]))
        return self.flag, info

    def get_next_joint_position_state(self, dt):
        return self.q_curr + dt*self.qd_targ

    def get_next_joint_velocity_state(self):
        return self.qd_targ.copy()


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
            joint_velocity_limit_factor=rospy.get_param('~joint_velocity_limit_factor', 0.95),
            K=rospy.get_param('~K', 1.0),
            D=rospy.get_param('~D', 0.1),
        )

        publish_as_joint_state = rospy.get_param('~publish_as_joint_state', True)
        self.hz = rospy.get_param('~hz', 100)
        self.dt = 1.0/float(self.hz)
        self.dur = rospy.Duration(self.dt)

        # Setup publishers
        self.diag_pub = rospy.Publisher('robot_safety/%s/status' % self.node_name, DiagnosticStatus, queue_size=10)
        if publish_as_joint_state:
            self.publish = self.publish_joint_state
            self.pub = rospy.Publisher('joint_state/target/command', JointState, queue_size=10)
        else:
            self.publish = self.publish_float_array
            self.pub = rospy.Publisher('joint_state/target/command', Float64MultiArray, queue_size=10)

        # Setup robot safety checker
        self.robot_safety_checker = RobotSafetyChecker(**robot_safety_checker_setup)

        # Set joint name order
        self.joint_name_order = rospy.get_param('~joint_name_order', self.robot_safety_checker.joint_names)

        # Start subscribers
        rospy.Subscriber('joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('joint_state/target', JointState, self.target_joint_state_callback)

        # Setup trigger service
        rospy.Service('toggle_robot_safety_remapper', SetBool, self.service_toggle_robot_safety_remapper)

    def joint_state_callback(self, msg):
        q = np.zeros(self.robot_safety_checker.ndof)
        qd = np.zeros(self.robot_safety_checker.ndof)
        for idx, joint_name in enumerate(self.robot_safety_checker.joint_names):
            msg_joint_index = msg.name.index(joint_name)
            q[idx] = msg.position[msg_joint_index]
            qd[idx] = msg.velocity[msg_joint_index]
        self.robot_safety_checker.set_current_joint_state(q, qd)

    def target_joint_state_callback(self, msg):
        q = np.zeros(self.robot_safety_checker.ndof)
        self.robot_safety_checker.set_target_joint_state(q)

    def service_toggle_robot_safety_remapper(self, req):
        if req.data:
            success, message = self.turn_on_main_loop()
        else:
            success, message = self.turn_off_main_loop()
        return SetBoolResponse(success=success, message=message)

    def turn_on_main_loop(self):
        success = True
        message = 'turned on main loop'
        if not self.main_loop_running:
            self.main_loop_timer = rospy.Timer(self.dur, self.main_loop)
            self.main_loop_running = True
        else:
            success = False
            message = f"[{self.node_name}] recieved request to turn on main loop, but it is already running! Doing nothing."
            rospy.logerr(message)
        return success, message

    def turn_off_main_loop(self):
        success = True
        message = 'turned off main loop'
        if self.main_loop_running:
            self.main_loop_timer.shutdown()
            self.main_loop_timer = None
            self.main_loop_running = False
        else:
            success = False
            message = f"[{self.node_name}] recieved request to turn off main loop, but it is not running! Doing nothing."
            rospy.logerr(message)
        return success, message

    def resolve_joint_order(self, q):
        qout = np.zeros(self.robot_safety_checker.ndof)
        for idx, joint_name in enumerate(self.joint_name_order):
            qout[idx] = q[self.robot_safety_checker.joint_name.index(joint_name)]
        return qout

    def publish_float_array(self, q_target, qd_target):
        self.pub.publish(Float64MultiArray(data=q_target))

    def publish_joint_state(self, q_target, qd_target):
        msg = JointState(name=self.joint_name_order, position=q_target, velocity=qd_target)
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)

    def publish_report(self):
        flag, info = self.robot_safety_checker.report()
        if flag == 0:
            level = DiagnosticStatus.OK
        elif flag > 0:
            level = DiagnosticStatus.WARN
        else:
            level = DiagnosticStatus.ERROR
        self.diag_pub.publish(DiagnosticStatus(level=level, name=str(flag), message=info, hardware_id=self.name))

    def main_loop(self, event):
        if self.robot_safety_checker.is_safe():
            if self.robot_safety_checker.flag == 0:
                q_target = self.resolve_joint_order(self.robot_safety_checker.get_next_joint_position_state())
                qd_target = self.resolve_joint_order(self.robot_safety_checker.get_next_joint_velocity_state())
                self.publish(q_target, qd_target)
        self.publish_report()

    def spin(self):
        rospy.spin()

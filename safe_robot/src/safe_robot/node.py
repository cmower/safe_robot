import rospy
import numpy as np
from sensor_msgs.msg import JointState
from safe_robot.msg import Status
from std_srvs.srv import Trigger, TriggerResponse

from .query import Query
from .checker import JointPositionLimit, JointVelocityLimit, LinkBoxLimit, StateValidity

class Node:

    def __init__(self):

        # Class variables
        self.seq = None

        # Initialize ROS node
        rospy.init_node('robot_safety_node')

        # Setup Query
        self.query = Query()

        # Setup checkers
        self.checkers = [
            JointPositionLimit(self.query),
            JointVelocityLimit(self.query),
            LinkBoxLimit(self.query),
            StateValidity(self.query)
        ]

        # Create the safe status message
        self.safe_status_msg = Status()
        self.safe_status_msg.message = 'target joint state is safe'
        self.safe_status_msg.is_safe = True

        # Create publishers
        self.status_pub = rospy.Publisher('robot_safety/status', Status, queue_size=10)
        self.joint_pub = rospy.Publisher('joint_states/command', JointState, queue_size=10)

        # Setup services
        self.sub = None
        rospy.Service('robot_safety/start_subscriber', Trigger, self.start_target_joint_state_subscriber)
        rospy.Service('robot_safety/stop_subscriber', Trigger, self.stop_target_joint_state_subscriber)

        # Start target joint state subscriber
        self.start_target_joint_state_subscriber()


    def start_target_joint_state_subscriber(self):
        self.seq = 0
        if not self.sub:
            self.sub = rospy.Subscriber('joint_states/target', JointState, self.callback)
            success = True
            message = 'started target joint state subscriber'
            rospy.loginfo(message)
        else:
            success = False
            message = 'tried to start target joint state subscriber, but it is already running!'
            rospy.logerr(message)
        return TriggerResponse(message=message, success=success)


    def stop_target_joint_state_subscriber(self):
        if self.sub:
            self.sub.unregister()
            self.sub = None
            success = True
            message = 'stopped target joint state subscriber'
            rospy.loginfo(message)
        else:
            success = False
            message = 'tried to start target joint state subscriber, but it is already running!'
            rospy.logerr(message)
        return TriggerResponse(message=message, success=success)

    def callback(self, msg):

        # Reset query and loop over safety checkers
        self.query.set_target(rospy.Time.now().to_sec(), msg)
        is_safe = [c.is_safe() for c in self.checkers]

        # Check if target is safe
        if all(is_safe):

            # Publish command joint state
            msgcmd = JointState(name=msg.name, position=msg.position, velocity=msg.velocity, effort=msg.effort)
            msgcmd.header.stamp = rospy.Time.now()
            msgcmd.header.seq = self.seq
            self.joint_pub.publish(msgcmd)

            # Publish safety check
            self.safe_status_msg.header.stamp = rospy.Time.now()
            self.safe_status_msg.header.seq = self.seq
            self.status_pub.publish(self.safe_status_msg)

            # Increment seq
            self.seq += 1

        else:

            # Publish unsafe status message
            sstr = 's' if sum(not r for r in is_safe) > 1 else ''
            message = 'target joint state is not safe, check%s that failed:\n' % sstr
            for i, is_safe_result in enumerate(is_safe):
                if not is_safe_result:
                    message += '  %s\n' % self.checkers[i].__class__.__name__.lower()

            unsafe_status_msg = Status()
            unsafe_status_msg.header.stamp = rospy.Time.now()
            unsafe_status_msg.header.seq = self.seq
            unsafe_status_msg.header.is_safe = False

            self.status_pub.publish(unsafe_status_msg)

            # Stop target joint state subscriber
            self.stop_target_joint_state_subscriber()

    def spin(self):
        rospy.spin()

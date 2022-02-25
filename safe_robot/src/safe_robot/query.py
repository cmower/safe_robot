import rospy
import numpy as np
import pyexotica as exo

class Query:

    def __init__(self):

        # Class variables
        self.ttarg = None
        self.qtarg = None

        # Get xml filename
        xml = rospy.get_param('~exotica_xml_filename')

        # Setup exotica
        solver = exo.Setup.load_solver(xml)
        self.scene = solver.get_problem().get_scene()
        self.kinematic_tree = self.scene.get_kinematic_tree()
        self.joint_names = self.scene.get_controlled_joint_names()
        self.ndof = len(self.joint_names)

    def get_joint_position_lower_limit(self):
        lim = self.kinematic_tree.get_joint_limits()
        return lim[:, 0]

    def get_joint_position_upper_limit(self):
        lim = self.kinematic_tree.get_joint_limits()
        return lim[:, 1]

    def get_joint_velocity_limit(self):
        return self.kinematic_tree.get_velocity_limits()

    def set_target(self, t, msg):

        # Set time target
        self.ttarg = t

        # Extract joint state from msg
        self.qtarg = np.array([msg.position[msg.name.index(name)] for name in self.joint_names])

        # Update Exotica
        self.scene.update(self.qtarg)

    def get_link_position(self, link):
        return self.scene.fk(link).get_translation()

    def is_state_valid(self, safe_distance):
        return self.scene.is_state_valid(safe_distance=safe_distance)

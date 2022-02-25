import rospy
import numpy as np
from functools import partial
from abc import ABC, abstractmethod

######################################
## Base Checker class

class Checker(ABC):


    def __init__(self, query):
        self.query = query
        self._active = rospy.get_param('~checker/%s/active' % self.__class__.__name__.lower(), True)
        self.init()


    @abstractmethod
    def init(self):
        pass


    @abstractmethod
    def _is_safe(self):
        pass


    def is_safe(self):
        is_safe = True
        if self._active:
            is_safe = self._is_safe()
        return is_safe


######################################
## Joint limit checker

class JointLimit(Checker):


    def get_limit_param(self, param_name, default):
        if rospy.has_param(param_name):
            return np.fromstring(rospy.get_param(param_name), sep=' ', dtype=float)
        else:
            return np.asarray(default)


    @abstractmethod
    def check_indicators(self):
        pass


    def _is_safe(self):
        return all(self.check_indicators())


######################################
## Joint position limit checker

class JointPositionLimit(JointLimit):


    def init(self):
        self.lo = self.get_limit_param(
            '~joint_position_lower_limit',
            self.query.get_joint_position_lower_limit(),
        )
        self.up = self.get_limit_param(
            '~joint_position_upper_limit',
            self.query.get_joint_position_upper_limit(),
        )


    def check_indicators(self):
        return np.logical_and(self.lo <= self.query.qtarg, self.query.qtarg <= self.up)


######################################
## Joint velocity limit checker

class JointVelocityLimit(JointLimit):


    def init(self):
        self.lim = self.get_limit_param(
            '~joint_velocity_limit', self.query.get_joint_velocity_limit()
        )
        self.prev = None


    def check_indicators(self):
        if self.prev is None:
            self.prev = (self.ttarg, self.qtarg)
            return True
        t, q = self.prev
        dt = self.query.ttarg - t
        dq = self.query.qtarg - q
        self.prev = (self.query.ttarg, self.query.qtarg)
        return abs(dq/dt) <= self.lim


######################################
## Link box limit checker

class LinkBoxLimit(Checker):


    def init(self):
        self.links = rospy.get_param('~links', [])
        if isinstance(self.links, str):
            self.links = self.links.split(' ')

        def in_lim(p, lim):
            return lim[0] <= p <= lim[1]

        self.in_xlim = partial(in_lim, lim=self.get_lim_param('~box_xlim'))
        self.in_ylim = partial(in_lim, lim=self.get_lim_param('~box_ylim'))
        self.in_zlim = partial(in_lim, lim=self.get_lim_param('~box_zlim'))

    def get_lim_param(self, name):
        if rospy.has_param(name):
            return np.fromstring(rospy.get_param(name), sep=' ', dtype=float)
        else:
            return [-np.inf, np.inf]


    def in_lim(self, link):
        p = self.query.get_link_position(link)
        return self.in_xlim(p) and self.in_ylim(p) and self.in_zlim(p)


    def _is_safe(self):
        return all(self.in_lim(link) for link in self.links)


######################################
## EXOTica state validity checker

class StateValidity(Checker):


    def init(self):
        self.safe_distance = np.clip(rospy.get_param('~safe_distance', 0.0), 0.0, np.inf)


    def _is_safe(self):
        return self.query.is_state_valid(self.safe_distance)

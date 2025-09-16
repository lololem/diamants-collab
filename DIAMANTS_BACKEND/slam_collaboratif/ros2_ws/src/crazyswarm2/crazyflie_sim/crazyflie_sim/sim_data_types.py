# DIAMANTS - Drone Intelligence for Advanced Mapping and Navigation Through Swarms
# 
# Copyright (c) 2025 DIAMANTS Project Contributors
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import numpy as np


class State:
    """Class that stores the state of a UAV as used in the simulator interface."""

    def __init__(self, pos=np.zeros(3), vel=np.zeros(3),
                 quat=np.array([1, 0, 0, 0]), omega=np.zeros(3)):
        # internally use one numpy array
        self._state = np.empty(13)
        self.pos = pos
        self.vel = vel
        self.quat = quat
        self.omega = omega

    @property
    def pos(self):
        """Position [m; world frame]."""
        return self._state[0:3]

    @pos.setter
    def pos(self, value):
        self._state[0:3] = value

    @property
    def vel(self):
        """Velocity [m/s; world frame]."""
        return self._state[3:6]

    @vel.setter
    def vel(self, value):
        self._state[3:6] = value

    @property
    def quat(self):
        """Quaternion [qw, qx, qy, qz; body -> world]."""
        return self._state[6:10]

    @quat.setter
    def quat(self, value):
        self._state[6:10] = value

    @property
    def omega(self):
        """Angular velocity [rad/s; body frame]."""
        return self._state[10:13]

    @omega.setter
    def omega(self, value):
        self._state[10:13] = value

    def __repr__(self) -> str:
        return 'State pos={}, vel={}, quat={}, omega={}'.format(
            self.pos, self.vel, self.quat, self.omega)


class Action:
    """Class that stores the action of a UAV as used in the simulator interface."""

    def __init__(self, rpm):
        # internally use one numpy array
        self._action = np.empty(4)
        self.rpm = rpm

    @property
    def rpm(self):
        """Rotation per second [rpm]."""
        return self._action

    @rpm.setter
    def rpm(self, value):
        self._action = value

    def __repr__(self) -> str:
        return 'Action rpm={}'.format(self.rpm)

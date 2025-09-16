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

from __future__ import annotations

import math

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from ..sim_data_types import Action, State


class Visualization:
    """Publishes ROS 2 transforms of the states, so that they can be visualized in RVIZ."""

    def __init__(
        self,
        node: Node,
        params: dict,
        names: list[str],
        states: list[State],
        reference_frames: list[str],
    ):
        self.node = node
        self.names = names
        self.reference_frames = reference_frames
        self.tfbr = TransformBroadcaster(self.node)

    def step(self, t, states: list[State], states_desired: list[State], actions: list[Action]):
        # publish transformation to visualize in rviz
        msgs = []
        for name, state, reference_frame in zip(self.names, states, self.reference_frames):
            msg = TransformStamped()
            msg.header.stamp.sec = math.floor(t)
            msg.header.stamp.nanosec = int((t - msg.header.stamp.sec) * 1e9)
            msg.header.frame_id = reference_frame
            msg.child_frame_id = name
            msg.transform.translation.x = state.pos[0]
            msg.transform.translation.y = state.pos[1]
            msg.transform.translation.z = state.pos[2]
            msg.transform.rotation.x = state.quat[1]
            msg.transform.rotation.y = state.quat[2]
            msg.transform.rotation.z = state.quat[3]
            msg.transform.rotation.w = state.quat[0]
            msgs.append(msg)
        self.tfbr.sendTransform(msgs)

    def shutdown(self):
        pass

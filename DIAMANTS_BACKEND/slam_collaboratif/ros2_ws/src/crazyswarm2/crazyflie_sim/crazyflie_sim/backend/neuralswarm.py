# DIAMANTS V3 - Drone Intelligence for Advanced Mapping and Navigation Through Swarms
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

"""
This implementes interaction force prediction using NeuralSwarm(2).

See https://github.com/aerorobotics/neural-swarm

Logic copied from https://github.com/aerorobotics/neural-swarm/blob/master/planning/robots.py
"""

from pathlib import Path

import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
import torch
import torch.nn as nn
import torch.nn.functional as F

from .np import Quadrotor
from ..sim_data_types import Action, State


# H is the dimension of the hidden state
class phi_Net(nn.Module):

    def __init__(self, inputdim=6, hiddendim=40):
        super(phi_Net, self).__init__()
        self.fc1 = nn.Linear(inputdim, 25)
        self.fc2 = nn.Linear(25, 40)
        self.fc3 = nn.Linear(40, 40)
        self.fc4 = nn.Linear(40, hiddendim)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        x = self.fc4(x)
        return x


class rho_Net(nn.Module):

    def __init__(self, hiddendim=40):
        super(rho_Net, self).__init__()
        self.fc1 = nn.Linear(hiddendim, 40)
        self.fc2 = nn.Linear(40, 40)
        self.fc3 = nn.Linear(40, 40)
        self.fc4 = nn.Linear(40, 1)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        x = self.fc4(x)
        return x


class NeuralSwarm:

    def __init__(self, model_folder):
        self.H = 20
        self.rho_L_net = rho_Net(hiddendim=self.H)
        self.phi_L_net = phi_Net(inputdim=6, hiddendim=self.H)  # x,y,z,vx,vy,vz
        self.rho_L_net.load_state_dict(torch.load('{}/rho_L.pth'.format(model_folder)))
        self.phi_L_net.load_state_dict(torch.load('{}/phi_L.pth'.format(model_folder)))
        self.rho_S_net = rho_Net(hiddendim=self.H)
        self.phi_S_net = phi_Net(inputdim=6, hiddendim=self.H)  # x,y,z,vx,vy,vz
        self.rho_S_net.load_state_dict(torch.load('{}/rho_S.pth'.format(model_folder)))
        self.phi_S_net.load_state_dict(torch.load('{}/phi_S.pth'.format(model_folder)))
        self.phi_G_net = phi_Net(inputdim=4, hiddendim=self.H)  # z,vx,vy,vz
        self.phi_G_net.load_state_dict(torch.load('{}/phi_G.pth'.format(model_folder)))

    def compute_Fa(self, data_self, data_neighbors):
        rho_input = torch.zeros(self.H)
        cftype, x = data_self
        for cftype_neighbor, x_neighbor in data_neighbors:
            x_12 = torch.zeros(6)
            x_12 = (x_neighbor - x).float()
            if abs(x_12[0]) < 0.2 and abs(x_12[1]) < 0.2 and abs(x_12[3]) < 1.5:
                if cftype_neighbor == 'small' or cftype_neighbor == 'small_powerful_motors':
                    rho_input += self.phi_S_net(x_12)
                elif cftype_neighbor == 'large':
                    rho_input += self.phi_L_net(x_12)
                else:
                    raise Exception('Unknown cftype!')

        # interaction with the ground
        x_12 = torch.zeros(4)
        x_12[0] = 0 - x[2]
        x_12[1:4] = -x[3:6]
        rho_input += self.phi_G_net(x_12)

        if cftype == 'small' or cftype == 'small_powerful_motors':
            faz = self.rho_S_net(rho_input)
        elif cftype == 'large':
            faz = self.rho_L_net(rho_input)
        else:
            raise Exception('Unknown cftype!')

        return np.array([0, 0, faz[0].item()])


class Backend:
    """Backend that is based on the one defined in np.py."""

    def __init__(self, node: Node, names: list[str], states: list[State]):
        self.node = node
        self.names = names
        self.clock_publisher = node.create_publisher(Clock, 'clock', 10)
        self.t = 0
        self.dt = 0.0005

        self.uavs = []
        for state in states:
            uav = Quadrotor(state)
            self.uavs.append(uav)
        self.neuralswarm = NeuralSwarm(Path(__file__).parent / 'data/neuralswarm2')

    def time(self) -> float:
        return self.t

    def step(self, states_desired: list[State], actions: list[Action]) -> list[State]:
        # advance the time
        self.t += self.dt

        fa_data = []
        for uav in self.uavs:
            fa_data.append(('small', torch.hstack(
                (torch.tensor(uav.state.pos), torch.tensor(uav.state.vel)))))

        next_states = []
        for k, (uav, action) in enumerate(zip(self.uavs, actions)):
            # estimate F_a
            # print(k, fa_data[k], fa_data[0:k] + fa_data[k+1:])
            f_a = self.neuralswarm.compute_Fa(fa_data[k], fa_data[0:k] + fa_data[k+1:])
            # convert grams to Newtons
            f_a = f_a / 1000 * 9.81
            # print(f_a)
            uav.step(action, self.dt, f_a)
            next_states.append(uav.state)

        # print(states_desired, actions, next_states)
        # publish the current clock
        clock_message = Clock()
        clock_message.clock = Time(seconds=self.time()).to_msg()
        self.clock_publisher.publish(clock_message)

        return next_states

    def shutdown(self):
        pass


if __name__ == '__main__':

    # test case, see Fig 5g in NeuralSwarm2 paper
    ns = NeuralSwarm(Path(__file__).parent / 'data/neuralswarm2')

    # x, y, z, vx, vy, vz
    states = torch.tensor([
        [0, 0, 0.6, 0, 0, 0],
        [0, -0.1, 0.5, 0, 0, 0],
        [0, 0.1, 0.5, 0, 0, 0],
        [0, 0, 0.3, 0, 0, 0],
    ])

    print(ns.compute_Fa(('small', states[3]),
                        [('small', states[0]),
                        ('small', states[1]),
                        ('small', states[2])]))

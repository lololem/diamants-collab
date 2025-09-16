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

#!/usr/bin/env python

from crazyflie_py import Crazyswarm
import numpy as np


def main():
    Id2 = 231
    Id1 = 5
    Pos1 = np.array([0.0, -0.2, 0.0])
    Pos2 = np.array([0.0, 0.2, 0.0])
    Height1 = 0.4
    Height2 = 0.5
    swapTime = 3

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Height1, duration=3.0)
    timeHelper.sleep(3.5)

    # go to initial positions
    allcfs.crazyfliesById[Id1].goTo(Pos1 + np.array([0, 0, Height1]), 0, 3.0)
    allcfs.crazyfliesById[Id2].goTo(Pos2 + np.array([0, 0, Height2]), 0, 3.0)
    timeHelper.sleep(3.5)

    # swap 1
    allcfs.crazyfliesById[Id1].goTo(Pos2 + np.array([0, 0, Height1]), 0, swapTime)
    allcfs.crazyfliesById[Id2].goTo(Pos1 + np.array([0, 0, Height2]), 0, swapTime)
    timeHelper.sleep(swapTime + 1.5)

    # swap 2
    allcfs.crazyfliesById[Id1].goTo(Pos1 + np.array([0, 0, Height1]), 0, swapTime)
    allcfs.crazyfliesById[Id2].goTo(Pos2 + np.array([0, 0, Height2]), 0, swapTime)
    timeHelper.sleep(swapTime + 1.5)

    allcfs.land(targetHeight=0.02, duration=3.0)
    timeHelper.sleep(3.5)


if __name__ == '__main__':
    main()

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

import time

import rospy
from sensor_msgs.msg import Joy


class Joystick:

    def __init__(self):
        self.lastButtonState = 0
        self.buttonWasPressed = False
        rospy.Subscriber('/joy', Joy, self.joyChanged)

    def joyChanged(self, data):
        if (not self.buttonWasPressed and
                data.buttons[5] == 1 and
                self.lastButtonState == 0):
            self.buttonWasPressed = True
        self.lastButtonState = data.buttons[5]

    def waitUntilButtonPressed(self):
        while not rospy.is_shutdown() and not self.buttonWasPressed:
            time.sleep(0.01)
        self.buttonWasPressed = False

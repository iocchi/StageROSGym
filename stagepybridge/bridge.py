#!/usr/bin/env python2

"""Command Ros from sockets.

It will open a pair of sockets:
  - One is used to receive actions that should be forwarded to ROS.
  - Another is used to send back the observations/state from ROS.
The data exchanged is a subset of those allowed by the repo:
    marrtino_apps.program.robot_cmd_ros
"""

from __future__ import absolute_import, division, print_function
from future import standard_library
standard_library.install_aliases()

import sys
import os
import numpy as np

sys.path.append(os.getenv('MARRTINO_APPS_HOME')+'/program')

import robot_cmd_ros as robot

from .streaming import Sender, Receiver


# NOTE: which actions to choose, what increments? dt?

class StageControls(object):
    """Define actions and observations."""

    def __init__(self):
        """Initialize."""

        self.tv = 0
        self.rv = 0
        self.dt = 0.2
        self.state = [0, 0, 0, 0, 0]  # x,y,th,tv,rv

        self.actions = [
            self.action_faster,
            self.action_slower,
            self.action_turn1,
            self.action_turn2,
            self.action_wait,
        ]

        # Start
        self.ros_init()

    def ros_init(self):
        """Initializations of the ros environment."""

        robot.begin()
        setMaxSpeed(0.5,1.0)
        enableObstacleAvoidance(True)

    def action_faster(self):
        self.tv += 0.1
        return robot.setSpeed(self.tv,self.rv,self.dt,False)

    def action_slower(self):
        self.tv -= 0.1
        return robot.setSpeed(self.tv,self.rv,self.dt,False)

    def action_turn1(self):
        self.rv += 0.05
        return robot.setSpeed(self.tv,self.rv,self.dt,False)

    def action_turn2(self):
        self.rv -= 0.05
        return robot.setSpeed(self.tv,self.rv,self.dt,False)

    def action_wait(self):
        self.rv = 0.0
        robot.wait(self.dt)        
        return True

    def get_state(self):
        p = robot.getRobotPose(frame='gt')
        v = robot.getRobotVel()
        self.state = [p[0],p[1],p[2],v[0],v[1]]
        return self.state


class Connector(object):
    """Connections to operate StageControls."""

    def __init__(self):
        """Initialize."""

        # StageControls
        self.stage_controls = StageControls()

        # TODO: setup connections, calling Sender and Receiver.
        # Also prepare a client


    @staticmethod
    def _state2binary(state):
        """Converts a state vector to bytes."""

        array = np.array(state, dtype=np.float32)
        return array.tobytes()

    @staticmethod
    def _action2binary(action):
        """Converts an action (int) to bytes."""

        array = np.array(action, dtype=np.uint8)
        return array.tobytes()

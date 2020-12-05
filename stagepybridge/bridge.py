#!/usr/bin/env python2

"""Communicate with ROS Stage environment from sockets.

It will open a pair of sockets:
  - One is used to receive actions that should be forwarded to ROS.
  - Another is used to send back the observations/state from ROS.
The allowed actions are a subset of those allowed by the repo:
    marrtino_apps.program.robot_cmd_ros
Currently, the other end of the communication has been implemented in:
    cipollone/ros-stage-rl
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


class StageControls(object):
    """Define actions and observations.

    self.actions defines a list of available actions. Positive indexes are
    mapped to these actions based on their position on this list.
    """

    # NOTE: actions can be personalized here

    def __init__(self):
        """Initialize."""

        self._tv = 0
        self._rv = 0
        self._dt = 0.2
        self.state = [0, 0, 0, 0, 0]  # x,y,th,tv,rv

        self.actions = [
            self._action_faster,
            self._action_slower,
            self._action_turn1,
            self._action_turn2,
            self._action_wait,
        ]
        self.n_actions = len(self.actions)

        # Start
        self.ros_init()


    def ros_init(self):
        """Initializations of the ros environment."""

        robot.begin()
        robot.setMaxSpeed(0.5,1.0)
        robot.enableObstacleAvoidance(True)


    def _action_faster(self):
        self._tv += 0.1
        return robot.setSpeed(self._tv,self._rv,self._dt,False)


    def _action_slower(self):
        self._tv -= 0.1
        return robot.setSpeed(self._tv,self._rv,self._dt,False)


    def _action_turn1(self):
        self._rv += 0.05
        return robot.setSpeed(self._tv,self._rv,self._dt,False)


    def _action_turn2(self):
        self._rv -= 0.05
        return robot.setSpeed(self._tv,self._rv,self._dt,False)


    def _action_wait(self):
        self._rv = 0.0
        robot.wait(self._dt)        
        return True


    def act(self, i):
        """Executes action number i (a positive index)."""

        return self.actions[i]()


    def get_state(self):
        """Computes and returns the state vector."""

        p = robot.getRobotPose(frame='gt')
        v = robot.getRobotVel()
        self.state = [p[0],p[1],p[2],v[0],v[1]]
        return self.state


class Connector(object):
    """Connections to operate StageControls.

    This maintains a socket communication with a remote agent that sends
    actions and receives states.
    """

    # Communication protocol
    actions_port = 30005
    states_port = 30006
    state_msg_len = 20    # a numpy vector of 5 float32
    action_msg_len = 4    # a numpy positive scalar of type int32


    class ActionReceiver(Receiver):
        """Just a wrapper that deserializes actions."""

        def receive(self):
            """Return an action received.

            :return: a scalar int that identifies an action (no bound checks)..
            """

            # Receive
            buff = Receiver.receive(self, wait=True)

            # Deserialize
            assert len(buff) == Connector.action_msg_len
            array = np.frombuffer(buff, dtype=np.int32)
            return array.item()


    class StateSender(Sender):
        """Just a wrapper that serializes states."""

        def send(self, state):
            """Send a state.

            :param state: a numpy array.
            """

            # Serialize
            buff = np.array(state, dtype=np.float32).tobytes()
            assert len(buff) == Connector.state_msg_len

            # Send
            Sender.send(self, buff)


    def __init__(self):
        """Initialize."""

        # StageControls
        self.stage_controls = StageControls()

        # Initialize connections
        self.state_sender = Connector.StateSender(
            msg_length=self.state_msg_len, port=self.states_port, wait=True,
        )
        self.action_receiver = Connector.ActionReceiver(
            msg_length=self.action_msg_len, ip="localhost",
            port=self.actions_port, wait=True,
        )

        # Connect now
        self.state_sender.start()
        print("> Serving states on", self.state_sender.server.server_address)
        print("> Connecting to ", self.action_receiver.ip, ":",
            self.action_receiver.port, " for actions. (pause)",
            sep="", end=" ",
        )
        raw_input()
        self.action_receiver.start()


    def run(self):
        """Loop: continuously execute actions and return states.

        This continuously waits for incoming actions in action_receiver,
        it executes them on StageControls and returns the next
        observation/state.
        This never terminates: use CTRL-C.
        """

        while True:

            # Get an action from the agent
            action = self.action_receiver.receive()

            # Check
            if not 0 <= action < self.stage_controls.n_actions:
                raise RuntimeError(
                    "Action not valid. " + str(action) + " not in " +
                    "[0, " + str(self.stage_controls.n_actions-1) + "]"
                )

            # Move robot
            self.stage_controls.act(action)

            # Return a state
            state = self.stage_controls.get_state()
            self.state_sender.send(state)

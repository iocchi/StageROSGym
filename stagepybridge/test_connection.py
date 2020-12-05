"""Testing the connection.

This file will be deleted.
"""

from __future__ import absolute_import, division, print_function
import numpy as np

from .streaming import Sender, Receiver


# TODO: verify that actions and states are correctly exchanged
# TODO: delete this file

# Communication protocol
actions_port = 30005
states_port = 30006
state_msg_len = 20    # a numpy vector of 5 float32
action_msg_len = 4    # a numpy scalar of type int32


def test():

    # Instantiate
    state_sender = Sender(
        msg_length=state_msg_len, port=states_port, wait=True)
    action_receiver = Receiver(
        msg_length=action_msg_len, ip="localhost", port=actions_port,
        wait=True
    )

    # Start server and client
    #   NOTE: make sure that the couple is started also on the other side
    state_sender.start()
    raw_input("Serving states on " + str(state_sender.server.server_address))
    action_receiver.start()

    # Test loop: the environment returns a random state
    while True:
        action = _binary2action(
            action_receiver.receive(wait=True))
        print("Received action", action)
        state = np.random.random_sample(5).astype(np.float32)
        print("Next (random) state", state)
        state_sender.send(_state2binary(state))

    print("done")


def _state2binary(state):
    """Converts a state vector to bytes."""

    buff = np.array(state, dtype=np.float32).tobytes()
    assert len(buff) == state_msg_len
    return buff

def _binary2action(buff):
    """Converts a byte to an action."""

    assert len(buff) == action_msg_len
    array = np.frombuffer(buff, dtype=np.int32)
    return array.item()

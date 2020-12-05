"""Testing the connection.

This file will be deleted.
"""

from __future__ import absolute_import, division, print_function

from .streaming import Sender, Receiver


# TODO: verify that actions and states are correctly exchanged
# TODO: delete this file

# Communication protocol
actions_port = 30005
states_port = 30006
state_msg_len = 20    # a numpy vector of 5 float32
action_msg_len = 1    # a numpy scalar of type uint8


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

    raw_input()
    print("done")

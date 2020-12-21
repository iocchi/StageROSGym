#!/usr/bin/env python2

from __future__ import absolute_import, division, print_function
import argparse
import random

from . import bridge
from .tools import QuitWithResources


def main():

    # Arguments
    parser = argparse.ArgumentParser(
        description="Allows to control the robot from a remote connection"
    )
    parser.add_argument(
        "-t", "--test", action="store_true",
        help="Test actions and returned states (Implies -v)",
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true",
        help="Verbose mode: prints actions and states."
    )

    args = parser.parse_args()

    if args.test:
        test_actions()                                     # Never ends

    else:
        # Start and loop on bridge connector
        bridge.Connector(verbose=args.verbose).run()       # Never ends


def test_actions():
    """Just test actions and their effect."""

    raw_input("Warning: testing random actions. Are you on a simulator? (Y) ")

    # Ros
    stage_controls = bridge.StageControls(verbose=True)

    # Flags
    quit = False
    def do_quit(): quit = True
    QuitWithResources.add("quit-test", do_quit)

    # Reset
    stage_controls.act(-1)

    while not quit:
        # These functions should print because verbose=True
        action = random.randint(-1, stage_controls.n_actions-1)
        stage_controls.act(action)
        stage_controls.get_state()


if __name__ == "__main__":
    main()

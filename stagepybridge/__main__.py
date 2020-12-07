#!/usr/bin/env python2

from __future__ import absolute_import, division, print_function

from . import bridge


def main():

    # Start and loop on bridge connector
    bridge.Connector().run()              # Never ends


if __name__ == "__main__":
    main()

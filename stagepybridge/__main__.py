#!/usr/bin/env python2

from __future__ import absolute_import, division, print_function

from . import test_connection


def main():

    # Just testing for now TODO: we should instantiate a Connector instead
    test_connection.test()


if __name__ == "__main__":
    main()

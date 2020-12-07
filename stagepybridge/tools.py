"""Python utilities."""

from __future__ import absolute_import, division, print_function

import signal


class QuitWithResources(object):
    """Close the resources when ctrl-c is pressed."""

    __deleters = {}
    __initialized = False

    def __init__(self):
        """Don't instantiate."""

        raise TypeError("Don't instantiate this class")

    @staticmethod
    def close():
        """Close all and quit."""

        for name, deleter in QuitWithResources.__deleters.items():
            deleter()
        quit()

    @staticmethod
    def add(name, deleter):
        """Declare a new resource to be closed.

        :param name: any identifier for this resource.
        :param deleter: callable to be used when closing.
        """

        if not QuitWithResources.__initialized:
            signal.signal(
                signal.SIGINT, lambda sig, frame: QuitWithResources.close())
            QuitWithResources.__initialized = True

        if name in QuitWithResources.__deleters:
            raise ValueError("This name is already used")

        QuitWithResources.__deleters[name] = deleter

    @staticmethod
    def remove(name):
        """Removes a resource.

        :param name: identifier of a resource.
        """

        if name not in QuitWithResources.__deleters:
            raise ValueError(str(name) + " is not a resource")

        QuitWithResources.__deleters.pop(name)

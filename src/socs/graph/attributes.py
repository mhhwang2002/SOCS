#########################################################
# Simulation and Optimization of Complex Systems (SOCS) #
# Author: moon.hwang@gmail.com                          #
#   Date: Oct.26.2026                                   #
#########################################################
"""Dictionary with optional attribute-style access."""

class AttrDict(dict):
    """A dictionary supporting both key and attribute access."""

    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError:
            raise AttributeError(
                f"{type(self).__name__!s} has no attribute {name!r}"
            ) from None

    def __setattr__(self, name, value):
        self[name] = value

    def __delattr__(self, name):
        try:
            del self[name]
        except KeyError:
            raise AttributeError(
                f"{type(self).__name__!s} has no attribute {name!r}"
            ) from None

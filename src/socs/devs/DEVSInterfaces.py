#########################################################
# Simulation and Optimization of Complex Systems (SOCS) #
# Author: moon.hwang@gmail.com                          #
#   Date: Oct.26.2026                                   #
#########################################################
"""
DEVS interface classes.
Originally written in 2011 and modernized for Python 3.
"""

class XY:
    """Interface for DEVS input and output ports."""

    def __init__(self):
        self.IP = {}
        self.OP = {}

    def addInputPorts(self, *args):
        """Add input ports by name."""
        for ipname in args:
            if not isinstance(ipname, str):
                raise TypeError("Input port names must be strings.")
            if ipname in self.IP:
                print(f"{ipname} already exists in IP")
            else:
                self.IP[ipname] = ipname

    def to_IP_s(self):
        """Return a string representation of input port names."""
        return "IP=" + str(list(self.IP.keys()))

    def addOutputPorts(self, *args):
        """Add output ports by name."""
        for opname in args:
            if not isinstance(opname, str):
                raise TypeError("Output port names must be strings.")
            if opname in self.OP:
                print(f"{opname} already exists in OP")
            else:
                self.OP[opname] = opname

    def to_OP_s(self):
        """Return a string representation of output port names."""
        return "OP=" + str(list(self.OP.keys()))


class PortValue:
    """A port-value pair."""

    def __init__(self, port, value=None):
        if not isinstance(port, str):
            raise TypeError("port must be a string.")
        self.port = port
        self.value = value

    def __eq__(self, other):
        return (
            self.__class__ is other.__class__
            and self.port == other.port
            and self.value == other.value
        )

    def __str__(self):
        result = str(self.port)
        if self.value is not None:
            result += ":" + str(self.value)
        return result

    def __hash__(self):
        return hash(str(self))

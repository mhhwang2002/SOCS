#########################################################
# Simulation and Optimization of Complex Systems (SOCS) #
# Author: moon.hwang@gmail.com                          #
#   Date: Oct.26.2026                                   #
#########################################################
"""
DEVS interface classes.
Originally written in 2011 and modernized for Python 3.
"""

from .DEVSBase import HierNamedObj


class Port(HierNamedObj):
    """A communication port."""
    """
    def __eq__(self, other):
        return (
            isinstance(other, self.__class__)
            and self.__class__ is other.__class__
            and self.name == other.name
        )

    def __hash__(self):
        # Keep hashing consistent with equality.
        return hash((self.__class__, self.name))
    """
    def __init__(self, name, parent=None):
        super().__init__(name, parent)

    def __str__(self):
        return super().__str__()


class InputPort(Port):
    """An input port."""


class OutputPort(Port):
    """An output port."""


class XY:
    """Interface for DEVS input and output ports."""

    def __init__(self):
        self.IP = {}
        self.OP = {}

    def addInputPorts(self, *args):
        """Add input ports by name."""
        for ipname in args:
            if ipname in self.IP:
                print(f"{ipname} already exists in IP")
            else:
                self.IP[ipname] = InputPort(ipname, self)

    def to_IP_s(self):
        """Return a string representation of input port names."""
        return "IP=" + str(list(self.IP.keys()))

    def addOutputPorts(self, *args):
        """Add output ports by name."""
        for opname in args:
            if opname in self.OP:
                print(f"{opname} already exists in OP")
            else:
                self.OP[opname] = OutputPort(opname, self)

    def to_OP_s(self):
        """Return a string representation of output port names."""
        return "OP=" + str(list(self.OP.keys()))


class PortValue:
    """A port-value pair."""

    def __init__(self, port, value=None):
        if not isinstance(port, Port):
            raise TypeError(f"{port.__class__} is not Port!")
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


if __name__ == "__main__":
    ip = InputPort("ip0")
    print(ip.__class__)
    print(f"ip={ip}\n")

    op = OutputPort("op0")
    print(op.__class__)
    print(f"op={op}\n")

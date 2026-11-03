"""Load example ported from FDDEVS to the unified FDDEVS class."""

from socs.devs.FDDEVS import FDDEVS
from socs.devs.DEVSInterfaces import PortValue
from socs.devs.Base import isnumeric

class LogBridge(FDDEVS):
    """Atomic LogBridge can handel only one walker at a time."""

    def __init__(self, name, length, parent=None):
        super().__init__(name, parent)
        self.length = length   
        self.addInputPorts("?in1", "?in2")
        self.addOutputPorts("!out1", "!out2")
        self.walker_speed = 0

    def delta_0(self):
        """Reset the Load to its initial idle phase."""
        self.Phase = "I"
        self.walker_speed = 0

    def tau(self):
        """Return the time until the next internal event."""
        if (self.Phase == "S1" or self.Phase == "S2") and self.walker_speed>0:
            return self.length/self.walker_speed
        return float("inf")
        
    def delta_x(self, x):
        """Handle a push event and reschedule if a walker exists"""
        if isnumeric(x.value):
            if self.Phase == "I":
                self.walker_speed = x.value
                if x.port == self.IP["?in1"]:
                    self.Phase = "S1"
                else:
                    self.Phase = "S2"
            elif self.Phase == "S1" or self.Phase == "S2":
                self.Phase = "C" # collided
                self.walker_speed = 0
        return True

    def delta_y(self):
        """Emit the walker poping from walkers and return it."""
        if (self.Phase == "S1" or self.Phase == "S2") and self.walker_speed > 0:
            completed_phase = self.Phase
            walker_speed = self.walker_speed
            self.walker_speed = 0
            self.Phase = "I"
            portname = "!out1" if completed_phase == "S1" else "!out2"
            return PortValue(self.OP[portname], walker_speed)
        return None

    def get_state(self):
        """Return the current LogBridge phase."""
        return self.Phase,self.walker_speed

    def set_state(self, state):
        """Set the current LogBridge phase."""
        self.Phase, self.walker_speed = state

 

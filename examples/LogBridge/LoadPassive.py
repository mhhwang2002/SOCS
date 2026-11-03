"""Load example ported from FDDEVS to the unified FDDEVS class."""

from socs.devs.FDDEVS import FDDEVS
from socs.devs.DEVSInterfaces import PortValue
from socs.devs.SimEngine import SimEngineThread  

class LoadPassive(FDDEVS):
    """Atomic load model with walker speeds."""

    def __init__(self, name, length, int_walker_speed=0, parent=None):
        super().__init__(name, parent)
        self.length = length 
        self.walker_init_speed=int_walker_speed
        self.walker_speed=int_walker_speed
        self.addInputPorts("?in", "?go")
        self.addOutputPorts("!out")

    def delta_0(self):
        """Reset the Load to its initial idle phase.""" 
        self.walker_speed = self.walker_init_speed
        if self.walker_speed:
            self.phase = "B"
        else:
            self.phase = "I"
        
    def tau(self):
        """Return the time until the next internal event."""
        if self.phase == "B" and self.walker_speed:
            return self.length/self.walker_speed
        elif self.phase == "S":
            return 0
        else:
            return float("inf")

    def delta_x(self, x):
        """Handle a push event and reschedule if a walker exists"""
        if self.phase == "I" and x.port == self.IP["?in"]: 
            self.walker_speed = x.value
            return True 
        elif self.phase == "W" and x.port == self.IP["?go"]:
            self.phase = "S"
            return True 
        else:
            return False    

    def delta_y(self):
        """Emit the walker poping from walkers and return it."""
        if self.phase == "B" :    
            self.phase = "W"
            return None
        elif self.phase == "S":
            self.phase = "I"
            return PortValue(self.OP["!out"], self.walker_speed) 
        else:
            return None

    def get_state(self):
        """Return the current Load phase."""
        return self.phase, self.walker_speed

    def set_state(self, state):
        """Set the current Load phase."""
        self.phase, self.walker_speed = state

def make_input_event(model):
    """Create an input event for the toaster's push port."""
    return model, PortValue(model.IP["?go"])

if __name__ == "__main__":
    
    Load = LoadPassive("L1", length=100, int_walker_speed=5)
    engine = SimEngineThread(Load,make_input_event)
    engine.StepByStep = True
    engine.sfactor = float("inf")
    engine.runInteractiveMenu()
    print("good bye~!")

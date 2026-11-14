"""Load example using the DEVS atomic model base."""

from socs.devs.DEVS import DEVS
from socs.devs.TESBase import PortValue

class LoadPassive(DEVS):
    """Atomic load model with walker speeds."""

    def __init__(self, name, length, int_walker_speed=0, parent=None):
        super().__init__(name, parent)
        self.length = length 
        self.walker_init_speed=int_walker_speed
        self.walker_speed=int_walker_speed
        self.add_input_ports("?in", "?go")
        self.add_output_ports("!out")

    def set_initial_state(self):
        """Reset the Load to its initial idle phase.""" 
        self.walker_speed = self.walker_init_speed
        if self.walker_speed:
            self.phase = "B"
        else:
            self.phase = "I"
        
    def tau(self):
        """Return the time until the next internal event."""
        if self.phase == "B" and self.walker_speed > 0:
            return self.length/self.walker_speed
        elif self.phase == "S":
            return 0
        else:
            return float("inf")

    def delta_x(self, x):
        """Handle a push event and reschedule if a walker exists"""
        if self.phase == "I" and x.port == "?in":
            self.walker_speed = x.value
            self.phase = "B"
            return True
        elif self.phase == "W" and x.port == "?go":
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
            walker_speed = self.walker_speed
            self.phase = "I"
            return PortValue(self.OP["!out"], walker_speed)
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
    engine = TESEnginThread(Load,make_input_event)
    engine.StepByStep = True
    engine.sfactor = float("inf")
    engine.run_interactive_menu()
    print("good bye~!")

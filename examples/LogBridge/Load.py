"""Load example using the DEVS atomic model base."""

from socs.devs.DEVS import DEVS
from socs.devs.TESBase import PortValue
from socs.devs.TESEngin import TESEnginThread

class Load(DEVS):
    """Atomic load model with walker speeds."""

    def __init__(self, name, length, int_walker_speed=0, parent=None):
        super().__init__(name, parent)
        self.length = length 
        self.walker_init_speeds=[] if int_walker_speed == 0 else [int_walker_speed]
        self.walker_speeds =[]
        self.add_input_ports("?in")
        self.add_output_ports("!out")

    def set_initial_state(self):
        """Reset the Load to its initial idle phase."""
        self.walker_speeds = self.walker_init_speeds.copy()
        
    def tau(self):
        """Return the time until the next internal event."""
        if len(self.walker_speeds) > 0:
            return self.length/self.walker_speeds[0]
        return float("inf")

    def delta_x(self, x):
        """Handle a push event and reschedule if a walker exists"""
        if x.port == "?in":
            self.walker_speeds.append(x.value)
        if(len(self.walker_speeds)==1): # become a new walker entered
            return True
        return False # no reschedule needed

    def delta_y(self):
        """Emit the walker poping from walkers and return it."""
        if len(self.walker_speeds) > 0:  
            walker_speed = self.walker_speeds.pop(0)
            return PortValue(self.OP["!out"], walker_speed) 
        else:
            return None

    def get_state(self):
        """Return the current Load phase."""
        return self.walker_speeds

    def set_state(self, state):
        """Set the current Load phase."""
        self.walker_speeds = state

if __name__ == "__main__":
    
    Load = Load("L1", length=100, int_walker_speed=5)
    engine = TESEnginThread(Load)
    engine.StepByStep = True
    engine.sfactor = float("inf")
    engine.run_interactive_menu()
    print("good bye~!")

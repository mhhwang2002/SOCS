"""Ping-pong example using the deterministic DEVS model base."""

from socs.devs.DEVS import DEVS
from socs.devs.CoupledDEVS import CoupledDEVS
from socs.devs.TESBase import PortValue
from socs.devs.TESEngin import TESEnginThread
from .Load import Load
from .LogBridge import LogBridge
 

def make_model():
    """Build the coupled load log_bridge model."""
    load11 = Load("L11", 100, 5)
    load12 = Load("L12", 100)
    log_bridge = LogBridge("LB", 100) 
    load21 = Load("L21", 100, 4.5)
    load22 = Load("L22", 100)
    model = CoupledDEVS("Load_LogBridge")
    model.add_components(load11, load12, log_bridge, load21, load22)
    model.add_couples(((load11, "!out"), (log_bridge, "?in1")))
    model.add_couples(((log_bridge, "!out1"), (load12, "?in")))
    model.add_couples(((load21, "!out"), (log_bridge, "?in2")))
    model.add_couples(((log_bridge, "!out2"), (load22, "?in")))
    return model


def run_simulation(model):
    """Run the model in the interactive simulation engine."""
    engine = TESEnginThread(model)
    engine.StepByStep = True
    engine.sfactor = float("inf")
    engine.run_interactive_menu()
 
 
def run_verification(model):
    """Placeholder for the legacy verification workflow."""
    inputs = []
    # The legacy VeriEngine integration is intentionally left disabled.


if __name__ == "__main__":
    model = make_model()
    print("is_network_of(Load) =", model.is_network_of(Load))

    running = True
    while running:
        command = input("(s)im, (v)eri, e(x)it> ").strip().lower()
        if command in ("s", "sim"):
            run_simulation(model)
        elif command in ("v", "veri"):
            run_verification(model)
        elif command in ("x", "exit"):
            running = False
        else:
            print(f"{command!r} is an invalid input.")
    print("good bye~!")

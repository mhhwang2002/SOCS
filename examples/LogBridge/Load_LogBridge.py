"""Ping-pong example using the deterministic DEVS model base."""

from socs.devs.DEVS import DEVS
from socs.devs.CoupledDEVS import CoupledDEVS
from socs.devs.TESBase import PortValue
from socs.devs.TESEngin import TESEnginThread
from .Load import Load
from .LogBridge import LogBridge
 

def makeModel():
    """Build the coupled load log_bridge model."""
    load11 = Load("L11", 100, 5)
    load12 = Load("L12", 100)
    log_bridge = LogBridge("LB", 100) 
    load21 = Load("L21", 100, 4.5)
    load22 = Load("L22", 100)
    model = CoupledDEVS("Load_LogBridge")
    model.addComponents(load11, load12, log_bridge, load21, load22)
    model.addCouples(((load11, "!out"), (log_bridge, "?in1")))
    model.addCouples(((log_bridge, "!out1"), (load12, "?in")))
    model.addCouples(((load21, "!out"), (log_bridge, "?in2")))
    model.addCouples(((log_bridge, "!out2"), (load22, "?in")))
    return model


def runSimulation(model):
    """Run the model in the interactive simulation engine."""
    engine = TESEnginThread(model)
    engine.StepByStep = True
    engine.sfactor = float("inf")
    engine.runInteractiveMenu()
 
 
def runVerification(model):
    """Placeholder for the legacy verification workflow."""
    inputs = []
    # The legacy VeriEngine integration is intentionally left disabled.


if __name__ == "__main__":
    model = makeModel()
    print("isNetworkOf(Load) =", model.isNetworkOf(Load))

    running = True
    while running:
        command = input("(s)im, (v)eri, e(x)it> ").strip().lower()
        if command in ("s", "sim"):
            runSimulation(model)
        elif command in ("v", "veri"):
            runVerification(model)
        elif command in ("x", "exit"):
            running = False
        else:
            print(f"{command!r} is an invalid input.")
    print("good bye~!")

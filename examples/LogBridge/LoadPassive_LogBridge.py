"""Ping-pong example using the deterministic FDDEVS model base."""

from socs.devs.FDDEVS import FDDEVS
from socs.devs.CoupledDEVS import CoupledDEVS
from socs.devs.DEVSInterfaces import PortValue
from socs.devs.SimEngine import SimEngineThread
from .LoadPassive import LoadPassive
from .LogBridge import LogBridge
from .FixedPriorityController import FixedPriorityController
from .FIFOController import FIFOController
 

def makeModel():
    """Build the coupled load log_bridge model."""
    load11 = LoadPassive("L11", 100, 5)
    load12 = LoadPassive("L12", 100)
    log_bridge = LogBridge("LB", 100) 
    load21 = LoadPassive("L21", 100, 4.5)
    load22 = LoadPassive("L22", 100)
    model = CoupledDEVS("Load_LogBridge")
    model.addComponents(load11, load12, log_bridge, load21, load22)
    model.addCouples((load11.OP["!out"], log_bridge.IP["?in1"]))
    model.addCouples((log_bridge.OP["!out1"], load12.IP["?in"])) 
    model.addCouples((load21.OP["!out"], log_bridge.IP["?in2"]))
    model.addCouples((log_bridge.OP["!out2"], load22.IP["?in"])) 
    return model


def select_go_event(model):
    """Select a waiting Load and create its GO input event."""
    waiting_loads = [
        component
        for component in model.children
        if isinstance(component, LoadPassive) and component.phase == "W"
    ]
    if not waiting_loads:
        print("No Load is currently waiting.")
        return None, None

    print("Waiting Loads:", ", ".join(load.name for load in waiting_loads))
    selected_name = input("Send GO to which Load? ").strip()
    target_model = next(
        (load for load in waiting_loads if load.name == selected_name), None
    )
    if target_model is None:
        print(f"{selected_name!r} is not a waiting Load.")
        return None, None

    return target_model, PortValue(target_model.IP["?go"])


def runSimulation(model):
    """Run the model with interactive external GO commands."""
    engine = SimEngineThread(model, inject_CBF=select_go_event)
    engine.StepByStep = True
    engine.sfactor = float("inf")
    engine.runInteractiveMenu()
 
 
def runAutomaticSimulation(model, end_time=300.0):
    """Run the model with automatic fixed-priority GO decisions."""
    engine = SimEngineThread(model)
    engine.bVerbose = False
    engine.set_simulation_time_bound(end_time)
    controller = FixedPriorityController()

    while True:
        result = engine.executeOneStep()
        if result.name == "TimeBoundReached":
            break
        controller.dispatch(engine, model)

    print("Automatic dispatches:", controller.dispatch_history)
    print(model.qstate_to_s())
    return engine, controller



def runAutomaticFIFOSimulation(model, end_time=300.0):
    """Run the model with automatic FIFO GO decisions."""
    engine = SimEngineThread(model)
    engine.bVerbose = False
    engine.set_simulation_time_bound(end_time)
    controller = FIFOController()

    while True:
        result = engine.executeOneStep()
        if result.name == "TimeBoundReached":
            break
        controller.dispatch(engine, model)

    print("FIFO dispatches:", controller.dispatch_history)
    print(model.qstate_to_s())
    return engine, controller


def runVerification(model):
    """Placeholder for the legacy verification workflow."""
    inputs = []
    # The legacy VeriEngine integration is intentionally left disabled.


if __name__ == "__main__":
    model = makeModel()
    print("isNetworkOf(LoadPassive) =", model.isNetworkOf(LoadPassive))

    running = True
    while running:
        command = input("(s)im, (a)uto, (f)ifo, (v)eri, e(x)it> ").strip().lower()
        if command in ("s", "sim"):
            runSimulation(model)
        elif command in ("a", "auto"):
            runAutomaticSimulation(model)
        elif command in ("f", "fifo"):
            runAutomaticFIFOSimulation(model)
        elif command in ("v", "veri"):
            runVerification(model)
        elif command in ("x", "exit"):
            running = False
        else:
            print(f"{command!r} is an invalid input.")
    print("good bye~!")

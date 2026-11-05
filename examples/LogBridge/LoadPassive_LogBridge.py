"""Ping-pong example using the deterministic FDDEVS model base."""

from socs.devs.FDDEVS import FDDEVS
from socs.devs.CoupledDEVS import CoupledDEVS
from socs.devs.DEVSInterfaces import PortValue
from socs.devs.SimEngine import SimEngineThread
from .LoadPassive import LoadPassive
from .LogBridge import LogBridge


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


def runExhaustiveOptimization(
    model=None,
    end_time=200.0,
    include_snapshot_details=True,
    show_state_percentage=False,
):
    """Enumerate all LogBridge decision paths and minimize makespan."""
    from .exhaustive_search import run_logbridge_exhaustive_search

    if model is None:
        model = makeModel()

    result = run_logbridge_exhaustive_search(model, end_time=end_time)
    print("Decision-tree nodes:", result.node_count)
    print("Terminal paths:", result.leaf_count)
    print("Optimal makespan:", result.best_cost)
    print("Number of optimal paths:", len(result.optimal_node_ids))
    for index, node_id in enumerate(result.optimal_node_ids, start=1):
        print(f"Optimal path {index}:")
        if include_snapshot_details:
            from socs.tree import SnapshotFormatter

            formatter = SnapshotFormatter(
                simulation_time_bound=end_time,
                show_state_percentage=show_state_percentage,
            )
            result.tree.print_path(node_id, node_formatter=formatter)
        else:
            result.tree.print_path(node_id)
    return result



if __name__ == "__main__":
    model = makeModel()
    print("isNetworkOf(LoadPassive) =", model.isNetworkOf(LoadPassive))

    running = True
    while running:
        command = input("(s)im, (o)ptimize, e(x)it> ").strip().lower()
        if command in ("s", "sim"):
            runSimulation(model)
        elif command in ("o", "optimize"):
            runExhaustiveOptimization(makeModel())
        elif command in ("x", "exit"):
            running = False
        else:
            print(f"{command!r} is an invalid input.")
    print("good bye~!")

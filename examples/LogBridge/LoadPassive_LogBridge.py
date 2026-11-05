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


def runOptimization(
    model=None,
    end_time=200.0,
    include_snapshot_details=True,
    show_state_percentage=False,
):
    """Optimize the LogBridge schedule and minimize makespan."""
    from .optimization import run_logbridge_optimization

    if model is None:
        model = makeModel()

    result = run_logbridge_optimization(model, end_time=end_time)
    graph = result.graph
    print("Decision-graph nodes:", result.node_count)
    print("Terminal outcomes:", result.outcome_count)
    print("Optimal makespan:", result.best_cost)
    print("Number of optimal paths:", len(result.optimal_outcomes))

    formatter = None
    if include_snapshot_details:
        from socs.tree.formatters import SnapshotFormatter

        formatter = SnapshotFormatter(
            simulation_time_bound=end_time,
            show_state_percentage=show_state_percentage,
        )

    edges_by_id = {
        graph.get_edge_id(edge): edge
        for edge in graph.edges()
    }

    for index, outcome in enumerate(result.optimal_outcomes, start=1):
        print(f"Optimal path {index}:")
        current_node_id = result.root_node_id

        for edge_id in outcome.edge_ids:
            edge = edges_by_id[edge_id]
            if edge.src != current_node_id:
                raise ValueError(
                    f"Outcome edge {edge_id} does not continue from "
                    f"node {current_node_id}."
                )

            snapshot = graph.get_snapshot(current_node_id)
            if formatter is None:
                print(f"Node {current_node_id}")
            else:
                print(formatter(current_node_id, snapshot))

            decision = graph.get_decision(edge)
            print(f"  -- Edge {edge_id}: {decision} -->")
            current_node_id = edge.dst

        snapshot = graph.get_snapshot(current_node_id)
        if formatter is None:
            print(f"Node {current_node_id}")
        else:
            print(formatter(current_node_id, snapshot))

        terminal_branch = result.get_terminal_branch(outcome.terminal_branch_id)
        if terminal_branch.decision is not None:
            print(f"  -- Terminal decision: {terminal_branch.decision} -->")

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
            runOptimization(makeModel())
        elif command in ("x", "exit"):
            running = False
        else:
            print(f"{command!r} is an invalid input.")
    print("good bye~!")

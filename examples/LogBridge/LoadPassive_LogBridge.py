"""Ping-pong example using the deterministic DEVS model base."""

from socs.devs.DEVS import DEVS
from socs.devs.CoupledDEVS import CoupledDEVS
from socs.devs.TESBase import PortValue
from socs.devs.TESEngin import TESEnginThread
from .LoadPassive import LoadPassive
from .LogBridge import LogBridge
from .FixedPriorityController import FixedPriorityController
from .FIFOController import FIFOController
from socs.decision import DecisionRuntimeController, FixedPrioritySelector
from .decision_generator import LogBridgeDecisionGenerator
from .runtime_executor import LogBridgeRuntimeExecutor
from socs.decision.decision import ControlDecision, NoControl
 

def make_model():
    """Build the coupled load log_bridge model."""
    load11 = LoadPassive("L11", 100, 5)
    load12 = LoadPassive("L12", 100)
    log_bridge = LogBridge("LB", 100) 
    load21 = LoadPassive("L21", 100, 4.5)
    load22 = LoadPassive("L22", 100)
    model = CoupledDEVS("Load_LogBridge")
    model.add_components(load11, load12, log_bridge, load21, load22)
    model.add_couples(((load11, "!out"), (log_bridge, "?in1")))
    model.add_couples(((log_bridge, "!out1"), (load12, "?in")))
    model.add_couples(((load21, "!out"), (log_bridge, "?in2")))
    model.add_couples(((log_bridge, "!out2"), (load22, "?in")))
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


def run_simulation(model):
    """Run the model with interactive external GO commands."""
    engine = TESEnginThread(model, inject_CBF=select_go_event)
    engine.StepByStep = True
    engine.sfactor = float("inf")
    engine.run_interactive_menu()
 
 
def run_automatic_simulation(model, end_time=300.0):
    """Run the model with automatic fixed-priority GO decisions."""
    engine = TESEnginThread(model)
    engine.bVerbose = False
    engine.set_simulation_time_bound(end_time)
    controller = FixedPriorityController()

    while True:
        result = engine.execute_one_step()
        if result.name == "TimeBoundReached":
            break
        controller.dispatch(engine, model)

    print("Automatic dispatches:", controller.dispatch_history)
    print(model.qstate_to_s())
    return engine, controller



def run_automatic_fifo_simulation(model, end_time=300.0):
    """Run the model with automatic FIFO GO decisions."""
    engine = TESEnginThread(model)
    engine.bVerbose = False
    engine.set_simulation_time_bound(end_time)
    controller = FIFOController()

    while True:
        result = engine.execute_one_step()
        if result.name == "TimeBoundReached":
            break
        controller.dispatch(engine, model)

    print("FIFO dispatches:", controller.dispatch_history)
    print(model.qstate_to_s())
    return engine, controller



def run_decision_runtime_simulation(model, end_time=100.0):
    """Run LogBridge through the SOCS runtime decision cycle."""
    engine = TESEnginThread(model)
    engine.bVerbose = False
    engine.set_simulation_time_bound(end_time)

    def get_priority(decision):
        if isinstance(decision, ControlDecision) and decision.target_model is not None:
            return {"Load_LogBridge.L11": 20, "Load_LogBridge.L21": 10}.get(decision.target_model, 0)
        if isinstance(decision, NoControl):
            return 0
        return -1

    controller = DecisionRuntimeController(
        engine=engine,
        generator=LogBridgeDecisionGenerator(),
        selector=FixedPrioritySelector(priority=get_priority),
        executor=LogBridgeRuntimeExecutor(),
    )

    dispatch_history = []
    while True:
        step = controller.execute_one_step()

        if step.decision_error is not None:
            raise step.decision_error

        if (
            step.selected_record is not None
            and isinstance(step.selected_record.decision, ControlDecision)
            and step.selected_record.decision.target_model is not None
        ):
            dispatch_history.append(
                (engine.tNow, step.selected_record.decision.target_model.rsplit(".", 1)[-1])
            )

        if step.execution_result.name == "TimeBoundReached":
            break

    return engine, controller, dispatch_history


def run_exhaustive_optimization(
    model=None,
    end_time=200.0,
    include_snapshot_details=True,
    show_state_percentage=False,
):
    """Enumerate all LogBridge decision paths and minimize makespan."""
    from .exhaustive_search import run_logbridge_exhaustive_search

    if model is None:
        model = make_model()

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


def run_verification(model):
    """Placeholder for the legacy verification workflow."""
    inputs = []
    # The legacy VeriEngine integration is intentionally left disabled.


if __name__ == "__main__":
    model = make_model()
    print("is_network_of(LoadPassive) =", model.is_network_of(LoadPassive))

    running = True
    while running:
        command = input("(s)im, (a)uto, (f)ifo, (d)ecision, (o)ptimize, (v)eri, e(x)it> ").strip().lower()
        if command in ("s", "sim"):
            run_simulation(model)
        elif command in ("a", "auto"):
            run_automatic_simulation(model)
        elif command in ("f", "fifo"):
            run_automatic_fifo_simulation(model)
        elif command in ("d", "decision"):
            engine, controller, history = run_decision_runtime_simulation(model)
            print("Decision dispatches:", history)
            print("DEVS executions:", controller.devs_execution_count)
        elif command in ("o", "optimize"):
            run_exhaustive_optimization(make_model())
        elif command in ("v", "veri"):
            run_verification(model)
        elif command in ("x", "exit"):
            running = False
        else:
            print(f"{command!r} is an invalid input.")
    print("good bye~!")

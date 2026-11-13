import pytest

from socs.devs.TESEngin import TESEnginThread
from socs.tree.decision_tree import DecisionTree

from examples.LogBridge.LoadPassive_LogBridge import makeModel
from socs.decision.decision import ControlDecision
from examples.LogBridge.decision_executor import LogBridgeDecisionExecutor


def make_root_snapshot():
    """Run the model until both source Loads are waiting."""

    model = makeModel()
    engine = TESEnginThread(model)
    engine.bVerbose = False

    # Allow both source Loads to reach the waiting phase.
    engine.set_simulation_time_bound(23.0)

    while True:
        result = engine.executeOneStep()
        if result.name == "TimeBoundReached":
            break

    loads = {component.name: component for component in model.children}

    assert loads["L11"].phase == "W"
    assert loads["L21"].phase == "W"
    assert loads["LB"].Phase == "I"

    return engine, engine.capture_snapshot()


def get_model_states(snapshot):
    """Return the states of all models in a system snapshot."""

    return {
        name: child.model_state
        for name, child in snapshot.root_model.children.items()
    }


def test_evaluate_l11_decision():
    engine, root_snapshot = make_root_snapshot()

    evaluator = LogBridgeDecisionExecutor(
        engine=engine,
        end_time=70.0,
    )

    decision = ControlDecision("Load_LogBridge.L11", "?go")

    result = evaluator.execute(root_snapshot, decision)
    states = get_model_states(result)

    assert states["L11"][0] == "I"
    assert states["LB"][0] == "I"
    assert states["L12"][0] == "W"

    # L21 was not selected.
    assert states["L21"][0] == "W"


def test_evaluate_l21_decision():
    engine, root_snapshot = make_root_snapshot()

    evaluator = LogBridgeDecisionExecutor(
        engine=engine,
        end_time=70.0,
    )

    decision = ControlDecision("Load_LogBridge.L21", "?go")

    result = evaluator.execute(root_snapshot, decision)
    states = get_model_states(result)

    assert states["L21"][0] == "I"
    assert states["LB"][0] == "I"
    assert states["L22"][0] == "W"

    # L11 was not selected.
    assert states["L11"][0] == "W"


def test_branch_evaluation_is_independent():
    engine, root_snapshot = make_root_snapshot()

    evaluator = LogBridgeDecisionExecutor(
        engine=engine,
        end_time=70.0,
    )

    # Evaluate the first branch.
    result_l11 = evaluator.execute(
        root_snapshot,
        ControlDecision("Load_LogBridge.L11", "?go"),
    )

    # Evaluate another branch from exactly the same root.
    result_l21 = evaluator.execute(
        root_snapshot,
        ControlDecision("Load_LogBridge.L21", "?go"),
    )

    states_l11 = get_model_states(result_l11)
    states_l21 = get_model_states(result_l21)

    # First branch: L11 entered the bridge.
    assert states_l11["L11"][0] == "I"
    assert states_l11["L21"][0] == "W"
    assert states_l11["L12"][0] == "W"
    assert states_l11["L22"][0] == "I"

    # Second branch: L21 entered the bridge.
    assert states_l21["L11"][0] == "W"
    assert states_l21["L21"][0] == "I"
    assert states_l21["L12"][0] == "I"
    assert states_l21["L22"][0] == "W"

    # The original root snapshot must remain unchanged.
    root_states = get_model_states(root_snapshot)

    assert root_states["L11"][0] == "W"
    assert root_states["L21"][0] == "W"
    assert root_states["LB"][0] == "I"
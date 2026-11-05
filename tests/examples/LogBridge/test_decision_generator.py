import pytest

from socs.devs.Snapshot import ModelSnapshot, SystemSnapshot

from socs.decision.decision import ControlDecision
from examples.LogBridge.decision_generator import generate_decisions, has_decisions


def make_model_snapshot(name, state):
    return ModelSnapshot(
        model_id=hash(name),
        model_path=name,
        common_state={"time_last": 0.0, "time_next": float("inf")},
        model_state=state,
        has_model_state=True,
        children={},
    )


def make_system_snapshot(
    *,
    bridge_state=("I", 0),
    l11_state=("W", 5),
    l21_state=("W", 4.5),
    l12_state=("I", 5),
    l22_state=("I", 4.5),
    x_buffer=None,
):
    children = {
        "LB": make_model_snapshot("LB", bridge_state),
        "L11": make_model_snapshot("L11", l11_state),
        "L21": make_model_snapshot("L21", l21_state),
        "L12": make_model_snapshot("L12", l12_state),
        "L22": make_model_snapshot("L22", l22_state),
    }
    root = ModelSnapshot(
        model_id=1,
        model_path="Load_LogBridge",
        common_state={"time_last": 0.0, "time_next": float("inf")},
        children=children,
    )
    return SystemSnapshot(
        t_now=22.222,
        root_model=root,
        t_next_continuous=float("inf"),
        t_next_animation=float("inf"),
        x_buffer=list(x_buffer or []),
    )


def test_generates_decisions_for_both_waiting_source_loads():
    decisions = list(generate_decisions(make_system_snapshot()))

    assert decisions == [
        ControlDecision("Load_LogBridge.L11", "?go"),
        ControlDecision("Load_LogBridge.L21", "?go"),
        ControlDecision(None),
    ]


def test_generates_decision_only_for_waiting_source_load():
    state = make_system_snapshot(l21_state=("B", 4.5))

    decisions = list(generate_decisions(state))

    assert decisions == [ControlDecision("Load_LogBridge.L11", "?go"), ControlDecision(None)]


def test_does_not_generate_when_bridge_is_busy():
    state = make_system_snapshot(bridge_state=("S1", 5))

    assert list(generate_decisions(state)) == []


def test_does_not_generate_when_a_load_is_already_authorized():
    state = make_system_snapshot(l11_state=("S", 5))

    assert list(generate_decisions(state)) == []


def test_does_not_generate_when_external_input_is_queued():
    state = make_system_snapshot(x_buffer=[(123, object())])

    assert list(generate_decisions(state)) == []


def test_destination_loads_are_not_decision_candidates():
    state = make_system_snapshot(
        l11_state=("I", 5),
        l21_state=("I", 4.5),
        l12_state=("W", 5),
        l22_state=("W", 4.5),
    )

    assert list(generate_decisions(state)) == []


def test_candidate_load_names_can_be_configured():
    decisions = list(generate_decisions(make_system_snapshot(), candidate_load_names=("L21",)))

    assert decisions == [ControlDecision("Load_LogBridge.L21", "?go"), ControlDecision(None)]


def test_rejects_missing_candidate_load():
    with pytest.raises(ValueError, match="missing"):
        list(generate_decisions(make_system_snapshot(), candidate_load_names=("L99",)))


def test_has_decisions_matches_generated_alternatives():
    assert has_decisions(make_system_snapshot())
    assert not has_decisions(make_system_snapshot(bridge_state=("S1", 5)))


def test_has_decisions_does_not_consume_generated_decisions():
    state = make_system_snapshot()

    assert has_decisions(state)
    assert list(generate_decisions(state)) == [
        ControlDecision("Load_LogBridge.L11", "?go"),
        ControlDecision("Load_LogBridge.L21", "?go"),
        ControlDecision(None),
    ]

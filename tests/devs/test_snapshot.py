"""Unit tests for in-memory simulation snapshots and branch exploration."""

from collections import deque
import math

import pytest

from socs.devs import CoupledDEVS, FDDEVS
from socs.devs.SimEngine import SimEngine
from socs.devs.Snapshot import SystemSnapshot


class BranchModel(FDDEVS):
    """Small deterministic model used to test independent decision branches."""

    def __init__(self, name, parent=None):
        super().__init__(name, parent)
        self.value = 0
        self.decisions = []
        self.received = []

    def delta_0(self):
        self.value = 0
        self.decisions = []
        self.received = []

    def tau(self):
        return math.inf

    def delta_x(self, event):
        self.received.append(event)
        return False

    def delta_y(self):
        return None

    def get_state(self):
        return {
            "value": self.value,
            "decisions": list(self.decisions),
            "received": list(self.received),
        }

    def set_state(self, state):
        self.value = state["value"]
        self.decisions = list(state["decisions"])
        self.received = list(state["received"])

    def apply_decision(self, decision):
        self.value += decision
        self.decisions.append(decision)


def make_engine():
    root = CoupledDEVS("Root")
    model = BranchModel("Model")
    root.add_child(model)

    engine = SimEngine(
        root,
        TimeIntervalContinuous=3.0,
        TimeIntervalAnimation=7.0,
    )
    engine.bVerbose = False
    return engine, root, model


def test_capture_and_restore_model_hierarchy():
    engine, root, model = make_engine()
    model.value = 12
    model.decisions = [2, 4]
    engine.tNow = 5.0

    snapshot = engine.capture_snapshot()

    model.value = 99
    model.decisions.append(8)
    engine.tNow = 20.0

    engine.restore_snapshot(snapshot)

    assert model.value == 12
    assert model.decisions == [2, 4]
    assert engine.tNow == 5.0


def test_snapshot_restores_engine_schedules():
    engine, _, _ = make_engine()

    engine.tNow = 4.5
    engine._tNextContinuous = 8.0
    engine._tNextAnimation = 11.0
    snapshot = engine.capture_snapshot()

    engine.tNow = 30.0
    engine._tNextContinuous = 31.0
    engine._tNextAnimation = 32.0

    engine.restore_snapshot(snapshot)

    assert engine.tNow == 4.5
    assert engine._tNextContinuous == 8.0
    assert engine._tNextAnimation == 11.0


def test_snapshot_restores_fifo_xbuffer_and_copies_payloads():
    engine, _, target = make_engine()

    first_payload = {"items": [1]}
    second_payload = {"items": [2]}
    engine.XBuffer = deque([
        (target, first_payload),
        (target, second_payload),
    ])

    snapshot = engine.capture_snapshot()

    first_payload["items"].append(10)
    second_payload["items"].append(20)
    engine.XBuffer.clear()

    engine.restore_snapshot(snapshot)

    assert isinstance(engine.XBuffer, deque)
    assert len(engine.XBuffer) == 2

    restored_first_target, restored_first_payload = engine.XBuffer[0]
    restored_second_target, restored_second_payload = engine.XBuffer[1]

    assert restored_first_target is target
    assert restored_second_target is target
    assert restored_first_payload == {"items": [1]}
    assert restored_second_payload == {"items": [2]}


def test_snapshot_rejects_xbuffer_target_outside_model_hierarchy():
    engine, _, _ = make_engine()
    external_model = BranchModel("External")
    engine.XBuffer.append((external_model, {"value": 1}))

    with pytest.raises(ValueError, match="outside the root hierarchy"):
        engine.capture_snapshot()


def test_snapshot_rejects_restore_when_model_hierarchy_changed():
    engine, root, model = make_engine()
    snapshot = engine.capture_snapshot()

    root.add_child(BranchModel("AddedLater"))

    with pytest.raises(ValueError, match="hierarchy does not match"):
        engine.restore_snapshot(snapshot)

    assert model.value == 0


def test_snapshot_operations_require_paused_engine():
    engine, _, _ = make_engine()
    engine.SkipLoop = False

    with pytest.raises(RuntimeError, match="paused"):
        engine.capture_snapshot()

    with pytest.raises(RuntimeError, match="paused"):
        engine.restore_snapshot(object())


def test_restore_rejects_non_system_snapshot():
    engine, _, _ = make_engine()

    with pytest.raises(TypeError, match="SystemSnapshot"):
        engine.restore_snapshot(object())


def test_two_decision_branches_from_same_snapshot_are_independent():
    engine, _, model = make_engine()

    root_snapshot = engine.capture_snapshot()

    # Explore branch A.
    engine.restore_snapshot(root_snapshot)
    model.apply_decision(5)
    model.apply_decision(3)
    branch_a_state = model.get_state()

    assert branch_a_state["value"] == 8
    assert branch_a_state["decisions"] == [5, 3]

    # Restore the same root and explore branch B.
    engine.restore_snapshot(root_snapshot)
    model.apply_decision(20)
    branch_b_state = model.get_state()

    assert branch_b_state["value"] == 20
    assert branch_b_state["decisions"] == [20]

    # Branch A's captured result remains unchanged after branch B.
    assert branch_a_state["value"] == 8
    assert branch_a_state["decisions"] == [5, 3]

    # The root snapshot remains reusable and unchanged.
    engine.restore_snapshot(root_snapshot)
    assert model.value == 0
    assert model.decisions == []

from copy import deepcopy

import pytest

from socs.devs.DEVS import DEVS
from socs.devs.TESBase import PortValue


class CounterModel(DEVS):
    """A simple counter model for snapshot branching tests."""

    def __init__(self, name):
        super().__init__(name)
        self.value = 0
        self.history = []

    def set_initial_state(self):
        self.value = 0
        self.history = []

    def tau(self):
        return 5.0

    def delta_x(self, event):
        x = event.value
        self.value += x
        self.history.append(x)
        return True

    def delta_y(self):
        return None

    def get_state(self):
        return {
            "value": self.value,
            "history": self.history,
        }

    def set_state(self, state):
        self.value = state["value"]
        self.history = state["history"]


@pytest.fixture
def model():
    instance = CounterModel("Counter")
    instance.set_init_total_state(0.0)
    return instance


def test_branches_are_independent(model):
    """Changes in one branch must not affect another branch."""

    snapshot_0 = model.capture_snapshot()

    # Execute Branch A.
    model.restore_snapshot(snapshot_0)
    model.Delta_x(PortValue(value=5), 0.0)
    snapshot_a = model.capture_snapshot()

    assert model.value == 5
    assert model.history == [5]

    # Restore the initial state and execute Branch B.
    model.restore_snapshot(snapshot_0)
    model.Delta_x(PortValue(value=10), 0.0)
    snapshot_b = model.capture_snapshot()

    assert model.value == 10
    assert model.history == [10]

    # Previously captured Branch A must remain unchanged.
    assert snapshot_a.model_state["value"] == 5
    assert snapshot_a.model_state["history"] == [5]

    # Branch B must also remain independent.
    assert snapshot_b.model_state["value"] == 10
    assert snapshot_b.model_state["history"] == [10]


def test_same_snapshot_and_input_produce_same_result(model):
    """The same initial snapshot and input must produce the same result."""

    snapshot_0 = model.capture_snapshot()

    model.restore_snapshot(snapshot_0)
    model.Delta_x(PortValue(value=7), 0.0)
    result_a = model.capture_snapshot()

    model.restore_snapshot(snapshot_0)
    model.Delta_x(PortValue(value=7), 0.0)
    result_b = model.capture_snapshot()

    assert result_a.model_state == result_b.model_state
    assert result_a.common_state == result_b.common_state


def test_restore_returns_to_initial_state(model):
    """Restoring the initial snapshot must undo branch execution."""

    snapshot_0 = model.capture_snapshot()

    model.Delta_x(PortValue(value=20), 0.0)

    assert model.value == 20
    assert model.tLast == 0.0
    assert model.tNext == 5.0

    model.restore_snapshot(snapshot_0)

    assert model.value == 0
    assert model.history == []
    assert model.tLast == 0.0
    assert model.tNext == 5.0


def test_original_snapshot_is_immutable_during_branching(model):
    """Branch execution must not modify the original snapshot."""

    snapshot_0 = model.capture_snapshot()
    original_state = deepcopy(snapshot_0.model_state)
    original_common_state = deepcopy(snapshot_0.common_state)

    model.Delta_x(PortValue(value=5), 0.0)
    model.capture_snapshot()

    model.restore_snapshot(snapshot_0)
    model.Delta_x(PortValue(value=10), 0.0)
    model.capture_snapshot()

    assert snapshot_0.model_state == original_state
    assert snapshot_0.common_state == original_common_state
    
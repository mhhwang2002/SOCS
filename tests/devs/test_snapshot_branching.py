from copy import deepcopy

import pytest

from socs.devs.DEVS import DEVS


class CounterModel(DEVS):
    """A simple counter model for snapshot branching tests."""

    def __init__(self, name):
        super().__init__(name)
        self.value = 0
        self.history = []

    def delta_0(self):
        self.value = 0
        self.history = []

    def tau(self):
        return 5.0

    def delta_x(self, x):
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
    instance.Delta_0(0.0)
    return instance


def test_branches_are_independent(model):
    """Changes in one branch must not affect another branch."""

    snapshot_0 = model.capture_state()

    # Execute Branch A.
    model.restore_state(snapshot_0)
    model.Delta_x(5, 0.0)
    snapshot_a = model.capture_state()

    assert model.value == 5
    assert model.history == [5]

    # Restore the initial state and execute Branch B.
    model.restore_state(snapshot_0)
    model.Delta_x(10, 0.0)
    snapshot_b = model.capture_state()

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

    snapshot_0 = model.capture_state()

    model.restore_state(snapshot_0)
    model.Delta_x(7, 0.0)
    result_a = model.capture_state()

    model.restore_state(snapshot_0)
    model.Delta_x(7, 0.0)
    result_b = model.capture_state()

    assert result_a.model_state == result_b.model_state
    assert result_a.common_state == result_b.common_state


def test_restore_returns_to_initial_state(model):
    """Restoring the initial snapshot must undo branch execution."""

    snapshot_0 = model.capture_state()

    model.Delta_x(20, 0.0)

    assert model.value == 20
    assert model.timeLast() == 0.0
    assert model.timeNext() == 5.0

    model.restore_state(snapshot_0)

    assert model.value == 0
    assert model.history == []
    assert model.timeLast() == 0.0
    assert model.timeNext() == 5.0


def test_original_snapshot_is_immutable_during_branching(model):
    """Branch execution must not modify the original snapshot."""

    snapshot_0 = model.capture_state()
    original_state = deepcopy(snapshot_0.model_state)
    original_common_state = deepcopy(snapshot_0.common_state)

    model.Delta_x(5, 0.0)
    model.capture_state()

    model.restore_state(snapshot_0)
    model.Delta_x(10, 0.0)
    model.capture_state()

    assert snapshot_0.model_state == original_state
    assert snapshot_0.common_state == original_common_state
    
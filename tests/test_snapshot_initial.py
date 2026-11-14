"""Tests for the initial in-memory snapshot contract."""

from copy import deepcopy

import pytest

from socs.devs.CoupledDEVS import CoupledDEVS
from socs.devs.DEVS import DEVS
from examples.DEVS_Simple.DEVSCounter import Counter


class MutableStateModel(DEVS):
    """Minimal model whose state contains mutable data."""

    def __init__(self, name, parent=None):
        super().__init__(name, parent)
        self.state = {"items": [1, 2]}

    def set_initial_state(self):
        self.state = {"items": [1, 2]}

    def tau(self):
        return 1

    def delta_x(self, x):
        return True

    def delta_y(self):
        self.state["items"].append(3)
        return None

    def get_state(self):
        return self.state

    def set_state(self, state):
        self.state = state


def test_capture_restore_counter_state_and_clock():
    model = Counter(name="counter", count_no=5)
    model.set_init_total_state(0)

    snapshot = model.capture_state()
    original_state = model.get_state()
    original_tl = model.tLast
    original_tn = model.tNext

    model.Delta_y(5)
    model.restore_state(snapshot)

    assert model.get_state() == original_state
    assert model.tLast == original_tl
    assert model.tNext == original_tn


def test_snapshot_is_independent_from_later_model_changes():
    model = Counter(name="counter", count_no=5)
    model.set_init_total_state(0)

    snapshot = model.capture_state()
    saved_state = deepcopy(snapshot.model_state)

    model.sum = 999
    model.count = 999

    assert snapshot.model_state == saved_state
    model.restore_state(snapshot)
    assert model.get_state() == ("C", 1, 0, 5)


def test_snapshot_deep_copies_mutable_model_state():
    model = MutableStateModel("mutable")
    model.set_init_total_state(0)
    snapshot = model.capture_state()

    model.state["items"].append(99)

    assert snapshot.model_state == {"items": [1, 2]}
    model.restore_state(snapshot)
    assert model.state == {"items": [1, 2]}
    assert model.state is not snapshot.model_state


def test_snapshot_cannot_be_restored_to_another_model_instance():
    first = Counter(name="counter", count_no=5)
    second = Counter(name="counter", count_no=5)
    snapshot = first.capture_state()

    with pytest.raises(ValueError, match="Snapshot model mismatch"):
        second.restore_state(snapshot)


def test_coupled_snapshot_restores_children_and_recomputes_imm():
    root = CoupledDEVS("root")
    first = Counter("first", count_no=3)
    second = Counter("second", count_no=3)
    root.add_child(first)
    root.add_child(second)
    root.set_init_total_state(0)

    snapshot = root.capture_state()
    assert root.tNext == 5
    assert root.IMM == {first, second}

    first.Delta_y(5)
    root.update_tn()
    assert first.get_state() == ("C", 2, 1, 3)

    root.restore_state(snapshot)

    assert first.get_state() == ("C", 1, 0, 3)
    assert second.get_state() == ("C", 1, 0, 3)
    assert root.tLast == 0
    assert root.tNext == 5
    assert root.IMM == {first, second}


def test_restore_rejects_changed_model_hierarchy_before_mutating_state():
    root = CoupledDEVS("root")
    first = Counter("first", count_no=3)
    root.add_child(first)
    root.set_init_total_state(0)
    snapshot = root.capture_state()

    first.sum = 77
    root.children.clear()

    with pytest.raises(ValueError, match="hierarchy does not match"):
        root.restore_state(snapshot)

    assert first.sum == 77

def test_snapshot_branch_independence():
    model = Counter("branch_test", count_no=5)

    # Establish a known state and schedule.
    model.tLast = 2.0
    model.tNext = 5.0

    initial_state = model.get_state()
    snapshot = model.capture_state()

    # Branch A: Modify the model state and schedule.
    phase, count, total, count_no = initial_state
    model.set_state((phase, count + 10, total + 100, count_no))
    model.tLast = 4.0
    model.tNext = 9.0

    branch_a_state = model.get_state()
    assert branch_a_state != initial_state

    # Restore the original snapshot before exploring Branch B.
    model.restore_state(snapshot)

    assert model.get_state() == initial_state
    assert model.tLast == 2.0
    assert model.tNext == 5.0

    # Branch B: Explore a different state from the same snapshot.
    model.set_state((phase, count + 1, total + 10, count_no))

    branch_b_state = model.get_state()
    assert branch_b_state != branch_a_state
    assert branch_b_state != initial_state

    # Restore again and verify that Branch B did not alter the snapshot.
    model.restore_state(snapshot)

    assert model.get_state() == initial_state
    assert model.tLast == 2.0
    assert model.tNext == 5.0
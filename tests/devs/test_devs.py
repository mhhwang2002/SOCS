import math

import pytest

from socs.devs.TESBase import HierNamedObj
from socs.devs.DEVS import DEVS
from socs.devs.TESBase import PortValue


class Counter(DEVS):
    """A small deterministic atomic model used by the tests."""

    def __init__(self, name, parent=None, delay=1.0):
        super().__init__(name, parent)
        self.delay = delay
        self.state = 0

    def set_initial_state(self):
        self.state = 0

    def tau(self):
        return self.delay

    def delta_x(self, event):
        value = event.value
        self.state += value
        return value != 0

    def delta_y(self):
        output = self.state
        self.state += 1
        return PortValue(value=output)

    def set_state(self, state):
        self.state = state

    def get_state(self):
        return self.state


def test_callback_time_is_independent_of_schedule_and_restored():
    class Model(Counter):
        def delta_x(self, event):
            assert self.tNow == 1
            assert self.timeElapsed() == 1
            assert self.timeRemaining() == 1
            return super().delta_x(event)

    model = Model("counter", delay=2)
    model.set_init_total_state(0)
    assert model.Delta_x(PortValue(value=0), 1) is False
    assert (model.tNow, model.tLast, model.tNext) == (1, 0, 2)
    snapshot = model.capture_state()
    model.Delta_y(2)
    assert model.tNow == 2
    model.restore_state(snapshot)
    assert model.tNow == 1
    assert model.timeElapsed() == 1
    assert model.timeElapsed(0.5) == 0.5


# DEVS behavior

def test_model_has_name():
    model = Counter("Counter01")
    assert model.name == "Counter01"


def test_model_rejects_empty_name():
    with pytest.raises(ValueError):
        Counter("")


def test_initial_time_fields():
    model = Counter("Counter")
    assert model.tLast == 0.0
    assert math.isinf(model.tNext)


def test_initialization_sets_state_and_schedule():
    model = Counter("Counter", delay=2.5)
    model.set_init_total_state(3.0)

    assert model.get_state() == 0
    assert model.tLast == 3.0
    assert model.tNext == 5.5


def test_tau_must_be_non_negative():
    model = Counter("Counter", delay=-1.0)
    with pytest.raises(ValueError):
        model.ta()


def test_tau_must_not_be_nan():
    model = Counter("Counter", delay=math.nan)
    with pytest.raises(ValueError):
        model.ta()


def test_tau_must_return_a_real_number():
    model = Counter("Counter", delay="one")
    with pytest.raises(TypeError):
        model.ta()


def test_external_transition_reschedules_when_requested():
    model = Counter("Counter", delay=2.0)
    model.set_init_total_state(0.0)

    model.Delta_x(PortValue(value=3), 1.0)

    assert model.get_state() == 3
    assert model.tLast == 1.0
    assert model.tNext == 3.0


def test_external_transition_preserves_schedule_when_not_requested():
    model = Counter("Counter", delay=2.0)
    model.set_init_total_state(0.0)
    original_time_next = model.tNext

    model.Delta_x(PortValue(value=0), 1.0)

    assert model.get_state() == 0
    assert model.tLast == 0.0
    assert model.tNext == original_time_next


def test_external_transition_rejects_time_before_last_event():
    model = Counter("Counter", delay=2.0)
    model.set_init_total_state(1.0)

    with pytest.raises(ValueError):
        model.Delta_x(PortValue(value=1), 0.5)


def test_external_transition_rejects_time_after_next_event():
    model = Counter("Counter", delay=2.0)
    model.set_init_total_state(1.0)

    with pytest.raises(ValueError):
        model.Delta_x(PortValue(value=1), 4.0)


def test_internal_transition_outputs_and_reschedules():
    model = Counter("Counter", delay=2.0)
    model.set_init_total_state(0.0)
    model.set_state(4)

    output = model.Delta_y(2.0)

    assert output.value == 4
    assert model.get_state() == 5
    assert model.tLast == 2.0
    assert model.tNext == 4.0


def test_internal_transition_rejects_wrong_event_time():
    model = Counter("Counter", delay=2.0)
    model.set_init_total_state(0.0)

    with pytest.raises(ValueError):
        model.Delta_y(1.0)


def test_state_string_representation():
    model = Counter("Counter")
    model.set_state(7)

    assert model.state_to_s() == "7"


# Hierarchical naming and parent-child relationships

def test_hierarchical_paths():
    root = HierNamedObj("TrafficSystem")
    network = HierNamedObj("RoadNetwork")
    intersection = Counter("Intersection01")

    root.add_child(network)
    network.add_child(intersection)

    assert root.path == "TrafficSystem"
    assert network.path == "TrafficSystem.RoadNetwork"
    assert intersection.path == "TrafficSystem.RoadNetwork.Intersection01"


def test_same_name_allowed_in_different_branches():
    root = HierNamedObj("TrafficSystem")
    north = HierNamedObj("North")
    south = HierNamedObj("South")
    root.add_child(north)
    root.add_child(south)

    intersection_north = Counter("Intersection01")
    intersection_south = Counter("Intersection01")
    north.add_child(intersection_north)
    south.add_child(intersection_south)

    assert intersection_north.path == "TrafficSystem.North.Intersection01"
    assert intersection_south.path == "TrafficSystem.South.Intersection01"


def test_add_child_sets_parent_and_path():
    root = HierNamedObj("TrafficSystem")
    network = HierNamedObj("RoadNetwork")
    intersection = Counter("Intersection01")

    root.add_child(network)
    network.add_child(intersection)

    assert network.parent is root
    assert intersection.parent is network
    assert intersection.path == "TrafficSystem.RoadNetwork.Intersection01"


def test_add_child_rejects_duplicate_names():
    root = HierNamedObj("TrafficSystem")
    root.add_child(Counter("Intersection01"))

    with pytest.raises(ValueError):
        root.add_child(Counter("Intersection01"))


def test_add_child_rejects_existing_parent():
    root1 = HierNamedObj("System1")
    root2 = HierNamedObj("System2")
    child = HierNamedObj("Network")
    root1.add_child(child)

    with pytest.raises(ValueError):
        root2.add_child(child)


def test_add_child_rejects_self():
    model = HierNamedObj("System")

    with pytest.raises(ValueError):
        model.add_child(model)


def test_add_child_rejects_cycle():
    root = HierNamedObj("Root")
    child = HierNamedObj("Child")
    root.add_child(child)

    with pytest.raises(ValueError):
        child.add_child(root)

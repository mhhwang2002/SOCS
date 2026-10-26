"""Unit tests for CoupledDEVS behavior."""

import pytest

from socs.devs import CoupledDEVS, FDDEVS
from socs.devs.DEVSInterfaces import PortValue


class TimedModel(FDDEVS):
    """Small deterministic model for CoupledDEVS tests."""

    def __init__(self, name, interval=1.0):
        super().__init__(name)
        self.interval = interval
        self.state = 0
        self.received = []
        self.output_events = []

    def delta_0(self):
        self.state = 0
        self.received = []
        self.output_events = []

    def tau(self):
        return self.interval

    def delta_x(self, x):
        self.received.append(x)
        return False

    def delta_y(self):
        self.state += 1
        return list(self.output_events)

    def set_state(self, state):
        self.state = state

    def get_state(self):
        return self.state

    def state_to_s(self):
        return str(self.state)


def test_add_child_accepts_fddevs_and_coupled_devs():
    root = CoupledDEVS("root")
    child = TimedModel("child")
    nested = CoupledDEVS("nested")

    root.add_child(child)
    root.add_child(nested)

    assert child in root.children
    assert nested in root.children
    assert child.parent is root
    assert nested.parent is root


def test_add_child_rejects_unsupported_model_type():
    root = CoupledDEVS("root")

    with pytest.raises(TypeError):
        root.add_child(object())


def test_add_components_adds_each_component():
    root = CoupledDEVS("root")
    first = TimedModel("first")
    second = TimedModel("second")

    root.addComponents(first, second)

    assert root.children == [first, second]


def test_add_couple_requires_port_objects():
    root = CoupledDEVS("root")

    with pytest.raises(TypeError):
        root.addCouple("source", "destination")


def test_add_couple_and_add_couples_store_destinations_without_duplicates():
    root = CoupledDEVS("root")
    source = TimedModel("source")
    target = TimedModel("target")
    source.addOutputPorts("out")
    target.addInputPorts("in")

    root.addCouple(source.OP["out"], target.IP["in"])
    root.addCouples((source.OP["out"], target.IP["in"]))

    assert root.Z[source.OP["out"]] == {target.IP["in"]}


def test_delta_zero_initializes_children_and_updates_next_event():
    root = CoupledDEVS("root")
    first = TimedModel("first", interval=2.0)
    second = TimedModel("second", interval=5.0)
    root.addComponents(first, second)

    root.Delta_0(0.0)

    assert root.timeLast() == 0.0
    assert first.timeLast() == 0.0
    assert second.timeLast() == 0.0
    assert first.timeNext() == 2.0
    assert second.timeNext() == 5.0
    assert root.timeNext() == 2.0
    assert root.IMM == {first}


def test_update_tn_tracks_all_children_with_the_earliest_time():
    root = CoupledDEVS("root")
    first = TimedModel("first", interval=3.0)
    second = TimedModel("second", interval=3.0)
    later = TimedModel("later", interval=8.0)
    root.addComponents(first, second, later)

    root.Delta_0(0.0)
    root.update_tN()

    assert root.timeNext() == 3.0
    assert root.IMM == {first, second}


def test_next_event_returns_earliest_atomic_model_and_ties():
    root = CoupledDEVS("root")
    nested = CoupledDEVS("nested")
    first = TimedModel("first", interval=4.0)
    second = TimedModel("second", interval=4.0)
    later = TimedModel("later", interval=9.0)
    root.add_child(nested)
    nested.addComponents(first, second)
    root.add_child(later)

    root.Delta_0(0.0)

    event = root.next_event()
    assert event is not None
    event_time, models = event
    assert event_time == 4.0
    assert set(models) == {first, second}


def test_next_event_returns_none_when_no_finite_atomic_event_exists():
    root = CoupledDEVS("root")
    root.Delta_0(0.0)

    assert root.next_event() is None


def test_delta_x_rejects_non_port_value():
    root = CoupledDEVS("root")
    root.Delta_0(0.0)

    with pytest.raises(TypeError):
        root.Delta_x("not a PortValue", 0.0)


def test_delta_x_routes_input_to_child_input_port():
    root = CoupledDEVS("root")
    child = TimedModel("child", interval=10.0)
    root.add_child(child)
    root.addInputPorts("external")
    child.addInputPorts("in")
    root.addCouple(root.IP["external"], child.IP["in"])
    root.Delta_0(0.0)

    root.Delta_x(PortValue(root.IP["external"], {"value": 42}), 0.0)

    assert len(child.received) == 1
    assert child.received[0].port is child.IP["in"]
    assert child.received[0].value == {"value": 42}


def test_translate_output_event_routes_to_child_input_port():
    root = CoupledDEVS("root")
    source = TimedModel("source", interval=2.0)
    target = TimedModel("target", interval=10.0)
    root.addComponents(source, target)
    source.addOutputPorts("out")
    target.addInputPorts("in")
    root.addCouple(source.OP["out"], target.IP["in"])
    root.Delta_0(0.0)

    event = PortValue(source.OP["out"], "payload")
    translated = root.translateOutputEvent(event, 0.0)

    assert translated == []
    assert len(target.received) == 1
    assert target.received[0].port is target.IP["in"]
    assert target.received[0].value == "payload"


def test_translate_output_event_passes_through_unconnected_port():
    root = CoupledDEVS("root")
    source = TimedModel("source")
    root.add_child(source)
    source.addOutputPorts("out")
    root.Delta_0(0.0)

    event = PortValue(source.OP["out"], 123)

    assert root.translateOutputEvent(event, 0.0) == [event]


def test_translate_output_event_ignores_invalid_events():
    root = CoupledDEVS("root")

    assert root.translateOutputEvent(None, 0.0) == []
    assert root.translateOutputEvent("not an event", 0.0) == []


def test_get_all_influencee_finds_terminal_input_ports():
    root = CoupledDEVS("root")
    nested = CoupledDEVS("nested")
    source = TimedModel("source")
    target = TimedModel("target")
    root.addComponents(nested, source)
    nested.add_child(target)
    source.addOutputPorts("out")
    target.addInputPorts("in")
    root.addCouple(source.OP["out"], nested.IP.setdefault(
        "external", __import__(
            "socs.devs.DEVSInterfaces", fromlist=["InputPort"]
        ).InputPort("external", nested)
    ))
    nested.addCouple(nested.IP["external"], target.IP["in"])

    influencees = set()
    root.getAllInfluencee(source.OP["out"], influencees)

    assert influencees == {target.IP["in"]}


def test_format_tree_shows_nested_model_hierarchy():
    root = CoupledDEVS("root")
    nested = CoupledDEVS("nested")
    child = TimedModel("child")
    root.addComponents(nested)
    nested.add_child(child)

    tree = root.format_tree()

    assert "root" in tree
    assert "nested" in tree
    assert "child" in tree

"""Unit tests for CoupledDEVS behavior."""

import pytest

from socs.devs import CoupledDEVS, DEVS
from socs.devs.TESBase import PortValue
from socs.devs.CoupledDEVS import CoupledDEVS
from socs.devs.ModelRegistry import ModelRegistry

class TimedModel(DEVS):
    """Small deterministic model for CoupledDEVS tests."""

    def __init__(self, name, interval=1.0):
        super().__init__(name)
        self.interval = interval
        self.state = 0
        self.received = []
        self.output_events = []

    def set_initial_state(self):
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


def test_add_couple_requires_endpoints():
    root = CoupledDEVS("root")

    with pytest.raises(TypeError):
        root.addCouple("source", "destination")


def test_add_couple_and_add_couples_store_destinations_without_duplicates():
    root = CoupledDEVS("root")
    source = TimedModel("source")
    target = TimedModel("target")
    source.addOutputPorts("out")
    target.addInputPorts("in")

    root.addComponents(source, target)
    root.addCouple((source, "out"), (target, "in"))
    root.addCouples(((source, "out"), (target, "in")))

    assert root.Z[(source, "out")] == {(target, "in")}


def test_delta_zero_initializes_children_and_updates_next_event():
    root = CoupledDEVS("root")
    first = TimedModel("first", interval=2.0)
    second = TimedModel("second", interval=5.0)
    root.addComponents(first, second)

    root.set_init_total_state(0.0)

    assert root.tLast == 0.0
    assert first.tLast == 0.0
    assert second.tLast == 0.0
    assert first.tNext == 2.0
    assert second.tNext == 5.0
    assert root.tNext == 2.0
    assert root.IMM == {first}


def test_update_tn_tracks_all_children_with_the_earliest_time():
    root = CoupledDEVS("root")
    first = TimedModel("first", interval=3.0)
    second = TimedModel("second", interval=3.0)
    later = TimedModel("later", interval=8.0)
    root.addComponents(first, second, later)

    root.set_init_total_state(0.0)
    root.update_tN()

    assert root.tNext == 3.0
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

    root.set_init_total_state(0.0)

    event = root.next_event()
    assert event is not None
    event_time, models = event
    assert event_time == 4.0
    assert set(models) == {first, second}


def test_next_event_returns_none_when_no_finite_atomic_event_exists():
    root = CoupledDEVS("root")
    root.set_init_total_state(0.0)

    assert root.next_event() is None


def test_delta_x_rejects_non_port_value():
    root = CoupledDEVS("root")
    root.set_init_total_state(0.0)

    with pytest.raises(TypeError):
        root.Delta_x("not a PortValue", 0.0)


def test_delta_x_routes_input_to_child_input_port():
    root = CoupledDEVS("root")
    child = TimedModel("child", interval=10.0)
    root.add_child(child)
    root.addInputPorts("external")
    child.addInputPorts("in")
    root.addCouple((root, "external"), (child, "in"))
    root.set_init_total_state(0.0)

    root.Delta_x(PortValue(root.IP["external"], {"value": 42}), 0.0)

    assert len(child.received) == 1
    assert child.received[0].port == "in"
    assert child.received[0].value == {"value": 42}


def test_translate_output_event_routes_to_child_input_port():
    root = CoupledDEVS("root")
    source = TimedModel("source", interval=2.0)
    target = TimedModel("target", interval=10.0)
    root.addComponents(source, target)
    source.addOutputPorts("out")
    target.addInputPorts("in")
    root.addCouple((source, "out"), (target, "in"))
    root.set_init_total_state(0.0)

    event = PortValue(source.OP["out"], "payload")
    translated = root.translateOutputEvent(source, event, 0.0)

    assert translated == []
    assert len(target.received) == 1
    assert target.received[0].port == "in"
    assert target.received[0].value == "payload"


def test_translate_output_event_ignores_unconnected_port():
    root = CoupledDEVS("root")
    source = TimedModel("source")
    root.add_child(source)
    source.addOutputPorts("out")
    root.set_init_total_state(0.0)

    event = PortValue(source.OP["out"], 123)

    assert root.translateOutputEvent(source, event, 0.0) == []


def test_translate_output_event_ignores_invalid_events():
    root = CoupledDEVS("root")

    assert root.translateOutputEvent(None, None, 0.0) == []
    assert root.translateOutputEvent(None, "not an event", 0.0) == []


def test_get_all_influencee_finds_terminal_input_ports():
    root = CoupledDEVS("root")
    nested = CoupledDEVS("nested")
    source = TimedModel("source")
    target = TimedModel("target")
    root.addComponents(nested, source)
    nested.add_child(target)
    source.addOutputPorts("out")
    target.addInputPorts("in")
    nested.addInputPorts("external")
    root.addCouple((source, "out"), (nested, "external"))
    nested.addCouple((nested, "external"), (target, "in"))

    influencees = set()
    root.getAllInfluencee((source, "out"), influencees)

    assert influencees == {(target, "in")}


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

def test_root_coupled_devs_owns_model_registry():
    root = CoupledDEVS("Root")

    assert root.model_registry is not None
    assert isinstance(root.model_registry, ModelRegistry)


def test_child_coupled_devs_uses_root_registry():
    root = CoupledDEVS("Root")
    child = CoupledDEVS("Child", parent=root)

    assert child.model_registry is root.model_registry

def test_add_child_registers_child_model():
    root = CoupledDEVS("Root")
    child = TimedModel("Child")

    root.add_child(child)

    assert child.model_id is not None
    assert root.model_registry.get(child.model_id) is child


def test_add_child_registers_entire_subtree():
    root = CoupledDEVS("Root")
    child = CoupledDEVS("Child")
    grandchild = TimedModel("Grandchild")

    child.add_child(grandchild)
    root.add_child(child)

    assert root.model_registry.get(child.model_id) is child
    assert root.model_registry.get(grandchild.model_id) is grandchild
    assert child.model_registry is root.model_registry


def test_add_child_rejects_duplicate_model_uuid():
    root = CoupledDEVS("Root")
    child_a = TimedModel("A")
    child_b = TimedModel("B")

    root.add_child(child_a)
    child_b._model_id = child_a.model_id

    with pytest.raises(ValueError):
        root.add_child(child_b)

    assert child_b not in root.children
    assert len(root.model_registry) == 2

    
def test_remove_child_unregisters_model():
    root = CoupledDEVS("Root")
    child = TimedModel("Child")

    root.add_child(child)
    child_id = child.model_id

    root.remove_child(child)

    assert child.parent is None
    assert child not in root.children
    assert child.model_id == child_id
    assert child_id not in root.model_registry


def test_remove_child_unregisters_entire_subtree():
    root = CoupledDEVS("Root")
    child = CoupledDEVS("Child")
    grandchild = TimedModel("Grandchild")

    child.add_child(grandchild)
    root.add_child(child)

    child_id = child.model_id
    grandchild_id = grandchild.model_id

    root.remove_child(child)

    assert child.parent is None
    assert grandchild.parent is child

    assert child_id not in root.model_registry
    assert grandchild_id not in root.model_registry

    assert child.model_id == child_id
    assert grandchild.model_id == grandchild_id


def test_remove_child_preserves_other_registered_models():
    root = CoupledDEVS("Root")
    child_a = TimedModel("A")
    child_b = TimedModel("B")

    root.add_child(child_a)
    root.add_child(child_b)

    root.remove_child(child_a)

    assert child_b in root.children
    assert root.model_registry.get(child_b.model_id) is child_b
    assert len(root.model_registry) == 2


def test_removed_subtree_can_be_registered_again():
    root = CoupledDEVS("Root")
    child = CoupledDEVS("Child")
    grandchild = TimedModel("Grandchild")

    child.add_child(grandchild)
    root.add_child(child)

    child_id = child.model_id
    grandchild_id = grandchild.model_id

    root.remove_child(child)
    root.add_child(child)

    assert child.model_id == child_id
    assert grandchild.model_id == grandchild_id

    assert root.model_registry.get(child_id) is child
    assert root.model_registry.get(grandchild_id) is grandchild



def test_move_child_to_preserves_model_identity_and_registry():
    root = CoupledDEVS("Root")
    parent_a = CoupledDEVS("A")
    parent_b = CoupledDEVS("B")
    child = TimedModel("Child")

    root.add_child(parent_a)
    root.add_child(parent_b)
    parent_a.add_child(child)

    child_id = child.model_id
    registry = root.model_registry

    parent_a.move_child_to(child, parent_b)

    assert child.parent is parent_b
    assert child not in parent_a.children
    assert child in parent_b.children

    assert child.model_id == child_id
    assert root.model_registry is registry
    assert registry.get(child_id) is child


def test_move_child_to_updates_path():
    root = CoupledDEVS("Root")
    parent_a = CoupledDEVS("A")
    parent_b = CoupledDEVS("B")
    child = TimedModel("Child")

    root.add_child(parent_a)
    root.add_child(parent_b)
    parent_a.add_child(child)

    parent_a.move_child_to(child, parent_b)

    assert child.path == "Root.B.Child"


def test_move_child_to_preserves_subtree():
    root = CoupledDEVS("Root")
    parent_a = CoupledDEVS("A")
    parent_b = CoupledDEVS("B")
    child = CoupledDEVS("Child")
    grandchild = TimedModel("Grandchild")

    root.add_child(parent_a)
    root.add_child(parent_b)
    parent_a.add_child(child)
    child.add_child(grandchild)

    parent_a.move_child_to(child, parent_b)

    assert child.parent is parent_b
    assert grandchild.parent is child
    assert grandchild in child.children
    assert grandchild.path == "Root.B.Child.Grandchild"
    assert root.model_registry.get(grandchild.model_id) is grandchild


def test_move_child_to_rejects_cycle():
    root = CoupledDEVS("Root")
    parent = CoupledDEVS("Parent")
    child = CoupledDEVS("Child")

    root.add_child(parent)
    parent.add_child(child)

    with pytest.raises(ValueError):
        root.move_child_to(parent, child)

    assert parent.parent is root
    assert child.parent is parent


def test_move_child_to_rejects_duplicate_name():
    root = CoupledDEVS("Root")
    parent_a = CoupledDEVS("A")
    parent_b = CoupledDEVS("B")

    child_a = TimedModel("Child")
    child_b = TimedModel("Child")

    root.add_child(parent_a)
    root.add_child(parent_b)
    parent_a.add_child(child_a)
    parent_b.add_child(child_b)

    with pytest.raises(ValueError):
        parent_a.move_child_to(child_a, parent_b)

    assert child_a.parent is parent_a
    assert child_a in parent_a.children


def test_move_child_to_rejects_different_system():
    root_a = CoupledDEVS("RootA")
    root_b = CoupledDEVS("RootB")

    parent_a = CoupledDEVS("A")
    parent_b = CoupledDEVS("B")
    child = TimedModel("Child")

    root_a.add_child(parent_a)
    root_b.add_child(parent_b)
    parent_a.add_child(child)

    with pytest.raises(ValueError):
        parent_a.move_child_to(child, parent_b)

    assert child.parent is parent_a


def test_move_child_to_rejects_non_child():
    root = CoupledDEVS("Root")
    parent_a = CoupledDEVS("A")
    parent_b = CoupledDEVS("B")
    child = TimedModel("Child")

    root.add_child(parent_a)
    root.add_child(parent_b)

    with pytest.raises(ValueError):
        parent_a.move_child_to(child, parent_b)

    assert child.parent is None
    assert child not in parent_b.children

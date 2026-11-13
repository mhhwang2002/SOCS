from math import inf

import pytest

from socs.devs import DEVSph
from socs.devs.TESBase import PortValue


@pytest.fixture
def model():
    model = DEVSph("Timer", variables={"count": 0})
    model.addInputPorts("start", "update")
    model.addOutputPorts("done")
    model.add_phase("Idle", tau=inf, initial=True)
    model.add_phase("Active", tau=lambda model: 4 + model._variable.count)
    model.add_external_transition(
        "Idle", "Active", input_port="start", match=lambda value: value == "go",
        reschedule=True,
    )
    model.add_external_transition(
        "Active", "Active", input_port="update", reschedule=False,
        action=lambda model, event: model._variable.update(count=event.value),
    )
    model.add_internal_transition(
        "Active", "Idle", guard=lambda model, event: model._variable.count >= 0,
        output=lambda model: PortValue("done", model._variable.count),
        action=lambda model, event: model._variable.update(count=0),
    )
    model.set_init_total_state(0)
    return model


def test_external_match_and_reschedule(model):
    model.Delta_x(PortValue("start", "wait"), 1)
    assert model.phase == "Idle"
    assert model.tNext == inf
    model.Delta_x(PortValue("start", "go"), 2)
    assert model.phase == "Active"
    assert model.tLast == 2
    assert model.tNext == 6


@pytest.mark.parametrize("restart", [True, False])
def test_reschedule_callback_sees_updated_variables_and_destination_phase(restart):
    model = DEVSph("Timer", variables={"count": 0})
    model.addInputPorts("go")
    model.add_phase("Before", tau=10, initial=True)
    model.add_phase("After", tau=3)
    event = PortValue("go", 7)

    def reschedule(current, received) -> bool:
        assert current.phase == "After"
        assert current._variable.count == 7
        assert received is event
        return restart

    model.add_external_transition(
        "Before", "After", input_port="go", reschedule=reschedule,
        action=lambda current, received: current._variable.update(count=received.value),
    )
    model.set_init_total_state(0)
    model.Delta_x(event, 2)
    assert (model.tLast, model.tNext) == ((2, 5) if restart else (0, 10))


def test_reschedule_rejects_non_boolean_constant(model):
    with pytest.raises(TypeError, match="reschedule must be a bool"):
        model.add_external_transition("Idle", "Active", input_port="update", reschedule=1)


@pytest.mark.parametrize("value", ["False", 1, None])
def test_reschedule_rejects_non_boolean_callback_result(model, value):
    model.add_external_transition(
        "Idle", "Active", input_port="update", reschedule=lambda current, event: value,
    )
    with pytest.raises(TypeError, match="reschedule must return a bool"):
        model.Delta_x(PortValue("update"), 2)


def test_external_update_preserves_schedule_and_internal_output_precedes_action(model):
    model.Delta_x(PortValue("start", "go"), 2)
    model.Delta_x(PortValue("update", 7), 3)
    assert model._variable == {"count": 7}
    assert model.tLast == 2
    assert model.tNext == 6
    assert model.Delta_y(6) == PortValue("done", 7)
    assert model.get_state() == ("Idle", {"count": 0})
    assert model.tNext == inf


def test_snapshot_restores_phase_variables_and_clock(model):
    model.Delta_x(PortValue("start", "go"), 1)
    model.Delta_x(PortValue("update", 3), 2)
    snapshot = model.capture_state()
    model.Delta_y(5)
    model.restore_state(snapshot)
    assert model.get_state() == ("Active", {"count": 3})
    assert (model.tLast, model.tNext) == (1, 5)
    assert model.Delta_y(5) == PortValue("done", 3)


def test_ambiguous_internal_guards_fail_before_action(model):
    model.add_internal_transition("Active", "Active")
    model.Delta_x(PortValue("start", "go"), 1)
    model.Delta_x(PortValue("update", 3), 2)
    with pytest.raises(ValueError, match="Multiple enabled internal transitions"):
        model.Delta_y(5)
    assert model.get_state() == ("Active", {"count": 3})


def test_dot_contains_declared_phases_ports_and_current_phase(model):
    model.Delta_x(PortValue("start", "go"), 1)
    dot = model.to_dot()
    assert "initial -> p0" in dot
    assert 'p1 [label="Active\\nτ = dynamic", style="filled"' in dot
    assert 'p0 [label="Idle\\nτ = inf", style="dashed"' in dot
    assert "p0 -> p1" in dot
    assert "external start" in dot
    assert "style=solid" in next(line for line in dot.splitlines() if "p0 -> p1" in line)
    assert "style=dashed" in next(line for line in dot.splitlines() if "p1 -> p1" in line)
    assert "Inputs\\nstart\\nupdate" in dot
    assert "Outputs\\ndone" in dot


def test_code_defined_model_records_observed_transitions():
    class Observed(DEVSph):
        def set_initial_state(self):
            self.phase = "Idle"

        def tau(self):
            return inf

        def delta_x(self, event):
            self.phase = "Ready"
            return False

    model = Observed("Observed")
    model.addInputPorts("start")
    model.set_init_total_state(0)
    model.Delta_x(PortValue("start"), 1)
    model.Delta_x(PortValue("start"), 2)
    model.Delta_x(PortValue("start"), 3)

    assert len(model.observed_graph.edges("Idle", "Ready")) == 1
    assert model.observed_graph.edges("Ready", "Ready")[0].attrs.count == 2
    assert "external start / reschedule=False ×2" in model.to_dot()
    model.set_init_total_state(0)
    assert model.observed_graph.num_edges() == 0


def test_diagram_omits_only_declared_no_op_loop(model):
    model.add_external_transition("Idle", "Idle", input_port="update", reschedule=False)
    model.add_external_transition(
        "Idle", "Idle", input_port="start", reschedule=lambda current, event: False,
    )
    dot = model.to_dot()
    assert model.graph.num_edges() == 5
    assert 'p0 -> p0 [label="external update' not in dot
    dynamic = next(line for line in dot.splitlines() if "p0 -> p0" in line)
    assert "reschedule=dynamic" in dynamic and "style=dotted" in dynamic
    changed = next(line for line in dot.splitlines() if "p1 -> p1" in line)
    assert "external update" in changed and "style=dashed" in changed


def test_same_phase_monitor_update_retains_dashed_edge_and_callback_names():
    model = DEVSph("Controller", variables={"occupied": False})
    model.addInputPorts("?traffic")
    model.add_phase("Idle", tau=inf, initial=True)

    def resource_available(current, event):
        return not current._variable.occupied

    def update_occupancy(current, event):
        current._variable.occupied = True

    model.add_external_transition(
        "Idle", "Idle", input_port="?traffic", reschedule=False,
        guard=resource_available, action=update_occupancy,
    )
    model.set_init_total_state(0)
    assert model.Delta_x(PortValue("?traffic"), 1) is False
    assert model.get_state() == ("Idle", {"occupied": True})
    assert (model.tLast, model.tNext) == (0, inf)
    line = next(line for line in model.to_dot().splitlines() if "p0 -> p0" in line)
    assert all(text in line for text in ("?traffic", "resource_available", "update_occupancy", "style=dashed"))

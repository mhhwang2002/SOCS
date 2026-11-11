from math import inf

import pytest

from socs.devs import DEVSph
from socs.devs.DEVSInterfaces import PortValue


@pytest.fixture
def model():
    model = DEVSph("Timer", variables={"count": 0})
    model.addInputPorts("start", "update")
    model.addOutputPorts("done")
    model.add_phase("Idle", tau=inf, initial=True)
    model.add_phase("Active", tau=lambda model: 4 + model.variables["count"])
    model.add_external_transition(
        "Idle", "Active", input_port="start", match=lambda value: value == "go",
        reschedule=True,
    )
    model.add_external_transition(
        "Active", "Active", input_port="update", reschedule=False,
        action=lambda model, event: model.variables.update(count=event.value),
    )
    model.add_internal_transition(
        "Active", "Idle", guard=lambda model, event: model.variables["count"] >= 0,
        output=lambda model: PortValue("done", model.variables["count"]),
        action=lambda model, event: model.variables.update(count=0),
    )
    model.Delta_0(0)
    return model


def test_external_match_and_reschedule(model):
    model.Delta_x(PortValue("start", "wait"), 1)
    assert model.phase == "Idle"
    assert model.timeNext() == inf
    model.Delta_x(PortValue("start", "go"), 2)
    assert model.phase == "Active"
    assert model.timeLast() == 2
    assert model.timeNext() == 6


def test_external_update_preserves_schedule_and_internal_output_precedes_action(model):
    model.Delta_x(PortValue("start", "go"), 2)
    model.Delta_x(PortValue("update", 7), 3)
    assert model.variables == {"count": 7}
    assert model.timeLast() == 2
    assert model.timeNext() == 6
    assert model.Delta_y(6) == PortValue("done", 7)
    assert model.get_state() == ("Idle", {"count": 0})
    assert model.timeNext() == inf


def test_snapshot_restores_phase_variables_and_clock(model):
    model.Delta_x(PortValue("start", "go"), 1)
    model.Delta_x(PortValue("update", 3), 2)
    snapshot = model.capture_state()
    model.Delta_y(5)
    model.restore_state(snapshot)
    assert model.get_state() == ("Active", {"count": 3})
    assert (model.timeLast(), model.timeNext()) == (1, 5)
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
    assert "style=dashed" in next(line for line in dot.splitlines() if "p0 -> p1" in line)
    assert "Inputs\\nstart\\nupdate" in dot
    assert "Outputs\\ndone" in dot


def test_code_defined_model_records_observed_transitions():
    class Observed(DEVSph):
        def delta_0(self):
            self.phase = "Idle"

        def tau(self):
            return inf

        def delta_x(self, event):
            self.phase = "Ready"
            return False

    model = Observed("Observed")
    model.addInputPorts("start")
    model.Delta_0(0)
    model.Delta_x(PortValue("start"), 1)
    model.Delta_x(PortValue("start"), 2)
    model.Delta_x(PortValue("start"), 3)

    assert len(model.observed_graph.edges("Idle", "Ready")) == 1
    assert model.observed_graph.edges("Ready", "Ready")[0].attrs.count == 2
    assert "external start ×2" in model.to_dot()
    model.Delta_0(0)
    assert model.observed_graph.num_edges() == 0

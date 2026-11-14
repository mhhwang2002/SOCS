from examples.LogBridge.LoadPassive_LogBridge import (
    make_model,
    run_decision_runtime_simulation,
)
from examples.LogBridge.LoadPassive import LoadPassive
from examples.LogBridge.LogBridge import LogBridge


def test_runtime_decision_cycle_dispatches_loads_without_collision():
    model = make_model()

    engine, controller, history = run_decision_runtime_simulation(
        model,
        end_time=100.0,
    )

    loads = {
        component.name: component
        for component in model.children
        if isinstance(component, LoadPassive)
    }
    bridge = next(
        component for component in model.children
        if isinstance(component, LogBridge)
    )

    assert history == [(20.0, "L11"), (40.0, "L21")]
    assert controller.devs_execution_count > len(history)
    assert bridge.Phase != "C"
    assert loads["L12"].phase == "W"
    assert loads["L22"].phase == "W"
    assert engine.tNow == 100.0

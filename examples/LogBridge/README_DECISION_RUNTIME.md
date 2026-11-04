# LogBridge Decision Runtime

The runtime example connects the DEVS engine to the decision pipeline:

1. Execute one DEVS event.
2. Capture the resulting system snapshot.
3. Generate feasible `ControlDecision` alternatives and an explicit `NoControl`.
4. Select one candidate using `FixedPrioritySelector`.
5. Apply the selected decision to the live engine.
6. Repeat from the next DEVS event.

`NoControl` intentionally performs no model mutation and queues no input. The next call to `DecisionRuntimeController.execute_one_step()` advances the engine to its next event.

The live runtime executor is separate from `LogBridgeDecisionExecutor`, which restores a snapshot and evaluates a branch to a configured time horizon.

Run the example from the repository root:

```python
from examples.LogBridge.LoadPassive_LogBridge import (
    makeModel,
    runDecisionRuntimeSimulation,
)

engine, controller, history = runDecisionRuntimeSimulation(
    makeModel(),
    end_time=100.0,
)

print(history)
print(controller.devs_execution_count)
```

Expected dispatch history with the example's fixed priorities:

```text
[(20.0, 'L11'), (40.0, 'L21')]
```

The bridge remains collision-free in this run.

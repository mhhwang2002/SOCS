# Decision Runtime Controller

`DecisionRuntimeController` invokes a `DecisionGenerator` after every DEVS execution.

- `ScheduledEventOccurred`: capture a `SystemSnapshot`, then invoke the generator.
- `ExternalEventOccurred`: capture a `SystemSnapshot`, then invoke the generator.
- Continuous, animation, and time-bound steps do not invoke the generator.

The runtime controller only generates and returns candidate decisions. It does not
select or execute them. Selection and execution will be added as a separate runtime
stage so that generation, selection, and action execution remain distinct.

## Example

```python
from examples.LogBridge.LoadPassive_LogBridge import makeModel
from examples.LogBridge.decision_generator import LogBridgeDecisionGenerator
from socs.decision import DecisionRuntimeController
from socs.devs.TESEngin import ExecutionResult, TESEnginThread

model = makeModel()
engine = TESEnginThread(model)
engine.bVerbose = False
engine.set_simulation_time_bound(30.0)

runtime = DecisionRuntimeController(
    engine,
    LogBridgeDecisionGenerator(),
)

while True:
    step = runtime.execute_one_step()
    if step.execution_result is ExecutionResult.TimeBoundReached:
        break

    if step.snapshot is not None:
        print(step.snapshot.t_now, step.decisions)
```

For the default LogBridge model, the first two internal DEVS events occur at
20.000 and 22.222. The generator is called after both events. At 20.000 it
returns the L11 entry decision; at 22.222 it returns decisions for both L11
and L21 because both Loads are waiting and the Bridge is idle.

## Scope

This controller is a synchronous, one-step orchestration layer. Use it as the
entry point instead of calling `engine.executeOneStep()` directly whenever
post-DEVS decision generation is required. It does not yet run the background
`TESEnginThread.run()` loop.

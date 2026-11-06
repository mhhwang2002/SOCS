# SOCS live 3D visualization

This package connects a `SimEngineThread` to Panda3D without calling Panda3D
from the simulation thread. The SOCS thread publishes complete visual states;
the Panda3D task applies them on the rendering thread. A runnable AGV example
is available at `examples/visual3D/simple_demo.py`.

Install the optional renderer dependency with:

```powershell
python -m pip install "socs[visual3d]"
```

Models provide a complete visual state after each simulation step:

```python
from socs.visual3D import VisualEntityState, VisualSimEngineThread
from socs.visual3D.panda_viewer import Panda3DViewer

def visual_state(model):
    agv = next(child for child in model.children if child.name == "AGV1")
    return [VisualEntityState("AGV1", (agv.x, agv.y, agv.z), agv.heading)]

engine = VisualSimEngineThread(
    model,
    visual_state,
    TimeIntervalAnimation=0.1,  # simulation-time units
)
viewer = Panda3DViewer(engine.state_buffer)
viewer.load_entity("AGV1", "models/box")

engine.start()
engine.make_run()
viewer.run()
engine.terminate()
```

The example model must expose `x`, `y`, `z`, and `heading`; replace those
attributes with the state fields used by the actual SOCS model. Entity heading
is in degrees. Create scene nodes with `load_entity()` or call
`attach_entity()` with a custom node on the main thread. `state_provider` runs on
the simulation thread and must only read model data and return
`VisualEntityState` values.

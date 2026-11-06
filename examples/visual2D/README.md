# PySide6 AGV demo

Install the optional dependency and the local SOCS package:

```powershell
python -m pip install -e ".[visual2d]"
```

Run the demo from the repository root:

```powershell
python -m examples.visual2D.simple_AGV_demo
```

Press **A** for automatic continuous running, **I** for interactive mode, and
**Space** to advance one motion phase in interactive mode. Use the mouse wheel
to zoom, right-button drag to pan, and **F** to fit the layout.

The 14-unit sides include a max-speed cruise phase; the 4-unit sides use a
triangular accelerate/decelerate profile. Maximum speed is 5 units/s and
acceleration and deceleration are 5 units/s². Animation uses 20 frames per
simulation-time unit and runs at one simulation-time unit per wall-clock second.
"""Tkinter controls and views for the two-floor elevator example."""

from math import isfinite
import tkinter as tk
from tkinter import ttk

from socs.devs.event_input import EventInput
from socs.devs.phase_diagram import PhaseDiagram


def run_live(engine):
    """Drive one engine and refresh the shaft and phase diagram together."""
    model = engine.model
    engine.TimeIntervalAnimation = 0.05
    engine.executeAnimation(engine.tNow, engine.tNow)
    root = tk.Tk()
    root.title("cDEVSph Elevator")
    views = ttk.Frame(root)
    views.pack()
    canvas = tk.Canvas(views, width=275, height=275, background="white", highlightthickness=0)
    canvas.pack(side="left")
    canvas.create_rectangle(130, 20, 200, 240, outline="#64748b", width=2)
    for floor, y in ((1, 225), (2, 55)):
        canvas.create_text(85, y, text=f"Floor {floor}")
        canvas.create_line(115, y, 130, y, fill="#64748b")
    car = canvas.create_rectangle(139, 208, 191, 238, fill="#93c5fd", outline="#1d4ed8", width=2)
    diagram = PhaseDiagram(
        views, model, width=None, height=275,
        columns=(("Idle", "Braking"), ("Accelerating", "Cruising")),
    )
    diagram.pack(side="left")
    status = ttk.Label(root, padding=10)
    status.pack()
    timer = None

    def refresh():
        state = model._variable
        y = 225 - 170 * state.position / model.FLOOR_HEIGHT
        canvas.coords(car, 139, y - 15, 191, y + 15)
        diagram.refresh(engine.tNow)
        status.configure(
            text=f"t={engine.tNow:.2f}  {model.phase}  "
                 f"{state.position:.2f} m  {state.velocity:.2f} m/s"
        )

    def schedule():
        nonlocal timer
        if timer is None and isfinite(model.tNext):
            time, _ = engine.PickNextExecutionTime_Type()
            timer = root.after(max(1, round(1000 * (time - engine.tNow))), advance)

    def advance():
        nonlocal timer
        timer = None
        engine.executeOneStep()
        refresh()
        schedule()

    def input_applied():
        refresh()
        schedule()

    def cancel_timer(event):
        if event.widget is root and timer is not None:
            root.after_cancel(timer)

    EventInput(root, engine, input_applied, initial_value="2").pack(pady=(0, 8))
    root.bind("<Destroy>", cancel_timer)
    refresh()
    root.mainloop()

"""Run a user-operated DEVS toaster in the console or in a live 2D window."""

from argparse import ArgumentParser
from math import inf, isfinite

from socs.devs import DEVSph
from socs.devs.TESBase import PortValue
from socs.devs.TESEngin import ExecutionResult, TESEnginBase


class DEVSphToaster(DEVSph):
    def __init__(self, name, toasting_time=10, parent=None):
        if not isfinite(toasting_time) or toasting_time <= 0:
            raise ValueError("Toasting time must be finite and positive.")
        super().__init__(name, parent)
        self.add_input_ports("?push")
        self.add_output_ports("!pop")
        self.add_phase("Idle", tau=inf, initial=True)
        self.add_phase("Toasting", tau=toasting_time)
        self.add_external_transition("Idle", "Toasting", input_port="?push", reschedule=True)
        self.add_internal_transition("Toasting", "Idle", output="!pop")


class ToasterEngine(TESEnginBase):
    """Expose the most recent output to the example's controls."""

    def __init__(self, toaster, animation_interval=inf):
        self.last_output = None
        super().__init__(toaster, TimeIntervalAnimation=animation_interval)
        self.bVerbose = False

    def execute_devs_star(self, tNow):
        self.last_output = self.model.Delta_y(tNow)
        return ExecutionResult.ScheduledEventOccurred

    def push(self):
        self.last_output = None
        self.XBuffer.append((self.model, PortValue("?push")))
        self.execute_one_step()


def run_console(toaster):
    engine = ToasterEngine(toaster)
    print("Enter: push, q: quit")
    while True:
        try:
            command = input(f"t={engine.tNow:g} {toaster.phase}> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            return
        if command == "q":
            return
        if command:
            continue
        engine.push()
        print(f"t={engine.tNow:g} {toaster.phase}")
        engine.execute_one_step()
        print(f"t={engine.tNow:g} {toaster.phase} {engine.last_output}")


def run_live(toaster):
    import tkinter as tk
    from tkinter import ttk

    animation_interval = 0.1
    engine = ToasterEngine(toaster, animation_interval=animation_interval)
    root = tk.Tk()
    root.title("DEVSph Toaster")
    diagram = None
    if isinstance(toaster, DEVSph):
        from socs.devs.phase_diagram import PhaseDiagram

        diagram = PhaseDiagram(root, toaster, width=None, height=175)
        diagram.pack()
    else:
        canvas = tk.Canvas(root, width=460, height=175, background="white", highlightthickness=0)
        canvas.pack()
        canvas.create_line(26, 87, 69, 87, arrow=tk.LAST, width=2)
        canvas.create_line(189, 68, 269, 68, arrow=tk.LAST, width=2)
        canvas.create_line(269, 104, 189, 104, arrow=tk.LAST, width=2)
        canvas.create_text(229, 53, text="?push")
        canvas.create_text(229, 119, text="!pop")
        phases = {}
        for phase, x in (("Idle", 130), ("Toasting", 330)):
            oval = canvas.create_oval(
                x - 60, 49, x + 60, 125, width=2, dash=(5, 4) if phase == "Idle" else (),
            )
            label = canvas.create_text(x, 87, justify="center")
            phases[phase] = oval, label
    status = ttk.Label(root, padding=8)
    status.pack()
    controls = ttk.Frame(root, padding=8)
    controls.pack()
    timer = None
    animation = True

    def refresh():
        remaining = toaster.tNext - engine.tNow
        if diagram is not None:
            diagram.refresh(engine.tNow)
        else:
            remaining_text = f"{remaining:g}" if isfinite(remaining) else "∞"
            for phase, (oval, label) in phases.items():
                active = phase == toaster.phase
                canvas.itemconfigure(oval, fill="#bfdbfe" if active else "white")
                canvas.itemconfigure(label, text=f"{phase}\nRemaining = {remaining_text}" if active else phase)
        output = f"    Output: {engine.last_output}" if engine.last_output else ""
        status.configure(text=f"t = {engine.tNow:g}{output}")
        step.configure(state="normal" if not animation and isfinite(remaining) else "disabled")
        mode.configure(text="Mode: Animation" if animation else "Mode: DEVS Only")

    def schedule():
        nonlocal timer
        if timer is None and animation and isfinite(toaster.tNext):
            next_time, _ = engine.pick_next_execution_time_type()
            timer = root.after(max(1, round(1000 * (next_time - engine.tNow))), advance)

    def advance():
        nonlocal timer
        timer = None
        engine.execute_one_step()
        refresh()
        schedule()

    def push():
        engine.push()
        refresh()
        schedule()

    def cancel_timer():
        nonlocal timer
        if timer is not None:
            root.after_cancel(timer)
            timer = None

    def toggle_mode():
        nonlocal animation
        cancel_timer()
        animation = not animation
        engine.TimeIntervalAnimation = animation_interval if animation else inf
        engine.execute_animation(engine.tNow, engine.tNow)
        refresh()
        schedule()

    if diagram is not None:
        from socs.devs.event_input import EventInput

        def input_applied():
            engine.last_output = None
            refresh()
            schedule()

        EventInput(root, engine, input_applied).pack(pady=(0, 8))
    else:
        ttk.Button(controls, text="Push", command=push).pack(side="left", padx=4)
    step = ttk.Button(controls, text="Step", command=advance)
    step.pack(side="left", padx=4)
    mode = ttk.Button(controls, command=toggle_mode)
    mode.pack(side="left", padx=4)
    root.bind("<Destroy>", lambda event: cancel_timer() if event.widget is root else None)
    refresh()
    root.mainloop()


def main(model_type=DEVSphToaster, description=__doc__):
    parser = ArgumentParser(description=description)
    display = parser.add_mutually_exclusive_group()
    display.add_argument("--live", action="store_true", help="Run in a live 2D window.")
    display.add_argument("--console", dest="live", action="store_false", help="Use the console input prompt.")
    parser.set_defaults(live=issubclass(model_type, DEVSph))
    parser.add_argument("--toasting-time", type=float, default=10, help="Toasting time in seconds.")
    args = parser.parse_args()
    if not isfinite(args.toasting_time) or args.toasting_time <= 0:
        parser.error("--toasting-time must be finite and positive")
    toaster = model_type("Toaster", args.toasting_time)
    if args.live:
        run_live(toaster)
    else:
        run_console(toaster)


if __name__ == "__main__":
    main()

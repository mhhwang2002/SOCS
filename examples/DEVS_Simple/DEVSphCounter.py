"""Count periodically and export the phase diagram as Graphviz DOT."""

from argparse import ArgumentParser
from pathlib import Path
from math import isfinite

from socs.devs import DEVSph
from socs.devs.DEVSInterfaces import PortValue


class DEVSphCounter(DEVSph):
    """Count at five-unit intervals and emit the sum before each increment."""

    def __init__(self, name, count_no, parent=None):
        super().__init__(name, parent, variables={"count": 0, "sum": 0})
        if count_no < 1:
            raise ValueError("count_no must be positive.")
        self.count_no = count_no
        self.addOutputPorts("!c")
        self.add_phase("Count", tau=5, initial=True)
        self.add_phase("Done", tau=float("inf"))
        self.add_internal_transition(
            "Count", "Count", guard=self.has_more_ticks,
            action=self.increment, output=self.emit_sum,
        )
        self.add_internal_transition(
            "Count", "Done", guard=self.is_last_tick,
            action=self.increment, output=self.emit_sum,
        )

    @staticmethod
    def has_more_ticks(model, event):
        return model.variables["count"] + 1 < model.count_no

    @staticmethod
    def is_last_tick(model, event):
        return not model.has_more_ticks(model, event)

    @staticmethod
    def increment(model, event):
        model.variables["count"] += 1
        model.variables["sum"] += model.variables["count"]

    @staticmethod
    def emit_sum(model):
        return PortValue("!c", model.variables["sum"])

    def display_state(self):
        return self.phase, self.variables["count"], self.variables["sum"]


def run_live(counter, speed, *, show_full_graph=False):
    """Advance the simulation from the 2D window's event loop."""
    import tkinter as tk
    from tkinter import ttk

    from socs.devs.SimEngine import ExecutionResult, SimEngineBase

    animation_interval = 1
    engine = SimEngineBase(counter, TimeIntervalAnimation=animation_interval)
    engine.bVerbose = False
    root = tk.Tk()
    root.title(counter.__class__.__name__)
    canvas = tk.Canvas(root, width=560, height=210, background="white", highlightthickness=0)
    canvas.pack(fill="x")
    edges = {}
    if show_full_graph:
        layout = {
            phase: (105 + 160 * index, attrs.tau)
            for index, (phase, attrs) in enumerate(counter.graph.V.items())
        }
        for edge in counter.graph.edges():
            start, _ = layout[edge.src]
            end, _ = layout[edge.dst]
            edges[edge.src, edge.dst] = canvas.create_line(
                start + 60, 110, end - 60, 110, arrow=tk.LAST, width=2,
            )
        canvas.create_line(12, 110, 45, 110, arrow=tk.LAST, width=2)
        canvas.configure(scrollregion=(0, 0, max(560, 160 * len(layout) + 25), 210))
        scrollbar = ttk.Scrollbar(root, orient="horizontal", command=canvas.xview)
        scrollbar.pack(fill="x")
        canvas.configure(xscrollcommand=scrollbar.set)
    else:
        layout = {"Count": (165, 5), "Done": (405, float("inf"))}
        canvas.create_line(35, 110, 104, 110, arrow=tk.LAST, width=2)
        edges["Count", "Count"] = canvas.create_line(
            125, 82, 82, 33, 163, 13, 246, 33, 207, 82,
            smooth=True, arrow=tk.LAST, width=2,
        )
        edges["Count", "Done"] = canvas.create_line(224, 110, 345, 110, arrow=tk.LAST, width=2)
    phases = {}
    for phase, (x, tau) in layout.items():
        oval = canvas.create_oval(
            x - 60, 77, x + 60, 143, width=2, dash=(5, 4) if tau == float("inf") else (),
        )
        label = canvas.create_text(x, 110, justify="center")
        phases[phase] = oval, label
    status = ttk.Label(root, padding=12)
    status.pack()
    controls = ttk.Frame(root, padding=8)
    controls.pack()
    timer = None
    running = False
    animation = True
    edge = None

    def active_node():
        return counter.phase if show_full_graph else counter.display_state()[0]

    def refresh():
        _, count, total = counter.display_state()
        current_phase = active_node()
        remaining = counter.timeNext() - engine.tNow
        remaining_text = f"{remaining:g}" if isfinite(remaining) else "∞"
        for phase, (oval, label) in phases.items():
            active = phase == current_phase
            canvas.itemconfigure(oval, fill="#bfdbfe" if active else "white")
            text = f"{phase}\nRemaining = {remaining_text}" if active else phase
            canvas.itemconfigure(label, text=text)
        for item in edges.values():
            canvas.itemconfigure(item, fill="#1d4ed8" if item == edge else "#526173")
        status.configure(
            text=f"t = {engine.tNow:g}    Count = {count}    Sum = {total}"
        )
        done = not isfinite(counter.timeNext())
        play.configure(
            text="Pause" if running else "Play",
            state="normal" if animation and not done else "disabled",
        )
        step.configure(state="normal" if not animation and not done else "disabled")
        mode.configure(text="Mode: Animation" if animation else "Mode: DEVS Only")

    def schedule():
        nonlocal timer
        next_time, _ = engine.PickNextExecutionTime_Type()
        delay = round(1000 * (next_time - engine.tNow) / speed)
        timer = root.after(max(1, delay), advance)

    def advance():
        nonlocal timer, running, edge
        timer = None
        source = active_node()
        result = engine.executeOneStep()
        if result == ExecutionResult.ScheduledEventOccurred:
            edge = edges[source, active_node()]
            if show_full_graph:
                x, _ = layout[active_node()]
                if x + 60 > canvas.canvasx(canvas.winfo_width()):
                    canvas.xview_moveto((x - 105) / max(560, 160 * len(layout) + 25))
        else:
            edge = None
        if running and isfinite(counter.timeNext()):
            schedule()
        else:
            running = False
        refresh()

    def pause():
        nonlocal timer, running
        if timer is not None:
            root.after_cancel(timer)
            timer = None
        running = False

    def toggle():
        nonlocal running
        if running:
            pause()
        else:
            running = True
            schedule()
        refresh()

    def toggle_mode():
        nonlocal animation, running
        pause()
        animation = not animation
        engine.TimeIntervalAnimation = animation_interval if animation else float("inf")
        engine.executeAnimation(engine.tNow, engine.tNow)
        if animation and isfinite(counter.timeNext()):
            running = True
            schedule()
        refresh()

    def reset():
        nonlocal edge
        pause()
        engine.reset()
        edge = None
        if show_full_graph:
            canvas.xview_moveto(0)
        refresh()

    play = ttk.Button(controls, command=toggle)
    play.pack(side="left", padx=4)
    step = ttk.Button(controls, text="Step", command=advance)
    step.pack(side="left", padx=4)
    mode = ttk.Button(controls, command=toggle_mode)
    mode.pack(side="left", padx=4)
    ttk.Button(controls, text="Reset", command=reset).pack(side="left", padx=4)
    toggle()
    root.mainloop()


def main(model_type=DEVSphCounter, description=__doc__, *, show_full_graph=False):
    parser = ArgumentParser(description=description)
    parser.add_argument("--count", type=int, default=10)
    if issubclass(model_type, DEVSph):
        parser.add_argument("--dot", type=Path, help="Write the phase diagram to this file.")
    else:
        parser.set_defaults(dot=None)
    parser.add_argument("--live", action="store_true", help="Run in a live 2D window.")
    parser.add_argument("--speed", type=float, default=5, help="Simulation units per real second.")
    args = parser.parse_args()
    if args.count < 1:
        parser.error("--count must be positive")
    counter = model_type("Counter", args.count)
    if args.live:
        if not isfinite(args.speed) or args.speed <= 0:
            parser.error("--speed must be finite and positive")
        run_live(counter, args.speed, show_full_graph=show_full_graph)
    else:
        counter.Delta_0(0)
        while isfinite(counter.timeNext()):
            time = counter.timeNext()
            counter.Delta_null(time)
            output = counter.Delta_y(time)
            phase, count, total = counter.display_state()
            print(f"t={time:g} phase={phase} count={count} sum={total} {output.port}={output.value}")
    if args.dot:
        args.dot.write_text(counter.to_dot(), encoding="utf-8")


if __name__ == "__main__":
    main()

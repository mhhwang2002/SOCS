"""Two phase-based players that send a ball back and forth."""

from argparse import ArgumentParser
from math import inf, isfinite

from socs.devs import CoupledDEVS, DEVSph
from socs.devs.TESEngin import TESEnginBase


class DEVSphPlayer(DEVSph):
    def __init__(self, name, send_time, *, starts=False):
        super().__init__(name)
        self.add_input_ports("?r")
        self.add_output_ports("!s")
        self.add_phase("Sending", tau=send_time, initial=starts)
        self.add_phase("Waiting", tau=inf, initial=not starts)
        self.add_external_transition("Waiting", "Sending", input_port="?r", reschedule=True)
        self.add_internal_transition("Sending", "Waiting", output="!s")


def make_model():
    model = CoupledDEVS("PingPong")
    player_a = DEVSphPlayer("A", 0.1, starts=True)
    player_b = DEVSphPlayer("B", 0.2)
    model.add_components(player_a, player_b)
    model.add_output_ports("!ping", "!pong")
    model.add_couples(
        ((player_a, "!s"), (player_b, "?r")),
        ((player_b, "!s"), (player_a, "?r")),
        ((player_a, "!s"), (model, "!ping")),
        ((player_b, "!s"), (model, "!pong")),
    )
    return model


def run_live(model, speed):
    import tkinter as tk
    from tkinter import ttk

    from socs.devs.phase_diagram import PhaseDiagram
    from socs.devs.event_input import EventInput

    interval = min(player.graph.vertex_attrs("Sending").tau for player in model.M) / 10
    engine = TESEnginBase(model, TimeIntervalAnimation=interval)
    engine.bVerbose = False
    root = tk.Tk()
    root.title("DEVSph PingPong")
    diagrams = []
    for player in model.M:
        diagram = PhaseDiagram(root, player, width=None, height=145)
        diagram.pack()
        diagrams.append(diagram)
    status = ttk.Label(root, padding=8)
    status.pack()
    controls = ttk.Frame(root, padding=8)
    controls.pack()
    coupling_text = "\n".join(
        f"{source.name}.{output} → {target.name}.{port}"
        for (source, output), destinations in model.Z.items()
        for target, port in sorted(destinations, key=lambda endpoint: (endpoint[0].name, endpoint[1]))
    )
    coupling = ttk.Label(root, text=coupling_text, justify="left", padding=12)
    timer = None
    animation = True
    running = True

    def refresh():
        for diagram in diagrams:
            diagram.refresh(engine.tNow)
        status.configure(text=f"Simulation time = {engine.tNow:.3f}")
        play.configure(text="Pause" if running else "Play", state="normal" if animation else "disabled")
        step.configure(state="disabled" if animation else "normal")
        mode.configure(text="Mode: Animation" if animation else "Mode: DEVS Only")

    def schedule():
        nonlocal timer
        if running:
            next_time, _ = engine.pick_next_execution_time_type()
            timer = root.after(max(1, round(1000 * (next_time - engine.tNow) / speed)), advance)

    def advance():
        nonlocal timer
        timer = None
        engine.execute_one_step()
        refresh()
        schedule()

    def pause():
        nonlocal timer, running
        if timer is not None:
            root.after_cancel(timer)
            timer = None
        running = False

    def toggle_play():
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
        engine.TimeIntervalAnimation = interval if animation else inf
        engine.execute_animation(engine.tNow, engine.tNow)
        running = animation
        refresh()
        schedule()

    def reset():
        pause()
        engine.reset()
        refresh()

    def input_applied():
        nonlocal timer
        if timer is not None:
            root.after_cancel(timer)
            timer = None
        refresh()
        schedule()

    def toggle_coupling():
        if coupling.winfo_manager():
            coupling.pack_forget()
            coupling_button.configure(text="Show Coupling")
        else:
            coupling.pack()
            coupling_button.configure(text="Hide Coupling")

    play = ttk.Button(controls, command=toggle_play)
    play.pack(side="left", padx=4)
    step = ttk.Button(controls, text="Step", command=advance)
    step.pack(side="left", padx=4)
    mode = ttk.Button(controls, command=toggle_mode)
    mode.pack(side="left", padx=4)
    ttk.Button(controls, text="Reset", command=reset).pack(side="left", padx=4)
    coupling_button = ttk.Button(controls, text="Show Coupling", command=toggle_coupling)
    coupling_button.pack(side="left", padx=4)
    EventInput(root, engine, input_applied).pack(pady=(0, 8))
    root.bind("<Destroy>", lambda event: pause() if event.widget is root else None)
    refresh()
    schedule()
    root.mainloop()


def main():
    parser = ArgumentParser(description=__doc__)
    display = parser.add_mutually_exclusive_group()
    display.add_argument("--live", action="store_true", help="Run in a live 2D window (default).")
    display.add_argument("--console", dest="live", action="store_false", help="Print sends in the console.")
    parser.set_defaults(live=True)
    parser.add_argument("--speed", type=float, default=0.1, help="Simulation units per real second.")
    parser.add_argument("--steps", type=int, default=10, help="Number of sends to print in console mode.")
    args = parser.parse_args()
    model = make_model()
    if args.live:
        if not isfinite(args.speed) or args.speed <= 0:
            parser.error("--speed must be finite and positive")
        run_live(model, args.speed)
    else:
        if args.steps < 1:
            parser.error("--steps must be positive")
        engine = TESEnginBase(model)
        engine.bVerbose = False
        for _ in range(args.steps):
            engine.execute_one_step()
            states = " ".join(f"{player.name}={player.phase}" for player in model.M)
            print(f"t={engine.tNow:g} {states}")


if __name__ == "__main__":
    main()

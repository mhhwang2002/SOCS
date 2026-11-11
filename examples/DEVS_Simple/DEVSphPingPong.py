"""Two phase-based players that send a ball back and forth."""

from argparse import ArgumentParser
from math import inf, isfinite

from socs.devs import CoupledDEVS, DEVSph
from socs.devs.SimEngine import SimEngineBase


class DEVSphPlayer(DEVSph):
    def __init__(self, name, send_time, *, starts=False):
        super().__init__(name)
        self.addInputPorts("?r")
        self.addOutputPorts("!s")
        self.add_phase("Sending", tau=send_time, initial=starts)
        self.add_phase("Waiting", tau=inf, initial=not starts)
        self.add_external_transition("Waiting", "Sending", input_port="?r", reschedule=True)
        self.add_internal_transition("Sending", "Waiting", output="!s")


def make_model():
    model = CoupledDEVS("PingPong")
    player_a = DEVSphPlayer("A", 0.1, starts=True)
    player_b = DEVSphPlayer("B", 0.2)
    model.addComponents(player_a, player_b)
    model.addOutputPorts("!ping", "!pong")
    model.addCouples(
        ((player_a, "!s"), (player_b, "?r")),
        ((player_b, "!s"), (player_a, "?r")),
        ((player_a, "!s"), (model, "!ping")),
        ((player_b, "!s"), (model, "!pong")),
    )
    return model


def run_live(model, speed):
    import tkinter as tk
    from tkinter import ttk

    interval = min(player.graph.vertex_attrs("Sending").tau for player in model.M) / 10
    engine = SimEngineBase(model, TimeIntervalAnimation=interval)
    engine.bVerbose = False
    root = tk.Tk()
    root.title("DEVSph PingPong")
    canvas = tk.Canvas(root, width=480, height=300, background="white", highlightthickness=0)
    canvas.pack()
    nodes = {}
    for player, y in zip(model.M, (88, 216)):
        canvas.create_text(32, y, text=player.name, font=("TkDefaultFont", 15, "bold"))
        initial_x = 342 if player.phase == "Sending" else 135
        canvas.create_line(initial_x, y - 65, initial_x, y - 38, arrow=tk.LAST, width=2)
        canvas.create_line(202, y - 12, 274, y - 12, arrow=tk.LAST, width=2, dash=(5, 4))
        canvas.create_line(274, y + 12, 202, y + 12, arrow=tk.LAST, width=2)
        canvas.create_text(238, y - 24, text="?r")
        canvas.create_text(238, y + 25, text="!s")
        for phase, x, dash in (("Waiting", 135, (5, 4)), ("Sending", 342, ())):
            oval = canvas.create_oval(x - 69, y - 37, x + 69, y + 37, width=2, dash=dash)
            label = canvas.create_text(x, y, justify="center")
            nodes[player, phase] = oval, label
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
        for (player, phase), (oval, label) in nodes.items():
            active = player.phase == phase
            canvas.itemconfigure(oval, fill="#bfdbfe" if active else "white")
            remaining = player.timeNext() - engine.tNow
            value = f"{max(0, remaining):.3f}" if isfinite(remaining) else "∞"
            canvas.itemconfigure(label, text=f"{phase}\nRemaining = {value}" if active else phase)
        status.configure(text=f"Simulation time = {engine.tNow:.3f}")
        play.configure(text="Pause" if running else "Play", state="normal" if animation else "disabled")
        step.configure(state="disabled" if animation else "normal")
        mode.configure(text="Mode: Animation" if animation else "Mode: DEVS Only")

    def schedule():
        nonlocal timer
        if running:
            next_time, _ = engine.PickNextExecutionTime_Type()
            timer = root.after(max(1, round(1000 * (next_time - engine.tNow) / speed)), advance)

    def advance():
        nonlocal timer
        timer = None
        engine.executeOneStep()
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
        engine.executeAnimation(engine.tNow, engine.tNow)
        running = animation
        refresh()
        schedule()

    def reset():
        pause()
        engine.reset()
        refresh()

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
    refresh()
    schedule()
    root.mainloop()


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--live", action="store_true", help="Run in a live 2D window.")
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
        engine = SimEngineBase(model)
        engine.bVerbose = False
        for _ in range(args.steps):
            engine.executeOneStep()
            states = " ".join(f"{player.name}={player.phase}" for player in model.M)
            print(f"t={engine.tNow:g} {states}")


if __name__ == "__main__":
    main()

"""Interactive Coke vending machine with a timed Petri net."""

from argparse import ArgumentParser
from math import inf, isfinite

from socs.devs import DEVSPetriNet
from socs.devs.TESBase import PortValue

from .DEVSphToaster import ToasterEngine


class VendingMachine(DEVSPetriNet):
    def __init__(self):
        super().__init__("VendingMachine")
        self.add_input_ports("?1USD", "?Coke")
        self.add_output_ports("!Coke")
        self.add_place("Credit")
        self.add_place("Stock", ["Coke"] * 5)
        self.add_place("Pending")
        self.add_transition("StartSale")
        for _ in range(2):
            self.add_input_arc("Credit", "StartSale", "coin")
        self.add_input_arc("Stock", "StartSale", "drink")
        self.add_output_arc("StartSale", "Pending", "drink")
        self.add_transition("Deliver")
        self.add_input_arc("Pending", "Deliver", "drink")
        self.connect_input_port("?1USD", "Credit", tokens=lambda _: [1])

    def tau(self):
        return 3 if self.tokens("Pending") else inf

    def delta_x(self, event):
        if event.port == "?1USD":
            self.receive_tokens(event)
            return False
        if event.port == "?Coke" and not self.tokens("Pending"):
            if next(self.enabled_bindings("StartSale"), None) is not None:
                self.fire("StartSale")
                return True
        return False

    def delta_y(self):
        self.fire("Deliver")
        return PortValue("!Coke")


def send(engine, port):
    engine.last_output = None
    engine.XBuffer.append((engine.model, PortValue(port)))
    engine.execute_one_step()


def run_live(model_type):
    import tkinter as tk
    from tkinter import ttk

    root = tk.Tk()
    root.title("Petri Net Vending Machine")
    canvas = tk.Canvas(root, width=690, height=220, background="white")
    canvas.pack()
    status = ttk.Label(root, padding=8)
    status.pack()
    controls = ttk.Frame(root, padding=8)
    controls.pack()
    engine = ToasterEngine(model_type(), animation_interval=0.1)
    timer = None

    def draw():
        model = engine.model
        canvas.delete("all")
        for points in (
            (68, 61, 121, 61), (187, 61, 260, 105),
            (187, 155, 260, 111), (310, 108, 371, 108),
            (437, 108, 490, 108), (550, 108, 599, 108),
            (283, 190, 283, 129),
        ):
            canvas.create_line(*points, arrow=tk.LAST, width=2, fill="#475569")
        for name, x, y in (("Credit", 154, 61), ("Stock", 154, 155), ("Pending", 404, 108)):
            count = len(model.tokens(name))
            canvas.create_oval(x - 33, y - 33, x + 33, y + 33, fill=("#bbf7d0" if name == "Pending" else "#dbeafe") if count else "white")
            label = f"{name}\n{count}"
            if name == "Pending" and count:
                remaining = max(0, model.tNext - engine.tNow)
                label += f"\n{remaining:.1f} s left"
            canvas.create_text(x, y, text=label, justify="center")
        for name, left, right in (
            ("StartSale", 260, 310),
            ("Deliver", 490, 550),
        ):
            canvas.create_rectangle(left, 87, right, 129, fill="#f1f5f9")
            canvas.create_text((left+right)/2, 108, text=name)
        if "?refund" in model.IP:
            canvas.create_line(154, 28, 240, 28, arrow=tk.LAST, fill="#475569")
            canvas.create_rectangle(240, 12, 305, 44, fill="#f1f5f9")
            canvas.create_text(272, 28, text="Refund")
            canvas.create_line(305, 28, 380, 28, arrow=tk.LAST, fill="#475569")
            canvas.create_text(412, 28, text="!refund")
            canvas.create_text(272, 58, text="?refund")
            canvas.create_line(272, 49, 272, 44, arrow=tk.LAST, fill="#475569")
        canvas.create_text(40, 61, text="?1USD")
        canvas.create_text(283, 201, text="?Coke")
        canvas.create_text(626, 108, text="!Coke")
        detail = ""
        if engine.last_output:
            output = engine.last_output
            detail += f"   Output: {output.port}"
            if output.port == "!refund":
                detail += f" ${output.value}"
        status.configure(text=f"t = {engine.tNow:.1f} s   {detail}")

    def schedule():
        nonlocal timer
        if timer is None and isfinite(engine.model.tNext):
            timer = root.after(100, advance)

    def advance():
        nonlocal timer
        timer = None
        engine.execute_one_step()
        draw()
        schedule()

    def apply(port):
        send(engine, port)
        draw()
        schedule()

    def reset():
        nonlocal engine, timer
        if timer is not None:
            root.after_cancel(timer)
            timer = None
        engine = ToasterEngine(model_type(), animation_interval=0.1)
        draw()

    buttons = [("Insert $1", "?1USD"), ("Coke", "?Coke")]
    if "?refund" in engine.model.IP:
        buttons.append(("Return change", "?refund"))
    for label, port in buttons:
        ttk.Button(controls, text=label, command=lambda p=port: apply(p)).pack(side="left", padx=4)
    ttk.Button(controls, text="Reset", command=reset).pack(side="left", padx=4)
    draw()
    root.mainloop()


def main(model_type=VendingMachine, description=__doc__):
    parser = ArgumentParser(description=description)
    parser.add_argument("--live", action="store_true", help="Open the 2D window (default).")
    parser.parse_args()
    run_live(model_type)


if __name__ == "__main__":
    main()

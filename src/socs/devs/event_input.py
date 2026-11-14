"""Tk controls for sending an external event at the current simulation time."""

from json import JSONDecodeError, loads
from tkinter import StringVar, ttk

from .TESBase import PortValue


def _input_models(model):
    if model.IP:
        yield model
    for child in model.children:
        yield from _input_models(child)


class EventInput(ttk.LabelFrame):
    """Select an input endpoint and submit one external event."""

    def __init__(self, parent, engine, on_applied, *, initial_value=""):
        super().__init__(parent, text="External event (X)", padding=8)
        self.engine = engine
        self.on_applied = on_applied
        self.models = {model.path: model for model in _input_models(engine.model)}
        self.model_path = StringVar(self, next(iter(self.models), ""))
        self.port_name = StringVar(self)
        self.value = StringVar(self, initial_value)
        self.message = StringVar(self)

        ttk.Label(self, text="Model path").grid(row=0, column=0, sticky="w")
        model_entry = ttk.Combobox(self, textvariable=self.model_path, values=tuple(self.models), width=22)
        model_entry.grid(row=1, column=0, padx=(0, 8))
        ttk.Label(self, text="Input port").grid(row=0, column=1, sticky="w")
        self.port_entry = ttk.Combobox(self, textvariable=self.port_name, width=12)
        self.port_entry.grid(row=1, column=1, padx=(0, 8))
        ttk.Label(self, text="Value (JSON)").grid(row=0, column=2, sticky="w")
        ttk.Entry(self, textvariable=self.value, width=16).grid(row=1, column=2, padx=(0, 8))
        ttk.Button(self, text="Apply", command=self.apply).grid(row=1, column=3)
        ttk.Label(self, textvariable=self.message).grid(row=2, column=0, columnspan=4, sticky="w")
        model_entry.bind("<<ComboboxSelected>>", self._update_ports)
        model_entry.bind("<FocusOut>", self._update_ports)
        self._update_ports()

    def _update_ports(self, event=None):
        model = self.models.get(self.model_path.get().strip())
        ports = tuple(model.IP) if model is not None else ()
        self.port_entry.configure(values=ports)
        if self.port_name.get() not in ports:
            self.port_name.set(next(iter(ports), ""))

    def apply(self):
        model = self.models.get(self.model_path.get().strip())
        port = self.port_name.get().strip()
        if model is None or port not in model.IP:
            self.message.set("Select a model path and one of its input ports.")
            return
        text = self.value.get().strip()
        try:
            value = loads(text) if text else None
        except JSONDecodeError:
            self.message.set('Enter JSON, such as 2 or "open", or leave the value blank.')
            return
        self.engine.XBuffer.append((model, PortValue(port, value)))
        self.engine.execute_one_step()
        self.message.set(f"Sent {port} to {model.path} at t={self.engine.tNow:g}")
        self.on_applied()

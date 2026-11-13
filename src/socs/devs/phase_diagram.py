"""Graphviz layout and live Tk rendering for declared DEVSph models."""

from json import dumps
from math import ceil, inf, isfinite
from pathlib import Path
from shlex import split
from shutil import which
from subprocess import run
import tkinter as tk
from tkinter import ttk

from .TESBase import PortValue
from .DEVSph import transition_line_style


class PhaseDiagram(ttk.Frame):
    """Lay out a declared graph once, then update its live state in Tk."""

    PIXELS_PER_INCH = 88
    MARGIN = 16

    def __init__(self, parent, model, *, width=560, height=210, columns=()):
        layout, phases = self._layout(model, columns=columns)
        super().__init__(parent)
        self.model = model
        self.nodes = {}
        self.edges = {}
        self.active = None
        self.transition = None
        if width is None:
            width = ceil(float(layout[0][2]) * self.PIXELS_PER_INCH + 2 * self.MARGIN)
        canvas = self.canvas = tk.Canvas(
            self, width=width, height=height, background="white", highlightthickness=0,
        )
        ttk.Label(self, text=model.path, anchor="center", padding=(0, 4)).grid(
            row=0, column=0, sticky="ew",
        )
        canvas.grid(row=1, column=0, sticky="nsew")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        self._draw_layout(layout, phases, width, height)

    def _draw_layout(self, layout, phases, width, height):
        """Convert the layout into Canvas coordinates and draw it once."""
        canvas = self.canvas
        _, _, graph_width, graph_height = layout[0]
        scale, margin = self.PIXELS_PER_INCH, self.MARGIN
        self.content_width = float(graph_width) * scale + 2 * margin
        self.content_height = float(graph_height) * scale + 2 * margin
        if self.content_width > width:
            scrollbar = ttk.Scrollbar(self, orient="horizontal", command=canvas.xview)
            scrollbar.grid(row=2, column=0, sticky="ew")
            canvas.configure(xscrollcommand=scrollbar.set)
        if self.content_height > height:
            scrollbar = ttk.Scrollbar(self, orient="vertical", command=canvas.yview)
            scrollbar.grid(row=1, column=1, sticky="ns")
            canvas.configure(yscrollcommand=scrollbar.set)
        canvas.configure(scrollregion=(0, 0, self.content_width, self.content_height))
        offset_x = max(0, (width - self.content_width) / 2)
        offset_y = max(0, (height - self.content_height) / 2)

        def point(x, y):
            return (offset_x + margin + float(x) * scale,
                    offset_y + margin + (float(graph_height) - float(y)) * scale)

        for record in layout[1:]:
            if record[0] != "edge":
                continue
            _, source, target, count, *fields = record
            end = 2 * int(count)
            coords = [value for x, y in zip(fields[:end:2], fields[1:end:2]) for value in point(x, y)]
            # Graphviz's plain spline stops before the arrowhead.
            x, y = coords[-2:]
            dx, dy = x - coords[-4], y - coords[-3]
            length = (dx * dx + dy * dy) ** 0.5
            if length:
                coords[-2:] = [x + 9 * dx / length, y + 9 * dy / length]
            style, _ = fields[-2:]
            line = canvas.create_line(
                *coords, smooth="raw", arrow=tk.LAST, width=2, fill="#526173",
                dash={"dashed": (5, 4), "dotted": (1, 3)}.get(style, ()),
            )
            self.edges.setdefault((phases.get(source), phases[target]), []).append(line)
            if len(fields) > end + 2:
                label, x, y = fields[end:end + 3]
                canvas.create_text(*point(x, y), text=label.replace("\\n", "\n"), justify="center")

        for record in layout[1:]:
            if record[0] != "node":
                continue
            _, node, x, y, width, height, *_ = record
            x, y = point(x, y)
            rx, ry = float(width) * scale / 2, float(height) * scale / 2
            if node == "initial":
                canvas.create_oval(x - rx, y - ry, x + rx, y + ry, fill="#526173")
                continue
            phase = phases[node]
            passive = self.model.graph.vertex_attrs(phase).tau == inf
            oval = canvas.create_oval(
                x - rx, y - ry, x + rx, y + ry, width=2, fill="white",
                dash=(5, 4) if passive else (),
            )
            label = canvas.create_text(x, y, text=phase, justify="center")
            self.nodes[phase] = oval, label

    @staticmethod
    def _layout(model, *, columns=()):
        dot = which("dot")
        windows_dot = Path(r"C:\Program Files\Graphviz\bin\dot.exe")
        if dot is None and windows_dot.is_file():
            dot = str(windows_dot)
        if dot is None:
            raise RuntimeError("Live phase diagrams require Graphviz dot on PATH.")
        ids = {phase: f"p{i}" for i, phase in enumerate(model.graph.V)}
        lines = [
            "digraph phases {",
            "rankdir=LR; nodesep=0.40; ranksep=0.55;",
            'node [fontname="Arial", width=1.55, height=0.70];',
            'edge [fontname="Arial", fontsize=11];',
            "initial [shape=point, width=0.08, height=0.08];",
            f"initial -> {ids[model._initial_phase]};",
        ]
        for phase, node in ids.items():
            label = dumps(f"{phase}\nRemaining = 0.00", ensure_ascii=False)
            lines.append(f"{node} [label={label}];")
        for column in columns:
            lines.append("{ rank=same; " + "; ".join(ids[phase] for phase in column) + "; }")
        for edge in model.diagram_edges():
            attrs = edge.attrs
            external = attrs.kind == "external"
            port = attrs.get("port") if external else attrs.get("output")
            label = str(port) if isinstance(port, (str, PortValue)) else attrs.get("label", "")
            labels = [label] if label else []
            for role in ("guard", "action"):
                callback = attrs.get(role)
                if callback is not None:
                    name = getattr(callback, "__name__", type(callback).__name__)
                    labels.append(f"{role}: {name}")
            label = "\n".join(labels)
            style = transition_line_style(attrs)
            lines.append(
                f"{ids[edge.src]} -> {ids[edge.dst]} "
                f'[style={style}, label={dumps(label, ensure_ascii=False)}];'
            )
        result = run(
            [dot, "-Tplain"], input="\n".join((*lines, "}")), encoding="utf-8",
            capture_output=True, check=True,
        )
        return [split(line) for line in result.stdout.splitlines() if line], {
            node: phase for phase, node in ids.items()
        }

    def refresh(self, time, transition=None):
        phase = self.model.phase
        remaining = self.model.tNext - time
        value = f"{max(0, remaining):g}" if isfinite(remaining) else "∞"
        if self.active != phase:
            if self.active is not None:
                oval, label = self.nodes[self.active]
                self.canvas.itemconfigure(oval, fill="white")
                self.canvas.itemconfigure(label, text=self.active)
            oval, _ = self.nodes[phase]
            self.canvas.itemconfigure(oval, fill="#bfdbfe")
            self.active = phase
            x1, _, x2, _ = self.canvas.coords(oval)
            width = self.canvas.winfo_width()
            if x1 < self.canvas.canvasx(0) or x2 > self.canvas.canvasx(width):
                self.canvas.xview_moveto(max(0, (x1 + x2 - width) / 2) / self.content_width)
        _, label = self.nodes[phase]
        self.canvas.itemconfigure(label, text=f"{phase}\nRemaining = {value}")
        if transition != self.transition:
            for line in self.edges.get(self.transition, ()):
                self.canvas.itemconfigure(line, fill="#526173")
            for line in self.edges.get(transition, ()):
                self.canvas.itemconfigure(line, fill="#1d4ed8")
            self.transition = transition

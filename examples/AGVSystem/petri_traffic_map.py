"""Live view of the Petri net subnet for a selected AGV departure point."""

from math import cos, pi, sin

from direct.gui.DirectGui import DirectButton

from .layout import SEGMENTS
from .traffic_map import FREE, RESERVED, TrafficMap, draw_line


class PetriTrafficMap(TrafficMap):
    def __init__(self, parent):
        super().__init__(parent)
        self.layout = self.root.attach_new_node("Layout")
        for child in list(self.root.get_children()):
            if child != self.layout:
                child.reparent_to(self.layout)
        self.net = self.root.attach_new_node("Petri net")
        self.net.hide()
        self.show_net = False
        self.sources = tuple(SEGMENTS)
        self.index = 0
        self.net_signature = None
        self.marking = self.net.attach_new_node("Marking")
        self.toggle = DirectButton(
            parent=self.root, text="Show Petri Net", scale=0.85,
            pos=(0, 0, -17), command=self.toggle_view,
        )
        for label, x, offset in (("<", -12, -1), (">", 12, 1)):
            DirectButton(
                parent=self.net, text=label, scale=1.1, pos=(x, 0, 11),
                command=self.select, extraArgs=[offset],
            )

    def toggle_view(self):
        self.show_net = not self.show_net
        self.layout.hide() if self.show_net else self.layout.show()
        self.net.show() if self.show_net else self.net.hide()
        self.toggle["text"] = "Show Layout" if self.show_net else "Show Petri Net"

    def select(self, offset):
        self.index = (self.index + offset) % len(self.sources)

    def update(self, simulation):
        if not self.show_net:
            super().update(simulation)
            self.dynamic.reparent_to(self.layout)
            return

        controller = simulation.controller
        source = self.sources[self.index]
        segment = SEGMENTS[source]
        places = (
            ("Waiting", -9, 5, "Waiting"),
            (("Free", f"SEG_{source}"), 0, 5, "Segment free"),
            (("Free", segment.target), 9, 5, "Point free"),
            (("Reserved", f"SEG_{source}"), -6, -7, "Segment reserved"),
            (("Reserved", segment.target), 6, -7, "Point reserved"),
        )
        tokens = [controller.tokens(place) for place, *_ in places]
        tokens[0] = tuple(vehicle for point, vehicle in tokens[0] if point == source)
        signature = source, tokens
        if signature == self.net_signature:
            return
        self.net_signature = signature
        self.marking.remove_node()
        self.marking = self.net.attach_new_node("Marking")
        node = self.marking
        self.label(node, "PETRI NET", 0, 14, 1.55)
        self.label(node, f"{source} -> {segment.target}", 0, 11, 1.15)
        for (_, x, y, label), values in zip(places, tokens):
            circle = [(x + 3.5*cos(i*pi/16), y + 2.4*sin(i*pi/16)) for i in range(33)]
            color = RESERVED if values else FREE
            draw_line(node, circle, color, 2)
            self.label(node, label, x, y + 0.45, 0.73)
            self.label(node, ", ".join("permit" if v is True else str(v) for v in values) or "-", x, y - 0.8, 0.8, color)
        draw_line(node, [(-3, -1.4), (3, -1.4), (3, 1.4), (-3, 1.4), (-3, -1.4)], FREE, 2)
        self.label(node, "Grant", 0, -0.32, 1)
        for start, end in (
            ((-7, 3), (-2, 1.4)), ((0, 2.6), (0, 1.4)), ((7, 3), (2, 1.4)),
            ((-2, -1.4), (-5, -4.8)), ((2, -1.4), (5, -4.8)),
        ):
            self.arrow(node, start, end)
        self.label(node, f"Output !{source}   Vehicle ID", 0, -12, 0.95)
        self.label(node, "Clearance returns each permit to its Free place.", 0, -14, 0.82)

    @staticmethod
    def arrow(parent, start, end):
        from math import atan2

        x, y = end
        angle = atan2(y - start[1], x - start[0])
        draw_line(parent, [start, end], FREE, 2)
        tips = [(x - 0.65*cos(angle + offset), y - 0.65*sin(angle + offset)) for offset in (-0.5, 0.5)]
        draw_line(parent, [tips[0], end, tips[1]], FREE, 2)

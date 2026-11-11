"""Top-down read-only traffic display for the Panda3D viewer."""
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import LineSegs, TextNode

from .layout import DWELL, POINTS, ROUTE_DEPTH, ROUTE_WIDTH, SEGMENTS

FREE = (0.45, 0.52, 0.62, 1)
RESERVED = (1, 0.75, 0.23, 1)
OCCUPIED = (0.33, 0.70, 1, 1)
WAITING = (1, 0.45, 0.29, 1)
CHARGING = (0.39, 0.92, 0.60, 1)
TEXT = (0.94, 0.97, 1, 1)
VEHICLE_TAG = {"Tugger": "TUG", "Unit load": "UNIT", "Forklift": "FORK"}


def draw_line(parent, positions, color, thickness, dashed=False):
    lines = LineSegs()
    lines.set_color(*color)
    lines.set_thickness(thickness)
    if dashed:
        for first, second in zip(positions[::2], positions[1::2]):
            lines.move_to(first[0], 0, first[1])
            lines.draw_to(second[0], 0, second[1])
    else:
        x, y = positions[0]
        lines.move_to(x, 0, y)
        for x, y in positions[1:]:
            lines.draw_to(x, 0, y)
    return parent.attach_new_node(lines.create())


def map_xy(x, y):
    return x * 32 / ROUTE_WIDTH, y * 18 / ROUTE_DEPTH


class TrafficMap:
    def __init__(self, parent):
        self.root = parent.attach_new_node("Traffic map")
        self.dynamic = self.root.attach_new_node("Traffic resources")
        self.signature = None
        self.label(self.root, "TRAFFIC CONTROL", 0, 14, 1.55)
        self.label(self.root, "Available", -12, -13, 0.9, FREE)
        self.label(self.root, "Reserved", -4, -13, 0.9, RESERVED)
        self.label(self.root, "Occupied", 4, -13, 0.9, OCCUPIED)
        self.label(self.root, "Waiting", 12, -13, 0.9, WAITING)
        self.label(self.root, "TUG  Tugger     UNIT  Unit load     FORK  Forklift", 0, -15, 0.78)

    @staticmethod
    def label(parent, text, x, y, size=0.88, color=TEXT):
        return OnscreenText(parent=parent, text=text, pos=(x, y), scale=size,
                            align=TextNode.A_center, fg=color, shadow=(0, 0, 0, 1))

    def resize(self, aspect):
        self.root.set_pos(aspect / 2, 0, 0)
        self.root.set_scale(min(aspect / 39, 1 / 19))

    def update(self, simulation):
        state = simulation.controller.state
        reserved, occupied, waiting = (state[key] for key in ("reserved", "occupied", "waiting"))
        charging = {
            point for point in DWELL
            if any(status == f"Charging at {point}" for status in simulation.status.values())
        }
        signature = (dict(reserved), dict(occupied), dict(waiting), charging)
        if self.signature == signature:
            return
        self.signature = signature
        self.dynamic.remove_node()
        self.dynamic = self.root.attach_new_node("Traffic resources")

        for point, segment in SEGMENTS.items():
            resource = f"SEG_{point}"
            owner = occupied.get(resource) or reserved.get(resource)
            dashed = resource in reserved and resource not in occupied
            color = OCCUPIED if resource in occupied else RESERVED if dashed else FREE
            samples = [map_xy(*segment.pose(index / 24)[:2]) for index in range(25)]
            draw_line(self.dynamic, samples, color, 4 if owner else 2, dashed)
            if owner:
                x, y, _ = segment.pose(0.5)
                x, y = map_xy(x, y)
                self.label(self.dynamic, VEHICLE_TAG[owner], 0.88 * x, 0.78 * y - 0.25, 0.78, color)

        for point, position in POINTS.items():
            x, y = map_xy(*position)
            owner = occupied.get(point) or reserved.get(point)
            color = OCCUPIED if point in occupied else RESERVED if owner else FREE
            dot = "[]" if point in reserved and point not in occupied else "+"
            self.label(self.dynamic, dot, x, y - 0.33, 1.18, color)
            label = point
            if point in waiting:
                label += f"  {VEHICLE_TAG[waiting[point]]} WAIT"
                color = WAITING
            elif point in charging:
                label += f"  {VEHICLE_TAG[owner]} CHG"
                color = CHARGING
            elif owner:
                label += f"  {VEHICLE_TAG[owner]}"
            self.label(self.dynamic, label, x, y + (0.88 if y >= 0 else -1.55), 0.76, color)

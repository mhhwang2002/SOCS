"""Shared speed controls for the RMFS viewers."""

from bisect import bisect_right
from itertools import accumulate, pairwise
from math import inf


def bind_speed(viewer, refresh):
    def change(direction):
        if direction > 0:
            viewer.speed = inf if viewer.speed * 2 >= 100 else viewer.speed * 2
        else:
            viewer.speed = 100. if viewer.speed == inf else max(.1, viewer.speed / 2)
        refresh()

    for key, direction in (("u", 1), ("d", -1), ("shift-u", 1), ("shift-d", -1)):
        viewer.accept(key, change, [direction])
        viewer.accept(f"{key}-repeat", change, [direction])


def playback_time(time, elapsed, speed, end, events):
    if elapsed <= 0:
        return time
    if speed == inf:
        index = bisect_right(events, time + 1e-9)
        return events[index] if index < len(events) else end
    return min(end, time + min(.1, elapsed) * speed)


def motion_events(frames):
    events = [frames[0][0]]
    previous_modes = None
    for (_, previous), (time, current) in zip(frames, frames[1:]):
        modes = tuple(
            (waiting, point != old_point, heading != old_heading)
            for (old_point, old_heading, _), (point, heading, waiting) in zip(previous, current)
        )
        if modes != previous_modes:
            events.append(time)
            previous_modes = modes
    if events[-1] != frames[-1][0]:
        events.append(frames[-1][0])
    return events


def speed_label(speed):
    return "Event only" if speed == inf else f"{speed:g}x"


class RouteProgress:
    """Track released segments in a recorded, axis-aligned drive."""

    def __init__(self, routes, frames, clearance):
        self.routes = routes
        self.frames = frames
        self.clearance = clearance
        self.ends = [
            tuple(accumulate(abs(a[0] - b[0]) + abs(a[1] - b[1]) for a, b in pairwise(route)))
            for route in routes
        ]
        self.distance = [0.] * len(routes)
        self.index = 0

    def remaining(self, frame_index):
        while self.index < frame_index:
            previous = self.frames[self.index][1]
            self.index += 1
            current = self.frames[self.index][1]
            for robot, ((a, _, _), (b, _, _)) in enumerate(zip(previous, current)):
                self.distance[robot] += abs(a[0] - b[0]) + abs(a[1] - b[1])
        return tuple(
            () if pose[0] == route[-1] else route[bisect_right(ends, distance - self.clearance):]
            for route, ends, distance, pose in zip(
                self.routes, self.ends, self.distance, self.frames[self.index][1]
            )
        )


def register_selection_color():
    from panda3d.core import TextProperties, TextPropertiesManager

    selected = TextProperties()
    selected.setTextColor(1, 1, 0, 1)
    TextPropertiesManager.getGlobalPtr().setProperties("rmfs-selected", selected)


def highlight(line, selected):
    return f"\1rmfs-selected\1{line}\2" if selected else line

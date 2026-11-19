"""Report only control-point clearance at analytically scheduled times."""

from math import inf

from socs.devs import DEVSph
from socs.devs.TESBase import PortValue

class PSensor(DEVSph):
    def __init__(self, name, *, stop_on_collision=True):
        super().__init__(name, variables={
            "motion_plan": None, "pending_events": [],
            "collision_partners": (),
        })
        self.stop_on_collision = stop_on_collision
        self.add_input_ports("?motion_plan", "?collision")
        self.add_output_ports("!sense", "!Collision")
        self.add_phase("Idle", inf, initial=True)
        self.add_phase("Sensing", lambda m: max(
            0.0, m._variable.pending_events[0][0]-m.tNow,
        ) if m._variable.pending_events else inf)
        self.add_phase("Collided", inf)
        for phase in ("Idle", "Sensing"):
            self.add_external_transition(
                phase, "Sensing", input_port="?motion_plan", reschedule=True,
                action=PSensor.schedule,
            )
            self.add_external_transition(
                phase, "Collided", input_port="?collision", reschedule=True,
                action=PSensor.receive_collision,
            )
        self.add_internal_transition(
            "Sensing", "Sensing", guard=lambda m, e: m.has_later_events(),
            action=PSensor.consume_batch,
        )
        self.add_internal_transition(
            "Sensing", "Idle", guard=lambda m, e: not m.has_later_events(),
            action=PSensor.consume_batch,
        )

    def delta_x(self, event):
        if event is None:
            return False
        if event.port == "?collision" and not self.stop_on_collision:
            return False
        return super().delta_x(event)

    def delta_y(self):
        if self.phase == "Sensing":
            outputs = self.emit_batch()
            super().delta_y()
            return outputs
        return super().delta_y()

    def schedule(self, event):
        plan = event.value
        origin, direction = plan["origin"], plan["direction"]
        profile, half = plan["profile"], plan["half_size"]
        events = []
        for point in plan["points"]:
            delta = tuple(p-o for p, o in zip(point, origin))
            intervals = []
            for offset, axis, radius in zip(delta, direction, half):
                if abs(axis) < 1e-12:
                    if abs(offset) > radius:
                        break
                    intervals.append((-inf, inf))
                else:
                    bounds = sorted(((offset-radius)/axis, (offset+radius)/axis))
                    intervals.append(bounds)
            else:
                entry = max(bounds[0] for bounds in intervals)
                leave = min(bounds[1] for bounds in intervals)
                if leave < 0 or entry > profile.length or entry >= leave:
                    continue
                if leave < profile.length:
                    position = tuple(
                        coordinate + leave*axis
                        for coordinate, axis in zip(origin, direction)
                    )
                    events.append((
                        plan["start_time"]+profile.time_at_distance(leave),
                        position, "leaving",
                    ))
        self._variable.motion_plan = plan
        self._variable.pending_events = sorted(events)

    def has_later_events(self):
        events = self._variable.pending_events
        return events[-1][0] > events[0][0] + 1e-9

    def emit_batch(self):
        return [
            PortValue("!sense", point)
            for time, point, kind in self._variable.pending_events
            if abs(time-self.tNow) < 1e-9
        ]

    def consume_batch(self, event):
        pending = []
        for time, point, kind in self._variable.pending_events:
            if abs(time-self.tNow) >= 1e-9:
                pending.append((time, point, kind))
        self._variable.pending_events = pending

    def cancel(self, event):
        self._variable.pending_events = []

    def receive_collision(self, event):
        self._variable.collision_partners = event.value
        self.cancel(event)

    def report_collision(self):
        return PortValue("!Collision", self._variable.collision_partners)

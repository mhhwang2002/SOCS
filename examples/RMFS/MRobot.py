"""One physical mobile robot, driven by an event-scheduled straight profile."""

from math import atan2, cos, degrees, dist, inf, isfinite, radians, sin

from socs.devs import DEVSph
from socs.devs.TESBase import PortValue

from .GridPath import CONTROL_POINTS
from .motion import EMPTY, LOADED, StraightProfile


class MRobot(DEVSph):
    def __init__(self, name, initial_point, *, lift_up=True, rack_size=(1.5, 1.5),
                 stop_on_collision=True, initial_heading_deg=None):
        if initial_heading_deg is not None and not isfinite(initial_heading_deg):
            raise ValueError("initial_heading_deg must be finite or None.")
        angle = radians(initial_heading_deg or 0.0)
        rack_size = tuple(rack_size)
        if len(rack_size) != 2 or any(
            not isfinite(side) or side <= 0 for side in rack_size
        ):
            raise ValueError("rack_size must contain two positive finite lengths in metres.")
        super().__init__(name, variables={
            "lift_up": lift_up, "rack": "Rack1" if lift_up else None,
            "path": (), "current_segment_index": 0,
            "current_position": CONTROL_POINTS[initial_point],
            "current_dir": (cos(angle), sin(angle)), "profile": None, "start_time": 0.0,
            "direction_initialized": initial_heading_deg is not None,
            "path_entry_turn": False,
            "run_start_index": 0, "run_end_index": 0,
            "turn_start_time": 0.0, "turn_duration": 0.0,
            "turn_angle": 0.0, "turn_start_heading": 0.0,
            "collision_partners": (), "collision_active": False,
        })
        self.rack_size = rack_size
        self.stop_on_collision = stop_on_collision
        self.add_input_ports("?command", "?collision")
        self.add_output_ports(
            "!motion_plan", "!arrived", "!rotation_started", "!rotation_finished",
        )
        self.add_phase("Wait", inf, initial=True)
        self.add_phase("StartMove", 0)
        self.add_phase("MoveTo", lambda m: m.next_waypoint_time() - m.tNow)
        self.add_phase("RotateTo", lambda m: m._variable.turn_duration)
        self.add_phase("Collided", inf)
        self.add_external_transition(
            "Wait", "StartMove", input_port="?command", reschedule=True,
            action=MRobot.prepare_path,
        )
        self.add_internal_transition(
            "StartMove", "MoveTo", guard=lambda m,e: not m._variable.path_entry_turn,
            output=MRobot.motion_plan,
        )
        self.add_internal_transition(
            "StartMove", "RotateTo", guard=lambda m,e: m._variable.path_entry_turn,
            action=MRobot.begin_entry_rotation, output=MRobot.report_entry_rotation,
        )
        self.add_internal_transition(
            "MoveTo", "MoveTo", guard=lambda m, e: m.has_straight_successor(),
            action=MRobot.advance_segment,
        )
        self.add_internal_transition(
            "MoveTo", "RotateTo",
            guard=lambda m, e: not m.is_last_segment() and not m.has_straight_successor(),
            action=MRobot.begin_rotation, output=MRobot.report_rotation_started,
        )
        self.add_internal_transition(
            "RotateTo", "StartMove",
            action=MRobot.finish_rotation, output=MRobot.report_rotation_finished,
        )
        self.add_internal_transition(
            "MoveTo", "Wait", guard=lambda m, e: m.is_last_segment(),
            action=MRobot.finish, output=MRobot.report_arrival,
        )
        for phase in ("Wait", "StartMove", "MoveTo", "RotateTo"):
            self.add_external_transition(
                phase, "Collided", input_port="?collision", reschedule=True,
                action=MRobot.stop,
            )

    def prepare_path(self, event):
        points = tuple(CONTROL_POINTS[point] for point in event.value)
        if len(points) < 2 or points[0] != self._variable.current_position:
            raise ValueError("The path must start at the robot and contain a segment.")
        for start, end in zip(points, points[1:]):
            if (start[0] == end[0]) == (start[1] == end[1]):
                raise ValueError("Path segments must be nonzero and axis aligned.")
        v = self._variable
        v.path = points
        v.current_segment_index = 0
        direction = self.segment_direction(0)
        if not v.direction_initialized:
            v.current_dir = direction
            v.direction_initialized = True
        v.path_entry_turn = any(abs(a-b) > 1e-10 for a,b in zip(v.current_dir,direction))
        if not v.path_entry_turn:
            v.current_dir = direction
            self.prepare_run()

    def report_entry_rotation(self):
        return PortValue("!rotation_started", self._variable.current_position)

    def begin_entry_rotation(self, event):
        self.set_rotation(self.segment_direction(0))

    def delta_x(self, event):
        if event is not None and event.port == "?collision":
            self._variable.collision_active = True
            self._variable.collision_partners = event.value
            if not self.stop_on_collision:
                return False
        elif self.phase != "Collided":
            self._variable.collision_active = False
            self._variable.collision_partners = ()
        if event is None:
            return False
        return super().delta_x(event)

    def segment_direction(self, index):
        start, end = self._variable.path[index:index+2]
        length = dist(start, end)
        return tuple((b-a)/length for a, b in zip(start, end))

    def prepare_run(self):
        v = self._variable
        v.run_start_index = v.current_segment_index
        end = v.current_segment_index+1
        while end < len(v.path)-1 and self.segment_direction(end) == v.current_dir:
            end += 1
        v.run_end_index = end
        v.start_time = self.tNow
        limits = self.motion_limits
        v.profile = StraightProfile(
            dist(v.path[v.run_start_index], v.path[end]), acceleration=limits.acceleration,
            deceleration=limits.deceleration, speed=limits.speed,
        )

    @property
    def motion_limits(self):
        return LOADED if self._variable.lift_up else EMPTY

    def motion_plan(self):
        v = self._variable
        return PortValue("!motion_plan", {
            "origin": v.path[v.run_start_index], "direction": v.current_dir,
            "points": v.path[v.run_start_index:v.run_end_index+1],
            "start_time": v.start_time, "profile": v.profile,
            "half_size": tuple(side/2 for side in self.rack_size),
        })

    def next_waypoint_time(self):
        v = self._variable
        distance = dist(v.path[v.run_start_index], self.get_current_segment_destination())
        return v.start_time + v.profile.time_at_distance(distance)

    def get_current_segment_source(self):
        return self._variable.path[self._variable.current_segment_index]

    def get_current_segment_destination(self):
        return self._variable.path[self._variable.current_segment_index + 1]

    def is_last_segment(self):
        return self._variable.current_segment_index == len(self._variable.path)-2

    def has_straight_successor(self):
        return self._variable.current_segment_index+1 < self._variable.run_end_index

    def begin_rotation(self, event):
        v = self._variable
        v.current_position = self.get_current_segment_destination()
        self.set_rotation(self.segment_direction(v.current_segment_index+1))

    def set_rotation(self, direction):
        v = self._variable
        v.turn_start_time = self.tNow
        v.turn_start_heading = degrees(atan2(v.current_dir[1], v.current_dir[0]))
        target = degrees(atan2(direction[1], direction[0]))
        v.turn_angle = (target-v.turn_start_heading+180) % 360-180
        v.turn_duration = abs(v.turn_angle)/self.motion_limits.yaw_rate

    def finish_rotation(self, event):
        v = self._variable
        if v.path_entry_turn:
            v.path_entry_turn = False
        else:
            v.current_segment_index += 1
        v.current_dir = self.segment_direction(v.current_segment_index)
        self.prepare_run()

    def report_rotation_started(self):
        return PortValue("!rotation_started", self.get_current_segment_destination())

    def report_rotation_finished(self):
        return PortValue("!rotation_finished", self._variable.current_position)

    def direction_at(self, time):
        v = self._variable
        if self.phase != "RotateTo":
            return v.current_dir
        fraction = min(1.0, max(0.0, (time-v.turn_start_time)/v.turn_duration))
        angle = radians(v.turn_start_heading+v.turn_angle*fraction)
        return cos(angle), sin(angle)

    def advance_segment(self, event):
        self._variable.current_position = self.get_current_segment_destination()
        self._variable.current_segment_index += 1

    def finish(self, event):
        self._variable.current_position = self._variable.path[-1]

    def report_arrival(self):
        return PortValue("!arrived", self._variable.path[-1])

    def position_at(self, time):
        v = self._variable
        if self.phase != "MoveTo":
            return v.current_position
        distance = v.profile.distance_at(time-v.start_time)
        return tuple(p + distance*d for p, d in zip(v.path[v.run_start_index], v.current_dir))

    def stop(self, event):
        self._variable.current_position = self.position_at(self.tNow)
        self._variable.current_dir = self.direction_at(self.tNow)


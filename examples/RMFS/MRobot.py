"""One physical mobile robot with event-scheduled translation and rotation."""

from math import atan2, cos, degrees, dist, inf, isfinite, radians, sin

from socs.devs import DEVSph
from socs.devs.TESBase import PortValue

from .GridPath import CONTROL_POINTS
from .motion import EMPTY, LOADED, StraightProfile, ExtendedProfile


# Simulation calibration for 90-degree rest-to-rest turns of 2 s empty and 3 s loaded.
_EMPTY_YAW_SPEED = 0.9758578862150818  # rad/s
_LOADED_YAW_SPEED = 0.5663685541487754  # rad/s
_YAW_ACCELERATION = 2.5  # rad/s², also used for braking


class MRobot(DEVSph):
    def __init__(self, name, initial_point, *, lift_up=True, rack_size=(1.5, 1.5),
                 stop_on_collision=True, initial_heading_deg=None,
                 command_coordinates="grid", control_points=None,
                 lift_duration=1.4, lower_duration=1.4, passage_push=False, clearance_points=(),
                 motion_limits=None):
        if command_coordinates not in ("grid", "metres"):
            raise ValueError("command_coordinates must be 'grid' or 'metres'.")
        if initial_heading_deg is not None and not isfinite(initial_heading_deg):
            raise ValueError("initial_heading_deg must be finite or None.")
        angle = radians(initial_heading_deg or 0.0)
        self.control_points = CONTROL_POINTS if control_points is None else control_points
        rack_size = tuple(rack_size)
        if len(rack_size) != 2 or any(
            not isfinite(side) or side <= 0 for side in rack_size
        ):
            raise ValueError("rack_size must contain two positive finite lengths in metres.")
        super().__init__(name, variables={
            "lift_up": lift_up, "rack": "Rack1" if lift_up else None,
            "operation_rack": None,
            "path": (), "current_segment_index": 0,
            "current_position": self.control_points[initial_point],
            "current_dir": (cos(angle), sin(angle)), "profile": None, "start_time": 0.0,
            "direction_initialized": initial_heading_deg is not None,
            "path_entry_turn": False,
            "orientation_only": False, "orientation_target": None,
            "run_start_index": 0, "run_end_index": 0,
            "turn_start_time": 0.0, "turn_duration": 0.0,
            "turn_angle": 0.0, "turn_start_heading": 0.0, "turn_profile": None,
            "collision_partners": (), "collision_active": False,
            "motion_plan_pending": False,
            "passage_requested_end": None,
            "motion_extension_counts": dict(accelerating=0,cruising=0,braking=0,
                                             deferred_moving=0,rotating=0,starting=0),
        })
        self.rack_size = rack_size
        if any(not isfinite(t) or t <= 0 for t in (lift_duration, lower_duration)):
            raise ValueError("Lift and lower durations must be positive and finite.")
        self.lift_duration, self.lower_duration = lift_duration, lower_duration
        self.command_coordinates = command_coordinates
        self.stop_on_collision = stop_on_collision
        self.passage_push = passage_push
        self.motion_limits_override = motion_limits
        self.clearance_points = tuple(clearance_points)
        self.add_input_ports("?command", "?collision", "?lift", "?lower", "?orient",
                             "?extend_motion")
        self.add_output_ports(
            "!motion_plan", "!arrived", "!rotation_started", "!rotation_finished",
            "!lifted", "!lowered", "!passage_request",
        )
        self.add_phase("Wait", inf, initial=True)
        self.add_phase("StartMove", 0)
        self.add_phase("MoveTo", lambda m: max(0., m.next_motion_time() - m.tNow))
        self.add_phase("RotateTo", lambda m: m._variable.turn_duration)
        self.add_phase("Collided", inf)
        self.add_phase("LiftRack", lift_duration)
        self.add_phase("LowerRack", lower_duration)
        self.add_external_transition("Wait", "LiftRack", input_port="?lift", reschedule=True,
                                     action=MRobot.begin_lift)
        self.add_external_transition("Wait", "LowerRack", input_port="?lower", reschedule=True,
                                     action=MRobot.begin_lower)
        self.add_internal_transition("LiftRack", "Wait", action=MRobot.finish_lift,
                                     output=lambda m: PortValue("!lifted", m._variable.rack))
        self.add_internal_transition("LowerRack", "Wait", action=MRobot.finish_lower,
                                     output=lambda m: PortValue("!lowered", m._variable.operation_rack))
        self.add_external_transition(
            "Wait", "StartMove", input_port="?command", reschedule=True,
            action=MRobot.prepare_path,
        )
        self.add_external_transition(
            'Wait', 'StartMove', input_port='?orient', reschedule=True,
            action=MRobot.prepare_orientation,
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
            "MoveTo", "MoveTo", guard=lambda m, e: m.passage_request_due(),
            action=lambda m, e: setattr(m._variable, 'passage_requested_end', m._variable.path[-1]),
            output=lambda m: PortValue('!passage_request', m._variable.path[-1]),
        )
        self.add_internal_transition(
            "MoveTo", "MoveTo", guard=lambda m, e: not m.passage_request_due() and m.has_straight_successor(),
            action=MRobot.advance_segment,
        )
        self.add_internal_transition(
            "MoveTo", "RotateTo",
            guard=lambda m, e: not m.passage_request_due() and not m.is_last_segment() and not m.has_straight_successor(),
            action=MRobot.begin_rotation, output=MRobot.report_rotation_started,
        )
        self.add_internal_transition(
            "RotateTo", "StartMove",
            guard=lambda m,e: not m._variable.orientation_only,
            action=MRobot.finish_rotation, output=MRobot.report_rotation_finished,
        )
        self.add_internal_transition(
            'RotateTo', 'Wait', guard=lambda m,e: m._variable.orientation_only,
            action=MRobot.finish_orientation, output=MRobot.report_rotation_finished,
        )
        self.add_internal_transition(
            "MoveTo", "Wait", guard=lambda m, e: not m.passage_request_due() and m.is_last_segment(),
            action=MRobot.finish, output=MRobot.report_arrival,
        )
        for phase in ("Wait", "StartMove", "MoveTo", "RotateTo", "LiftRack", "LowerRack"):
            self.add_external_transition(
                phase, "Collided", input_port="?collision", reschedule=True,
                action=MRobot.stop,
            )

    def begin_lift(self, event):
        if self._variable.lift_up:
            raise ValueError("Cannot lift another rack while already loaded.")
        self._variable.rack = event.value

    def finish_lift(self, event):
        self._variable.lift_up = True

    def begin_lower(self, event):
        if not self._variable.lift_up or event.value != self._variable.rack:
            raise ValueError("Lower command must identify the carried rack.")
        self._variable.operation_rack = event.value

    def finish_lower(self, event):
        self._variable.lift_up = False
        self._variable.rack = None

    def prepare_path(self, event):
        self._variable.passage_requested_end = None
        points = (tuple(self.control_points[point] for point in event.value)
                  if self.command_coordinates == "grid" else
                  tuple(tuple(float(x) for x in point) for point in event.value))
        if any(len(p) != 2 or not all(isfinite(x) for x in p) for p in points):
            raise ValueError("Command coordinates must be finite 2D positions.")
        if len(points) < 2 or points[0] != self._variable.current_position:
            raise ValueError("The path must start at the robot and contain a segment.")
        for start, end in zip(points, points[1:]):
            if (start[0] == end[0]) == (start[1] == end[1]):
                raise ValueError("Path segments must be nonzero and axis aligned.")
        v = self._variable
        v.orientation_only = False
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
        self.set_rotation(self._variable.orientation_target if self._variable.orientation_only
                          else self.segment_direction(0))

    def prepare_orientation(self, event):
        direction = tuple(float(x) for x in event.value)
        if len(direction) != 2 or not all(isfinite(x) for x in direction):
            raise ValueError('Orientation must be a finite direction vector.')
        length = dist((0.,0.),direction)
        if length == 0:
            raise ValueError('Orientation direction cannot be zero.')
        v = self._variable
        v.orientation_target = tuple(x/length for x in direction)
        v.orientation_only = True
        v.path_entry_turn = True
        v.direction_initialized = True

    def finish_orientation(self, event):
        v = self._variable
        v.current_dir = v.orientation_target
        v.orientation_only = False
        v.path_entry_turn = False

    def delta_x(self, event):
        if event is not None and event.port == '?extend_motion':
            return self.receive_extension(event.value)
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

    def next_motion_time(self):
        if self._variable.motion_plan_pending:
            return self.tNow
        return min(self.next_waypoint_time(), self.passage_request_time())

    def passage_request_time(self):
        v = self._variable
        if (not self.passage_push or v.run_end_index != len(v.path)-1
                or v.passage_requested_end == v.path[-1]):
            return inf
        return v.start_time + max(0., v.profile.brake_start - 0.1)

    def passage_request_due(self):
        return self.passage_request_time() <= self.tNow + 1e-9

    def delta_y(self):
        v = self._variable
        if self.phase == 'MoveTo' and v.motion_plan_pending:
            v.motion_plan_pending = False
            return self.motion_plan()
        return super().delta_y()

    def receive_extension(self, command):
        """Apply a controller-issued continuation at the current position/speed.

        No request/report handshake or predicted transition time is required.
        PSensor receives the replacement plan in a zero-time output event.
        """
        v = self._variable
        if self.phase not in ('MoveTo','RotateTo','StartMove') or v.orientation_only or not v.path:
            raise ValueError('An extension requires an outstanding movement command.')
        path = tuple(tuple(p) for p in command['path'])
        if any(len(p) != 2 or not all(isfinite(x) for x in p) for p in path):
            raise ValueError('Extension coordinates must be finite 2D positions.')
        if len(path) <= len(v.path) or path[:len(v.path)] != v.path:
            raise ValueError('An extension must preserve the entire commanded path prefix.')
        previous = v.path[-1]
        aligned = True
        for point in path[len(v.path):]:
            length = dist(previous, point)
            if not length or (previous[0] == point[0]) == (previous[1] == point[1]):
                raise ValueError('Extension segments must be nonzero and axis aligned.')
            aligned &= all(abs((b-a)/length-d) <= 1e-8
                           for a,b,d in zip(previous,point,v.current_dir))
            previous = point
        if self.phase != 'MoveTo' or v.run_end_index != len(v.path)-1 or not aligned:
            # Accept the destination without restarting the current run or turn.
            # prepare_run() uses the appended route after the required rotation.
            v.path = path
            kind = {'MoveTo':'deferred_moving','RotateTo':'rotating','StartMove':'starting'}[self.phase]
            v.motion_extension_counts[kind] += 1
            if self.phase == 'StartMove' and not v.path_entry_turn:
                self.prepare_run()
            return False
        limits = self.motion_limits
        elapsed = self.tNow-v.start_time
        kind = ('braking' if elapsed >= v.profile.brake_start-1e-9 else
                'accelerating' if v.profile.velocity_at(elapsed+1e-6)>
                                  v.profile.velocity_at(elapsed)+1e-8 else 'cruising')
        v.profile = ExtendedProfile(v.profile, self.tNow-v.start_time,
            dist(v.path[v.run_start_index],path[-1]),
            limits.acceleration, limits.deceleration, limits.speed)
        v.path = path
        v.run_end_index = len(path)-1
        v.motion_plan_pending = True
        v.motion_extension_counts[kind] += 1
        return True

    def segment_direction(self, index):
        start, end = self._variable.path[index:index+2]
        length = dist(start, end)
        return tuple((b-a)/length for a, b in zip(start, end))

    def prepare_run(self):
        v = self._variable
        v.motion_plan_pending = False
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
        return self.motion_limits_override or (LOADED if self._variable.lift_up else EMPTY)

    def motion_plan(self):
        v = self._variable
        return PortValue("!motion_plan", {
            "origin": v.path[v.run_start_index], "direction": v.current_dir,
            "points": v.path[v.run_start_index:v.run_end_index+1] + self.clearance_points,
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
        angle = radians(abs(v.turn_angle))
        yaw_speed = _LOADED_YAW_SPEED if v.lift_up else _EMPTY_YAW_SPEED
        v.turn_profile = (StraightProfile(angle, _YAW_ACCELERATION, _YAW_ACCELERATION, yaw_speed)
                          if angle else None)
        v.turn_duration = v.turn_profile.duration if v.turn_profile else 0.0

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
        angle = radians(v.turn_start_heading)
        if v.turn_profile is not None:
            offset = v.turn_profile.distance_at(time-v.turn_start_time)
            angle += -offset if v.turn_angle < 0 else offset
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


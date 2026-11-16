"""Single-order robotic mobile fulfillment system."""

from dataclasses import dataclass
from itertools import pairwise
from math import dist, inf
from random import Random

from socs.devs import CoupledDEVS, DEVSph
from socs.devs.TESBase import PortValue

from .motion import MissionMotion
from .traffic_layout import CLUSTERS, HORIZONTAL_LANES, RACK_POSITIONS, STANDBY, STATION_APPROACHES, WORKSTATIONS, shortest_route

RACKS_PER_CLUSTER = 2 * 6
BINS_PER_RACK = 8



@dataclass(frozen=True)
class Assignment:
    rack: tuple[int, int, int]
    workstation: str

    @property
    def rack_id(self):
        cluster, row, col = self.rack
        return f"C{cluster+1:02} R{row+1:02}-C{col+1:02}"


class OrderAllocator:
    """Keep cumulative workstation assignments across orders."""

    def __init__(self, seed=None):
        self.random = Random(seed)
        self.counts = dict.fromkeys(STATION_APPROACHES, 0)

    def assign(self):
        minimum = min(self.counts.values())
        eligible = [name for name, count in self.counts.items() if count == minimum]
        workstation = self.random.choice(eligible)
        rack = self.random.choice(tuple(RACK_POSITIONS))
        self.counts[workstation] += 1
        return Assignment(rack, workstation)



@dataclass(frozen=True)
class RobotStatus:
    name: str
    position: tuple[float, float]
    available: bool = True


DEMO_FLEET = (
    RobotStatus("MiR600-01", (2, HORIZONTAL_LANES[3])),
    RobotStatus("MiR600-02", STANDBY),
    RobotStatus("MiR600-03", (40, HORIZONTAL_LANES[3])),
)


def select_robot(fleet, slot):
    candidates = [
        (robot, shortest_route(robot.position, slot))
        for robot in fleet
        if robot.available
    ]
    if not candidates:
        raise ValueError("No available robot for the assigned rack.")
    return min(
        candidates,
        key=lambda candidate: (
            sum(dist(a, b) for a, b in pairwise(candidate[1])),
            candidate[0].name,
        ),
    )


class Controller(DEVSph):
    """Dispatch the demo order and wait for the robot to return its rack."""

    def __init__(self, assignment, dispatch_time=0):
        super().__init__("Controller")
        self.add_input_ports("?returned")
        self.add_output_ports("!dispatch", "!complete")
        self.add_phase("Dispatch", tau=dispatch_time, initial=True)
        self.add_phase("AwaitReturn", tau=inf)
        self.add_phase("ReportCompletion", tau=0)
        self.add_phase("Done", tau=inf)
        self.add_internal_transition("Dispatch", "AwaitReturn", output=PortValue("!dispatch", assignment))
        self.add_external_transition(
            "AwaitReturn", "ReportCompletion", input_port="?returned", reschedule=True
        )
        self.add_internal_transition("ReportCompletion", "Done", output="!complete")


class Robot(DEVSph):
    """Execute one assigned rack transport and return."""

    def __init__(self, assignment, motion, name):
        super().__init__(name)
        self.add_input_ports("?dispatch")
        self.add_input_ports("?collision")
        self.collided_with = ()
        self.blocked_robots = ()
        self.add_output_ports("!progress", "!returned")
        self.add_phase("Idle", tau=inf, initial=True)
        self.add_phase("Done", tau=inf)
        self.add_phase("Collided", tau=inf)
        phases = (
            ("TravelToRack", motion.travel_times["TravelToRack"], f"Arrived at {assignment.rack_id}"),
            ("LiftRack", 1.4, "Rack lifted"),
            ("TravelToWorkstation", motion.travel_times["TravelToWorkstation"], f"Arrived at {assignment.workstation}"),
            ("PickBin", 3.0, f"Order shipped at {assignment.workstation}"),
            ("ReturnRack", motion.travel_times["ReturnRack"], "Arrived at original rack slot"),
            ("LowerRack", 1.4, "Rack returned to original slot"),
            ("ReturnToStandby", motion.travel_times["ReturnToStandby"], "Robot at standby"),
        )
        for phase, duration, _ in phases:
            self.add_phase(phase, tau=duration)
        self.add_external_transition(
            "Idle", phases[0][0], input_port="?dispatch", reschedule=True
        )
        for current, following in zip(phases, phases[1:]):
            phase, _, milestone = current
            self.add_internal_transition(
                phase, following[0], output=PortValue("!progress", milestone)
            )
        self.add_internal_transition(
            phases[-1][0], "Done", output="!returned"
        )

    def delta_x(self, x):
        if x is not None and x.port == "?collision":
            self.collided_with = tuple(x.value)
            self.phase = "Collided"
            return True
        if x is not None and x.port == "?dispatch" and self.name in self.blocked_robots:
            return False
        return super().delta_x(x)


class RMFS(CoupledDEVS):
    """Connect dispatch and completion signals between controller and robot."""

    def __init__(self, assignment=None, fleet=DEMO_FLEET, *, name="RMFS", dispatch_time=0, approach=None, motion=None):
        super().__init__(name)
        self.assignment = assignment if assignment is not None else OrderAllocator().assign()
        self.slot = RACK_POSITIONS[self.assignment.rack]
        self.station = STATION_APPROACHES[self.assignment.workstation]
        if approach is None:
            self.selected_robot, approach = select_robot(fleet, self.slot)
        else:
            self.selected_robot, = fleet
        self.motion = motion if motion is not None else MissionMotion(self.slot, self.station, approach)
        self.controller = Controller(self.assignment, dispatch_time)
        self.robot = Robot(self.assignment, self.motion, self.selected_robot.name)
        self.add_components(self.controller, self.robot)
        self.add_output_ports("!progress", "!complete")
        self.add_couples(
            ((self.controller, "!dispatch"), (self.robot, "?dispatch")),
            ((self.robot, "!returned"), (self.controller, "?returned")),
            ((self.robot, "!progress"), (self, "!progress")),
            ((self.controller, "!complete"), (self, "!complete")),
        )






"""Coupled DEVS model for the RMFS example."""

from math import inf

from socs.devs import CoupledDEVS, DEVSph
from socs.devs.TESBase import PortValue

from .MRobot import MRobot
from .PSensor import PSensor


class RMFS(CoupledDEVS):
    def __init__(self, name="RMFS", *, collision_check=False):
        super().__init__(name)
        self.collision_check = collision_check
        self.aabbs = {}
        self.active_contacts = set()
        self.collision_history = []

    def set_init_total_state(self, t):
        super().set_init_total_state(t)
        self.aabbs = {}
        self.active_contacts = set()
        self.collision_history = []

    def Delta_x(self, x, t):
        rescheduled = super().Delta_x(x, t)
        if not self.collision_check:
            return rescheduled
        return self.check_collision(t) or bool(rescheduled)

    def check_collision(self, t):
        """Refresh fixed rack footprints and sweep along X to find contact pairs."""
        robots = {model.name: model for model in self.M if isinstance(model, MRobot)}
        self.aabbs = {}
        for name, robot in robots.items():
            x, y = robot.position_at(t)
            half_x, half_y = (side/2 for side in robot.rack_size)
            self.aabbs[name] = ((x-half_x,y-half_y), (x+half_x,y+half_y))
        boxes = sorted(self.aabbs.items(), key=lambda item: item[1][0][0])
        contacts = set()
        for index, (name, (lower,upper)) in enumerate(boxes):
            for other, (other_lower,other_upper) in boxes[index+1:]:
                if other_lower[0] > upper[0]:
                    break
                if lower[1] <= other_upper[1] and other_lower[1] <= upper[1]:
                    contacts.add(tuple(sorted((name,other))))
        previous_contacts = self.active_contacts
        new_contacts = contacts-previous_contacts
        self.active_contacts = contacts
        affected = {name for pair in contacts | previous_contacts for name in pair}
        for pair in sorted(new_contacts):
            self.collision_history.append((t,pair))
            print(f"Collision {pair} at={t}")
        rescheduled = False
        for name in sorted(affected):
            robot = robots[name]
            partners = tuple(sorted(
                other for pair in contacts if name in pair
                for other in pair if other != name
            ))
            event = PortValue("?collision", partners) if partners else None
            # super().Delta_x has already advanced each child's clock to t.
            # The atomic hook must explicitly receive None on clearance:
            # the framework's Delta_x(None, t) does not call delta_x(None).
            if event is None:
                rescheduled = robot.delta_x(None) or rescheduled
            elif robot.phase != "Collided":
                rescheduled = robot.Delta_x(event,t) or rescheduled
            for sensor, port in self.Z.get((robot,"!motion_plan"), ()):
                if isinstance(sensor,PSensor):
                    if event is None:
                        rescheduled = sensor.delta_x(None) or rescheduled
                    elif sensor.phase != "Collided":
                        rescheduled = sensor.Delta_x(event,t) or rescheduled
        self.update_tn()
        return rescheduled

    def get_snapshot(self):
        return {
            "active_contacts": tuple(sorted(self.active_contacts)),
            "collision_history": tuple(self.collision_history),
        }

    def set_snapshot(self, snapshot):
        self.active_contacts = set(snapshot["active_contacts"])
        self.collision_history = list(snapshot["collision_history"])
        self.aabbs = {}


class Controller(DEVSph):
    def __init__(self, robot_count, order_count):
        super().__init__("Controller", variables={"pending_orders": []})
        self.order_count = order_count
        self.add_input_ports("?report", "?orders")
        for number in range(1, robot_count + 1):
            self.add_output_ports(f"!command_{number}", f"!grant_{number}")
        self.add_phase("Idle", tau=inf, initial=True)
        self.add_external_transition("Idle", "Idle", input_port="?orders",
                                     reschedule=False, action=Controller.receive_orders)

    def receive_orders(self, event):
        self._variable["pending_orders"].extend(event.value)


class Robot(DEVSph):
    def __init__(self, number):
        super().__init__(f"Robot{number}")
        self.number = number
        self.add_input_ports("?command", "?grant")
        self.add_output_ports("!report")
        self.add_phase("Idle", tau=inf, initial=True)

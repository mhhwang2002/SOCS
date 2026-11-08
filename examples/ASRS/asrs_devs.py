"""DEVS model for a single-aisle automated storage and retrieval system."""

from __future__ import annotations

from math import sqrt

from socs.devs.CoupledDEVS import CoupledDEVS
from socs.devs.DEVSInterfaces import PortValue
from socs.devs.FDDEVS import FDDEVS


CELL_WIDTH = 1.0
LEVEL_PITCH = 0.75
RACK_COLUMNS = 15
RACK_LEVELS = 6
RACK_DEPTH = 1.0
AISLE_Y = -0.75
MAX_FORK_REACH = 1.25
UNLOAD_REACH = 0.5
UNLOAD_Y = AISLE_Y + UNLOAD_REACH
LOAD_HEIGHT = 0.42
LOAD_CENTER_OFFSET = 0.27
# Cell (0, 0) is centered at x=0.5 m; cell (-1, 0) is outside the rack.
HOME_X = -CELL_WIDTH / 2.0
HOME_Z = LEVEL_PITCH / 2.0
UNLOAD_STATION_XS = (HOME_X, RACK_COLUMNS * CELL_WIDTH + CELL_WIDTH / 2.0)
BOX_COLORS = {
    "red": (0.88, 0.08, 0.07, 1.0),
    "blue": (0.08, 0.24, 0.90, 1.0),
    "green": (0.08, 0.68, 0.20, 1.0),
    "yellow": (0.95, 0.78, 0.08, 1.0),
    "white": (0.88, 0.91, 0.92, 1.0),
}


class ASRSCell(FDDEVS):
    """Passive storage cell whose occupancy changes only on crane input."""

    def __init__(self, bay: int, level: int, occupied: bool, box_id=None, box_color=None):
        super().__init__(f"Cell-B{bay:02d}-L{level}")
        self.bay = bay
        self.level = level
        self.initially_occupied = bool(occupied)
        self.occupied = self.initially_occupied
        self.box_pending = False
        self.status_pending = False
        self.box_id = box_id or f"Box-{self.name}"
        self.box_color = box_color
        self.addInputPorts("?crane")
        self.addOutputPorts("!box", "!status")

    def delta_0(self):
        self.occupied = self.initially_occupied
        self.box_pending = False
        self.status_pending = True

    def tau(self):
        return 0.0 if self.box_pending or self.status_pending else float("inf")

    def delta_x(self, event):
        if not isinstance(event, PortValue):
            return False
        request = event.value
        if not isinstance(request, dict) or request.get("cell") != self.name:
            return False
        if request.get("operation") != "retrieve":
            raise ValueError(f"Unsupported cell operation: {request.get('operation')!r}")
        if not self.occupied:
            raise ValueError(f"Cannot retrieve from empty cell {self.name}.")
        self.box_pending = True
        return True

    def delta_y(self):
        if self.status_pending:
            self.status_pending = False
            return PortValue(
                self.OP["!status"],
                {"cell": self.name, "occupied": self.occupied,
                 "bay": self.bay, "level": self.level,
                 "box_id": self.box_id, "color": self.box_color},
            )
        if not self.box_pending:
            return None
        self.box_pending = False
        self.occupied = False
        return PortValue(
            self.OP["!box"],
            {"cell": self.name, "box": self.box_id, "color": self.box_color},
        )

    def get_state(self):
        return self.occupied, self.box_pending, self.status_pending

    def set_state(self, state):
        self.occupied, self.box_pending, self.status_pending = state


class ASRSController(FDDEVS):
    """Collect cell status, wait for startup, then dispatch retrieval jobs."""

    STARTUP_DELAY = 10.0

    def __init__(
        self,
        cell_names,
        retrieval_plan=None,
        order_penalties=None,
        requested_order_items=None,
    ):
        super().__init__("Controller")
        self.cell_names = tuple(cell_names)
        self._initial_retrieval_plan = (
            None if retrieval_plan is None
            else tuple(dict(task) for task in retrieval_plan)
        )
        self.order_penalties = dict(order_penalties or {})
        self._initial_requested_order_items = dict(requested_order_items or {})
        self.addInputPorts("?cell_status", "?crane_idle", "?box_out")
        self.addOutputPorts("!command")
        self.delta_0()

    def delta_0(self):
        self.cell_status = {}
        self.occupied_cells = []
        self.pending_tasks = (
            [] if self._initial_retrieval_plan is None
            else [dict(task) for task in self._initial_retrieval_plan]
        )
        self.assigned_cell = None
        self.completed_order_times = {}
        self.remaining_order_items = {}
        self.requested_order_items = {}
        self.delivered_products_by_order = {}
        self.product_colors = {}
        if self._initial_requested_order_items:
            self.requested_order_items = dict(self._initial_requested_order_items)
            self.delivered_products_by_order = {
                order_id: [] for order_id in self.requested_order_items
            }
        if self._initial_retrieval_plan is not None:
            for task in self._initial_retrieval_plan:
                order_id = task["order_id"]
                self.remaining_order_items[order_id] = (
                    self.remaining_order_items.get(order_id, 0) + 1
                )
                self.delivered_products_by_order.setdefault(order_id, [])
                product_id = task.get("product_id")
                if product_id is not None:
                    self.product_colors[product_id] = task.get("color")
            if not self.requested_order_items:
                fulfillable_by_order = {
                    order_id: count
                    for order_id, count in self.remaining_order_items.items()
                }
                self.requested_order_items = {
                    order_id: count + self.order_penalties.get(order_id, 0)
                    for order_id, count in fulfillable_by_order.items()
                }
        self.crane_idle = False
        self.phase = "collecting_status"

    def tau(self):
        return 0.0 if self.phase == "dispatch" else (
            self.STARTUP_DELAY if self.phase == "startup_delay" else float("inf")
        )

    def delta_x(self, event):
        if not isinstance(event, PortValue):
            return False
        if event.port is self.IP["?cell_status"]:
            status = event.value
            if not isinstance(status, dict) or status.get("cell") not in self.cell_names:
                return False
            self.cell_status[status["cell"]] = bool(status["occupied"])
            if len(self.cell_status) == len(self.cell_names):
                self.occupied_cells = [
                    name for name in self.cell_names if self.cell_status[name]
                ]
                self.phase = "startup_delay"
                return True
            return False

        if event.port is self.IP["?box_out"]:
            box_out = event.value
            if not isinstance(box_out, dict) or box_out.get("order_id") not in self.remaining_order_items:
                return False
            order_id = box_out["order_id"]
            box_id = box_out.get("box")
            if box_id is not None:
                self.delivered_products_by_order.setdefault(order_id, []).append(box_id)
            self.remaining_order_items[order_id] -= 1
            if self.remaining_order_items[order_id] == 0:
                self.completed_order_times[order_id] = box_out["completed_at"]
            return True

        if event.port is self.IP["?crane_idle"]:
            self.crane_idle = True
            if self.phase == "awaiting_crane":
                has_work = (
                    bool(self.pending_tasks)
                    if self._initial_retrieval_plan is not None
                    else bool(self.occupied_cells)
                )
                self.phase = "dispatch" if has_work else "completed"
                return True
            if self.phase == "ready":
                has_work = (
                    bool(self.pending_tasks)
                    if self._initial_retrieval_plan is not None
                    else bool(self.occupied_cells)
                )
                self.phase = "dispatch" if has_work else "completed"
                return True
        return False

    def delta_y(self):
        if self.phase == "startup_delay":
            if self.crane_idle:
                has_work = (
                    bool(self.pending_tasks)
                    if self._initial_retrieval_plan is not None
                    else bool(self.occupied_cells)
                )
                self.phase = "dispatch" if has_work else "completed"
            else:
                self.phase = "ready"
            return None
        if self.phase == "dispatch":
            has_work = (
                bool(self.pending_tasks)
                if self._initial_retrieval_plan is not None
                else bool(self.occupied_cells)
            )
            if not self.crane_idle or not has_work:
                self.phase = "completed" if not has_work else "ready"
                return None
            if self._initial_retrieval_plan is None:
                self.assigned_cell = self.occupied_cells.pop(0)
                command = {"cell": self.assigned_cell, "operation": "retrieve"}
            else:
                task = self.pending_tasks.pop(0)
                self.assigned_cell = task["cell"]
                if self.assigned_cell not in self.occupied_cells:
                    raise ValueError(f"Scheduled cell {self.assigned_cell} is not occupied.")
                self.occupied_cells.remove(self.assigned_cell)
                command = {**task, "operation": "retrieve"}
            self.crane_idle = False
            self.phase = "awaiting_crane"
            return PortValue(self.OP["!command"], command)
        return None

    def get_state(self):
        return (
            tuple(self.cell_status.items()), tuple(self.occupied_cells),
            tuple(tuple(task.items()) for task in self.pending_tasks),
            self.assigned_cell, self.crane_idle, self.phase,
            tuple(self.completed_order_times.items()),
            tuple(self.remaining_order_items.items()),
            tuple(
                (order_id, tuple(product_ids))
                for order_id, product_ids in self.delivered_products_by_order.items()
            ),
        )

    def set_state(self, state):
        (
            statuses, occupied, tasks, self.assigned_cell, self.crane_idle,
            self.phase, completed, remaining, delivered,
        ) = state
        self.cell_status = dict(statuses)
        self.occupied_cells = list(occupied)
        self.pending_tasks = [dict(task) for task in tasks]
        self.completed_order_times = dict(completed)
        self.remaining_order_items = dict(remaining)
        self.delivered_products_by_order = {
            order_id: list(product_ids) for order_id, product_ids in delivered
        }


class Crane(FDDEVS):
    """Single-mast crane with X/Z positioning and a Y-axis telescopic fork."""

    x_max_speed = 2.0
    x_acceleration = 1.0
    x_deceleration = 1.0
    z_max_speed = 2.0
    z_acceleration = 1.0
    z_deceleration = 1.0
    fork_max_speed = 2.0
    fork_acceleration = 1.0
    fork_deceleration = 1.0

    def __init__(self, target_bay=None, target_level=None, operation="retrieve"):
        super().__init__("Crane")
        if target_bay is not None and not 1 <= target_bay <= RACK_COLUMNS:
            raise ValueError("target_bay is outside the rack")
        if target_level is not None and not 1 <= target_level <= RACK_LEVELS:
            raise ValueError("target_level is outside the rack")
        if operation != "retrieve":
            raise ValueError("This crane cycle retrieves a box from a rack cell.")

        self.target_bay = target_bay
        self.target_level = target_level
        self.operation = operation
        self.target_cell = None
        self.target_station = 0
        self.target_order_id = None
        self.box_color = None
        self.addInputPorts("?box", "?command")
        self.addOutputPorts("!retrieve_request", "!idle", "!box_out")
        self.x = HOME_X
        self.y = AISLE_Y
        self.z = HOME_Z
        self.fork_lift = 0.0
        self.has_load = False
        self.box_id = None
        self.phase = "notify_idle"
        self._moves = []
        self._move_index = 0
        self._profiles = {}
        self._move_start_time = 0.0
        self._notify_time = float("inf")

    def delta_0(self):
        self.x = HOME_X
        self.y = AISLE_Y
        self.z = HOME_Z
        self.fork_lift = 0.0
        self.has_load = False
        self.box_id = None
        self.box_color = None
        self.target_bay = None
        self.target_level = None
        self.target_cell = None
        self.target_station = 0
        self.target_order_id = None
        self._moves = []
        self._move_index = 0
        self.phase = "notify_idle"
        self._notify_time = self.t_last_update

    def tau(self):
        if self.phase in {"notify_idle", "request_box", "send_box_out"}:
            return max(0.0, self._notify_time - self.timeLast())
        if self._profiles:
            elapsed = max(0.0, self.timeLast() - self._move_start_time)
            next_boundaries = [
                profile["boundaries"][profile["segment_index"]]
                for profile in self._profiles.values()
            ]
            return max(0.0, min(next_boundaries) - elapsed)
        return float("inf")

    def delta_x(self, event):
        if not isinstance(event, PortValue):
            return False
        if event.port is self.IP["?command"]:
            command = event.value
            if self.phase != "idle" or not isinstance(command, dict):
                return False
            cell_name = command.get("cell")
            if not isinstance(cell_name, str):
                return False
            try:
                bay = int(cell_name.split("-B", 1)[1].split("-", 1)[0])
                level = int(cell_name.rsplit("-L", 1)[1])
            except (IndexError, ValueError):
                return False
            if not (1 <= bay <= RACK_COLUMNS and 1 <= level <= RACK_LEVELS):
                return False
            self.target_bay = bay
            self.target_level = level
            self.target_cell = cell_name
            station = command.get("station", 0)
            if station not in (0, 1):
                return False
            self.target_station = station
            self.target_order_id = command.get("order_id")
            self._start_retrieval(self._external_event_time)
            return True
        if event.port is not self.IP["?box"]:
            return False
        transfer = event.value
        if (
            self.phase != "waiting_for_box"
            or not isinstance(transfer, dict)
            or transfer.get("cell") != self.target_cell
        ):
            return False
        self.box_id = transfer.get("box")
        self.box_color = transfer.get("color")
        self.has_load = True
        self._begin_next_move(self._external_event_time)
        return True

    def Delta_x(self, event, t):
        self._external_event_time = t
        try:
            return super().Delta_x(event, t)
        finally:
            del self._external_event_time

    def delta_y(self):
        now = self.timeNext()
        if self.phase == "notify_idle":
            self.phase = "idle"
            self._notify_time = float("inf")
            idle_info = {
                "crane": self.name,
                "cell": self.target_cell,
                "order_id": self.target_order_id,
                "completed_at": now,
            }
            self.target_order_id = None
            return PortValue(self.OP["!idle"], idle_info)
        if self.phase == "request_box":
            self._move_index += 1
            self.phase = "waiting_for_box"
            self._profiles = {}
            return PortValue(
                self.OP["!retrieve_request"],
                {"cell": self.target_cell, "operation": self.operation},
            )
        if self.phase == "send_box_out":
            box_out = PortValue(
                self.OP["!box_out"],
                {
                    "box": self.box_id,
                    "order_id": self.target_order_id,
                    "completed_at": now,
                    "station": self.target_station,
                },
            )
            self.has_load = False
            self.box_id = None
            self.box_color = None
            self._move_index += 1
            self._begin_next_move(now)
            return box_out

        elapsed = max(0.0, now - self._move_start_time)
        completed_axes = []
        for axis, profile in self._profiles.items():
            while (
                profile["segment_index"] < len(profile["boundaries"])
                and profile["boundaries"][profile["segment_index"]]
                <= elapsed + 1e-10
            ):
                profile["segment_index"] += 1
            if profile["segment_index"] >= len(profile["boundaries"]):
                self._set_axis(axis, profile["target"])
                completed_axes.append(axis)
        for axis in completed_axes:
            del self._profiles[axis]

        if not self._profiles:
            self._move_index += 1
            self._begin_next_move(now)
        return None

    def delta_cont(self, t):
        if not self._profiles:
            return False
        elapsed = max(0.0, t - self._move_start_time)
        for axis, profile in self._profiles.items():
            move_time = min(elapsed, profile["total_time"])
            accel_time = profile["accel_time"]
            cruise_time = profile["cruise_time"]
            if move_time <= accel_time:
                distance = 0.5 * profile["acceleration"] * move_time**2
            elif move_time <= accel_time + cruise_time:
                distance = profile["accel_distance"] + profile["peak_speed"] * (
                    move_time - accel_time
                )
            else:
                decel_elapsed = min(
                    move_time - accel_time - cruise_time,
                    profile["decel_time"],
                )
                distance = (
                    profile["accel_distance"]
                    + profile["cruise_distance"]
                    + profile["peak_speed"] * decel_elapsed
                    - 0.5 * profile["deceleration"] * decel_elapsed**2
                )
            distance = min(distance, profile["distance"])
            self._set_axis(
                axis,
                profile["start"] + profile["direction"] * distance,
            )
        return False

    def _begin_next_move(self, t):
        self._profiles = {}
        while self._move_index < len(self._moves):
            move = self._moves[self._move_index]
            action = next(
                (key for key in ("request_box", "send_box_out") if key in move),
                None,
            )
            if action is not None:
                self.phase = action
                self._notify_time = t + 1e-9
                return

            self._move_start_time = t
            for axis, target in move.items():
                profile = self._make_profile(axis, target)
                if profile is not None:
                    self._profiles[axis] = profile
            if self._profiles:
                self.phase = "moving"
                return
            self._move_index += 1

        self.phase = "notify_idle"
        self._notify_time = t

    def _start_retrieval(self, t):
        target_x = (self.target_bay - 0.5) * CELL_WIDTH
        target_load_center = (self.target_level - 1) * LEVEL_PITCH + LOAD_CENTER_OFFSET
        target_z = target_load_center - LOAD_HEIGHT / 2.0 + 0.08
        self.has_load = False
        self.box_id = None
        self._moves = [
            {"X": target_x, "Z": target_z},
            {"Y": AISLE_Y + MAX_FORK_REACH},
            {"request_box": True},
            {"fork_lift": 0.12},
            {"Y": AISLE_Y},
            {"X": UNLOAD_STATION_XS[self.target_station], "Z": HOME_Z},
            {"Y": UNLOAD_Y},
            {"send_box_out": True},
            {"fork_lift": 0.0},
        ]
        self._move_index = 0
        self._begin_next_move(t)

    def _make_profile(self, axis, target):
        start = self._get_axis(axis)
        distance = abs(target - start)
        if distance <= 1e-12:
            self._set_axis(axis, target)
            return None

        if axis == "X":
            acceleration, deceleration, max_speed = (
                self.x_acceleration,
                self.x_deceleration,
                self.x_max_speed,
            )
        elif axis == "Z":
            acceleration, deceleration, max_speed = (
                self.z_acceleration,
                self.z_deceleration,
                self.z_max_speed,
            )
        else:
            acceleration, deceleration, max_speed = (
                self.fork_acceleration,
                self.fork_deceleration,
                self.fork_max_speed,
            )

        peak_speed = min(
            max_speed,
            sqrt(2.0 * distance * acceleration * deceleration / (acceleration + deceleration)),
        )
        accel_time = peak_speed / acceleration
        decel_time = peak_speed / deceleration
        accel_distance = 0.5 * acceleration * accel_time**2
        decel_distance = 0.5 * deceleration * decel_time**2
        cruise_distance = max(0.0, distance - accel_distance - decel_distance)
        cruise_time = cruise_distance / peak_speed

        # Skip a zero-length cruise phase; all boundaries are relative to the
        # common start time for this group of simultaneous axis moves.
        durations = [accel_time]
        if cruise_time > 1e-12:
            durations.append(cruise_time)
        durations.append(decel_time)
        boundaries = []
        total = 0.0
        for duration in durations:
            total += duration
            boundaries.append(total)

        return {
            "start": start,
            "target": target,
            "distance": distance,
            "direction": 1.0 if target >= start else -1.0,
            "acceleration": acceleration,
            "deceleration": deceleration,
            "peak_speed": peak_speed,
            "accel_time": accel_time,
            "cruise_time": cruise_time,
            "decel_time": decel_time,
            "accel_distance": accel_distance,
            "cruise_distance": cruise_distance,
            "total_time": total,
            "boundaries": boundaries,
            "segment_index": 0,
        }

    def _get_axis(self, axis):
        return getattr(self, axis.lower())

    def _set_axis(self, axis, value):
        setattr(self, axis.lower(), value)

    def get_state(self):
        return (
            self.x,
            self.y,
            self.z,
            self.fork_lift,
            self.has_load,
            self.box_id,
            self.phase,
            tuple(tuple(move.items()) for move in self._moves),
            self._move_index,
            tuple(
                (axis, tuple(profile.items()))
                for axis, profile in self._profiles.items()
            ),
            self._move_start_time,
            self._notify_time,
            self.target_bay,
            self.target_level,
            self.target_cell,
            self.target_station,
            self.target_order_id,
            self.box_color,
        )

    def set_state(self, state):
        (
            self.x,
            self.y,
            self.z,
            self.fork_lift,
            self.has_load,
            self.box_id,
            self.phase,
            moves,
            self._move_index,
            profiles,
            self._move_start_time,
            self._notify_time,
            self.target_bay,
            self.target_level,
            self.target_cell,
            self.target_station,
            self.target_order_id,
            self.box_color,
        ) = state
        self._moves = [dict(move) for move in moves]
        self._profiles = {
            axis: dict(profile) for axis, profile in profiles
        }


def make_asrs_model(
    columns=RACK_COLUMNS,
    levels=RACK_LEVELS,
    target_bay=None,
    target_level=None,
    operation="retrieve",
    inventory=None,
    retrieval_plan=None,
    order_penalties=None,
    requested_order_items=None,
):
    """Create an AS/RS rack coordinated by a controller and one crane."""
    if columns != RACK_COLUMNS or levels != RACK_LEVELS:
        raise ValueError("This example model uses the sketched 15 x 6 rack.")
    if operation != "retrieve":
        raise ValueError("This ASRS cycle supports retrieval to the unload station.")
    model = CoupledDEVS("ASRS")
    cells = []
    for level in range(1, levels + 1):
        for bay in range(1, columns + 1):
            product = None if inventory is None else inventory.get((bay, level))
            occupied = (bay + level) % 3 == 0 if inventory is None else product is not None
            if target_bay is not None and (bay, level) == (target_bay, target_level):
                occupied = operation == "retrieve"
            cell = ASRSCell(
                bay,
                level,
                occupied,
                box_id=product.get("box_id") if product else None,
                box_color=product.get("color") if product else None,
            )
            cells.append(cell)
            model.add_child(cell)

    crane = Crane(operation=operation)
    controller = ASRSController(
        (cell.name for cell in cells),
        retrieval_plan=retrieval_plan,
        order_penalties=order_penalties,
        requested_order_items=requested_order_items,
    )
    model.add_child(crane)
    model.add_child(controller)
    for cell in cells:
        model.addCouple(crane.OP["!retrieve_request"], cell.IP["?crane"])
        model.addCouple(cell.OP["!box"], crane.IP["?box"])
        model.addCouple(cell.OP["!status"], controller.IP["?cell_status"])
    model.addCouple(controller.OP["!command"], crane.IP["?command"])
    model.addCouple(crane.OP["!idle"], controller.IP["?crane_idle"])
    model.addCouple(crane.OP["!box_out"], controller.IP["?box_out"])
    return model


def iter_cells(model):
    """Yield rack cells in bay-major, then level-major order."""
    return (child for child in model.children if isinstance(child, ASRSCell))

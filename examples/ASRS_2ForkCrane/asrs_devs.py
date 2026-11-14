"""DEVS models for a single-aisle AS/RS crane with two side-by-side forks."""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt

from socs.devs.CoupledDEVS import CoupledDEVS
from socs.devs.TESBase import PortValue
from socs.devs.DEVS import DEVS
from socs.devs.cDEVS import cDEVS

from examples.ASRS.asrs_devs import (
    AISLE_Y,
    BOX_COLORS,
    CELL_WIDTH,
    HOME_X,
    HOME_Z,
    LEVEL_PITCH,
    LOAD_CENTER_OFFSET,
    LOAD_HEIGHT,
    MAX_FORK_REACH,
    RACK_COLUMNS,
    RACK_DEPTH,
    RACK_LEVELS,
    UNLOAD_REACH,
    UNLOAD_STATION_XS,
    UNLOAD_Y,
)

STARTUP_DELAY = 10.0
FORK_SPACING = CELL_WIDTH
FORK_LIFT_HEIGHT = 0.12
CRANE_X_ACCELERATION = 1.0
CRANE_X_DECELERATION = 1.0
CRANE_X_MAX_SPEED = 2.0
CRANE_Z_ACCELERATION = 1.0
CRANE_Z_DECELERATION = 1.0
CRANE_Z_MAX_SPEED = 2.0
FORK_ACCELERATION = 1.0
FORK_DECELERATION = 1.0
FORK_MAX_SPEED = 2.0
EVENT_EPSILON = 1e-9


class ASRSCell(DEVS):
    """Passive storage cell with occupancy and transfer messages."""

    def __init__(self, bay, level, occupied, box_id=None, box_color=None):
        super().__init__(f"Cell-B{bay:02d}-L{level}")
        self.bay = bay
        self.level = level
        self.initially_occupied = bool(occupied)
        self.initial_box_id = box_id or f"Box-{self.name}"
        self.initial_box_color = box_color
        self.add_input_ports("?crane")
        self.add_output_ports("!box", "!status")
        self.set_initial_state()

    def set_initial_state(self):
        self.occupied = self.initially_occupied
        self.box_id = self.initial_box_id
        self.box_color = self.initial_box_color
        self.box_pending = False
        self.status_pending = True
        self.requested_fork = None

    def tau(self):
        return 0.0 if self.box_pending or self.status_pending else float("inf")

    def delta_x(self, event):
        if not isinstance(event, PortValue) or event.port != "?crane":
            return False
        request = event.value
        if not isinstance(request, dict) or request.get("cell") != self.name:
            return False
        if request.get("operation") != "retrieve":
            raise ValueError(f"Unsupported cell operation: {request.get('operation')!r}")
        if not self.occupied:
            raise ValueError(f"Cannot retrieve from empty cell {self.name}.")
        self.requested_fork = request.get("fork")
        self.box_pending = True
        return True

    def delta_y(self):
        if self.status_pending:
            self.status_pending = False
            return PortValue(
                self.OP["!status"],
                {
                    "cell": self.name,
                    "occupied": self.occupied,
                    "bay": self.bay,
                    "level": self.level,
                    "box_id": self.box_id,
                    "color": self.box_color,
                },
            )
        if not self.box_pending:
            return None
        self.box_pending = False
        self.occupied = False
        return PortValue(
            self.OP["!box"],
            {
                "cell": self.name,
                "fork": self.requested_fork,
                "box": self.box_id,
                "color": self.box_color,
            },
        )

    def get_state(self):
        return (
            self.occupied,
            self.box_pending,
            self.status_pending,
            self.requested_fork,
            self.box_id,
            self.box_color,
        )

    def set_state(self, state):
        (
            self.occupied,
            self.box_pending,
            self.status_pending,
            self.requested_fork,
            self.box_id,
            self.box_color,
        ) = state


class ASRSController(DEVS):
    """Wait for initial cell status, then dispatch single- or double-pick jobs."""

    STARTUP_DELAY = STARTUP_DELAY

    def __init__(self, cell_names, retrieval_plan, requested_order_items=None):
        super().__init__("Controller")
        self.cell_names = tuple(cell_names)
        self._initial_retrieval_plan = tuple(dict(task) for task in retrieval_plan)
        self._initial_requested_order_items = dict(requested_order_items or {})
        self.add_input_ports("?cell_status", "?crane_idle", "?box_out")
        self.add_output_ports("!command")
        self.set_initial_state()

    def set_initial_state(self):
        self.cell_status = {}
        self.occupied_cells = []
        self.pending_tasks = [dict(task) for task in self._initial_retrieval_plan]
        self.assigned_cells = ()
        self.crane_idle = False
        self.phase = "collecting_status"
        self.completed_order_times = {}
        self.remaining_order_items = {}
        self.requested_order_items = dict(self._initial_requested_order_items)
        self.delivered_products_by_order = {
            order_id: [] for order_id in self.requested_order_items
        }
        for task in self._initial_retrieval_plan:
            for item in task["items"]:
                order_id = item["order_id"]
                self.remaining_order_items[order_id] = (
                    self.remaining_order_items.get(order_id, 0) + 1
                )
                self.delivered_products_by_order.setdefault(order_id, [])

    def tau(self):
        if self.phase == "dispatch":
            return 0.0
        if self.phase == "startup_delay":
            return self.STARTUP_DELAY
        return float("inf")

    def delta_x(self, event):
        if not isinstance(event, PortValue):
            return False
        if event.port == "?cell_status":
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

        if event.port == "?box_out":
            box_out = event.value
            order_id = box_out.get("order_id") if isinstance(box_out, dict) else None
            if order_id not in self.remaining_order_items:
                return False
            product_id = box_out.get("box")
            if product_id is not None:
                self.delivered_products_by_order.setdefault(order_id, []).append(product_id)
            self.remaining_order_items[order_id] -= 1
            if self.remaining_order_items[order_id] == 0:
                self.completed_order_times[order_id] = box_out["completed_at"]
            return True

        if event.port == "?crane_idle":
            self.crane_idle = True
            if self.phase in {"ready", "awaiting_crane"}:
                self.phase = "dispatch" if self.pending_tasks else "completed"
                return True
        return False

    def delta_y(self):
        if self.phase == "startup_delay":
            self.phase = "dispatch" if self.crane_idle and self.pending_tasks else (
                "completed" if not self.pending_tasks else "ready"
            )
            return None
        if self.phase == "dispatch":
            if not self.crane_idle or not self.pending_tasks:
                self.phase = "completed" if not self.pending_tasks else "ready"
                return None
            task = self.pending_tasks.pop(0)
            self.assigned_cells = tuple(item["cell"] for item in task["items"])
            if any(cell not in self.occupied_cells for cell in self.assigned_cells):
                raise ValueError(f"Scheduled cells are not all occupied: {self.assigned_cells!r}")
            for cell in self.assigned_cells:
                self.occupied_cells.remove(cell)
            self.crane_idle = False
            self.phase = "awaiting_crane"
            return PortValue(self.OP["!command"], task)
        return None

    def get_state(self):
        return (
            tuple(self.cell_status.items()),
            tuple(self.occupied_cells),
            tuple(tuple((key, tuple(value) if key == "items" else value) for key, value in task.items()) for task in self.pending_tasks),
            self.assigned_cells,
            self.crane_idle,
            self.phase,
            tuple(self.completed_order_times.items()),
            tuple(self.remaining_order_items.items()),
            tuple((key, tuple(value)) for key, value in self.delivered_products_by_order.items()),
        )

    def set_state(self, state):
        (
            cell_status,
            occupied_cells,
            pending_tasks,
            self.assigned_cells,
            self.crane_idle,
            self.phase,
            completion_times,
            remaining,
            delivered,
        ) = state
        self.cell_status = dict(cell_status)
        self.occupied_cells = list(occupied_cells)
        self.pending_tasks = [dict(task) for task in pending_tasks]
        self.completed_order_times = dict(completion_times)
        self.remaining_order_items = dict(remaining)
        self.delivered_products_by_order = {key: list(value) for key, value in delivered}


@dataclass
class ForkState:
    name: str
    x_offset: float
    reach: float = 0.0
    lift: float = 0.0
    has_load: bool = False
    box_id: str | None = None
    box_color: str | None = None

    def reset(self):
        self.reach = 0.0
        self.lift = 0.0
        self.has_load = False
        self.box_id = None
        self.box_color = None

    def snapshot(self):
        return (self.reach, self.lift, self.has_load, self.box_id, self.box_color)

    def restore(self, state):
        self.reach, self.lift, self.has_load, self.box_id, self.box_color = state


class TwoForkCrane(cDEVS):
    """One X/Z carriage with two independently extending fork mechanisms."""

    def __init__(self):
        super().__init__("TwoForkCrane")
        self.add_input_ports("?command", "?box")
        self.add_output_ports("!retrieve_request", "!idle", "!box_out")
        self.fork1 = ForkState("Fork1", FORK_SPACING / 2.0)
        self.fork2 = ForkState("Fork2", -FORK_SPACING / 2.0)
        self._initial_moves = ()
        self.set_initial_state()

    def set_initial_state(self):
        self.x = HOME_X
        self.z = HOME_Z
        self.fork1.reset()
        self.fork2.reset()
        self.phase = "notify_idle"
        self._notify_time = self.tLast
        self._moves = []
        self._move_index = 0
        self._profiles = {}
        self._move_start_time = 0.0
        self._target_items = ()
        self._target_station = 0
        self._target_order_id = None
        self._cell_to_fork = {}
        self._received_cells = set()
        self._requested_cells = ()

    def tau(self):
        if self.phase in {"notify_idle", "request_cells", "send_boxes"}:
            return max(0.0, self._notify_time - self.tLast)
        if self._profiles:
            elapsed = max(0.0, self.tLast - self._move_start_time)
            return max(
                0.0,
                min(
                    profile["boundaries"][profile["segment_index"]]
                    for profile in self._profiles.values()
                ) - elapsed,
            )
        return float("inf")

    def delta_x(self, event):
        if not isinstance(event, PortValue):
            return False
        if event.port == "?command":
            if self.phase != "idle" or not isinstance(event.value, dict):
                return False
            self._start_retrieval(event.value, self._external_event_time)
            return True
        if event.port == "?box":
            transfer = event.value
            if self.phase != "waiting_for_boxes" or not isinstance(transfer, dict):
                return False
            cell = transfer.get("cell")
            fork_name = self._cell_to_fork.get(cell)
            if (
                fork_name is None
                or cell not in self._requested_cells
                or cell in self._received_cells
            ):
                return False
            fork = self._fork(fork_name)
            fork.has_load = True
            fork.box_id = transfer.get("box")
            fork.box_color = transfer.get("color")
            self._received_cells.add(cell)
            if all(requested in self._received_cells for requested in self._requested_cells):
                self._move_index += 1
                self._begin_next_move(self._external_event_time)
                return True
            return False
        return False

    def Delta_x(self, event, t):
        self._external_event_time = t
        try:
            return super().Delta_x(event, t)
        finally:
            del self._external_event_time

    def delta_y(self):
        now = self.tNext
        if self.phase == "notify_idle":
            self.phase = "idle"
            self._notify_time = float("inf")
            return PortValue(
                self.OP["!idle"], {"crane": self.name, "completed_at": now}
            )
        if self.phase == "request_cells":
            self.phase = "waiting_for_boxes"
            move = self._moves[self._move_index]
            requested_items = move.get("items", self._target_items)
            self._requested_cells = tuple(item["cell"] for item in requested_items)
            return [
                PortValue(
                    self.OP["!retrieve_request"],
                    {"cell": item["cell"], "fork": item["fork"], "operation": "retrieve"},
                )
                for item in requested_items
            ]
        if self.phase == "send_boxes":
            outputs = []
            for item in self._target_items:
                fork = self._fork(item["fork"])
                outputs.append(
                    PortValue(
                        self.OP["!box_out"],
                        {
                            "box": fork.box_id,
                            "color": fork.box_color,
                            "order_id": item["order_id"],
                            "completed_at": now,
                            "station": self._target_station,
                            "fork": fork.name,
                        },
                    )
                )
                fork.has_load = False
                fork.box_id = None
                fork.box_color = None
            self._move_index += 1
            self._begin_next_move(now)
            return outputs

        elapsed = max(0.0, now - self._move_start_time)
        for axis, profile in tuple(self._profiles.items()):
            while (
                profile["segment_index"] < len(profile["boundaries"])
                and profile["boundaries"][profile["segment_index"]] <= elapsed + 1e-10
            ):
                profile["segment_index"] += 1
            if profile["segment_index"] >= len(profile["boundaries"]):
                self._set_axis(axis, profile["target"])
                del self._profiles[axis]
        if not self._profiles:
            self._move_index += 1
            self._begin_next_move(now)
        return None

    def delta_cont(self, elapsed):
        if not self._profiles:
            return False
        elapsed = max(0.0, self.tLast + elapsed - self._move_start_time)
        for axis, profile in self._profiles.items():
            move_time = min(elapsed, profile["total_time"])
            acceleration_time = profile["acceleration_time"]
            cruise_time = profile["cruise_time"]
            if move_time <= acceleration_time:
                distance = 0.5 * profile["acceleration"] * move_time**2
            elif move_time <= acceleration_time + cruise_time:
                distance = profile["acceleration_distance"] + profile["peak_speed"] * (
                    move_time - acceleration_time
                )
            else:
                decel_elapsed = min(
                    move_time - acceleration_time - cruise_time,
                    profile["deceleration_time"],
                )
                distance = (
                    profile["acceleration_distance"]
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

    def _start_retrieval(self, command, now):
        items = command.get("items")
        station = command.get("station")
        mode = command.get("mode", "simultaneous" if isinstance(items, (list, tuple)) and len(items) == 2 else "single")
        if not isinstance(items, (list, tuple)) or len(items) not in (1, 2):
            raise ValueError("A retrieval command must contain one or two fork items.")
        if station not in (0, 1):
            raise ValueError("station must be 0 (left) or 1 (right).")
        if mode not in {"single", "simultaneous", "sequential"}:
            raise ValueError(f"Unsupported retrieval mode: {mode!r}.")
        if (mode == "single") != (len(items) == 1):
            raise ValueError("Single mode requires one item; double-pick modes require two.")
        if len({item.get("fork") for item in items}) != len(items):
            raise ValueError("A retrieval command cannot assign two cells to one fork.")
        if any(item.get("fork") not in {"Fork1", "Fork2"} for item in items):
            raise ValueError("Each retrieval item must name Fork1 or Fork2.")

        bays_levels = []
        for item in items:
            try:
                bay = int(item["cell"].split("-B", 1)[1].split("-", 1)[0])
                level = int(item["cell"].rsplit("-L", 1)[1])
            except (KeyError, IndexError, ValueError) as exc:
                raise ValueError(f"Invalid cell in retrieval command: {item!r}") from exc
            if not (1 <= bay <= RACK_COLUMNS and 1 <= level <= RACK_LEVELS):
                raise ValueError(f"Cell is outside the rack: {item['cell']!r}")
            bays_levels.append((bay, level))
        if mode == "simultaneous":
            if (
                bays_levels[0][1] != bays_levels[1][1]
                or abs(bays_levels[0][0] - bays_levels[1][0]) != 1
            ):
                raise ValueError("Two forks can pick only horizontally adjacent cells on one level.")
            left_index = min(range(2), key=lambda i: bays_levels[i][0])
            right_index = 1 - left_index
            if items[left_index]["fork"] != "Fork2" or items[right_index]["fork"] != "Fork1":
                raise ValueError("Fork2 must pick the left cell and Fork1 the right cell.")
            rail_x = sum((bay - 0.5) * CELL_WIDTH for bay, _ in bays_levels) / 2.0
        elif mode == "sequential":
            if items[0]["fork"] != "Fork1" or items[1]["fork"] != "Fork2":
                raise ValueError("Sequential picks use Fork1 first and Fork2 second.")
            rail_x = None
        else:
            only_bay, _ = bays_levels[0]
            if items[0]["fork"] != "Fork1":
                raise ValueError("Single-cell retrievals use Fork1.")
            rail_x = (only_bay - 0.5) * CELL_WIDTH - self.fork1.x_offset

        self._target_items = tuple(dict(item) for item in items)
        self._target_station = station
        self._target_order_id = command.get("order_id")
        self._cell_to_fork = {item["cell"]: item["fork"] for item in items}
        self._received_cells = set()
        self._requested_cells = ()
        self._moves = []
        if mode == "sequential":
            for item, (bay, level) in zip(items, bays_levels):
                fork = item["fork"]
                rail_target = (bay - 0.5) * CELL_WIDTH - self._fork(fork).x_offset
                cell_load_center = (level - 1) * LEVEL_PITCH + LOAD_CENTER_OFFSET
                target_z = cell_load_center - LOAD_HEIGHT / 2.0 + 0.08
                self._moves.extend((
                    {"X": rail_target, "Z": target_z},
                    {f"{fork}Reach": MAX_FORK_REACH},
                    {"action": "request", "items": (dict(item),)},
                    {f"{fork}Lift": FORK_LIFT_HEIGHT},
                    {f"{fork}Reach": 0.0},
                ))
        else:
            first_level = bays_levels[0][1]
            cell_load_center = (first_level - 1) * LEVEL_PITCH + LOAD_CENTER_OFFSET
            target_z = cell_load_center - LOAD_HEIGHT / 2.0 + 0.08
            self._moves.extend((
                {"X": rail_x, "Z": target_z},
                {f"{item['fork']}Reach": MAX_FORK_REACH for item in items},
                {"action": "request", "items": tuple(dict(item) for item in items)},
                {f"{item['fork']}Lift": FORK_LIFT_HEIGHT for item in items},
                {f"{item['fork']}Reach": 0.0 for item in items},
            ))
        self._moves.extend((
            {"X": UNLOAD_STATION_XS[station], "Z": HOME_Z},
            {f"{item['fork']}Reach": UNLOAD_REACH for item in items},
            {"action": "unload"},
            {
                key: 0.0
                for item in items
                for key in (f"{item['fork']}Reach", f"{item['fork']}Lift")
            },
        ))
        self._move_index = 0
        self._begin_next_move(now)

    def _begin_next_move(self, now):
        self._profiles = {}
        while self._move_index < len(self._moves):
            move = self._moves[self._move_index]
            action = move.get("action")
            if action is not None:
                if action == "request":
                    self.phase = "request_cells"
                else:
                    self.phase = "send_boxes"
                self._notify_time = now + EVENT_EPSILON
                return
            self._move_start_time = now
            for axis, target in move.items():
                profile = self._make_profile(axis, target)
                if profile is not None:
                    self._profiles[axis] = profile
            if self._profiles:
                self.phase = "moving"
                return
            self._move_index += 1
        self.phase = "notify_idle"
        self._notify_time = now

    def _make_profile(self, axis, target):
        start = self._get_axis(axis)
        distance = abs(target - start)
        if distance <= 1e-12:
            self._set_axis(axis, target)
            return None
        if axis == "X":
            acceleration, deceleration, max_speed = (
                CRANE_X_ACCELERATION,
                CRANE_X_DECELERATION,
                CRANE_X_MAX_SPEED,
            )
        elif axis == "Z":
            acceleration, deceleration, max_speed = (
                CRANE_Z_ACCELERATION,
                CRANE_Z_DECELERATION,
                CRANE_Z_MAX_SPEED,
            )
        else:
            acceleration, deceleration, max_speed = (
                FORK_ACCELERATION,
                FORK_DECELERATION,
                FORK_MAX_SPEED,
            )
        peak_speed = min(
            max_speed,
            sqrt(2.0 * distance * acceleration * deceleration / (acceleration + deceleration)),
        )
        acceleration_time = peak_speed / acceleration
        deceleration_time = peak_speed / deceleration
        acceleration_distance = 0.5 * acceleration * acceleration_time**2
        deceleration_distance = 0.5 * deceleration * deceleration_time**2
        cruise_distance = max(0.0, distance - acceleration_distance - deceleration_distance)
        cruise_time = cruise_distance / peak_speed
        durations = [acceleration_time]
        if cruise_time > 1e-12:
            durations.append(cruise_time)
        durations.append(deceleration_time)
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
            "acceleration_time": acceleration_time,
            "deceleration_time": deceleration_time,
            "acceleration_distance": acceleration_distance,
            "cruise_distance": cruise_distance,
            "cruise_time": cruise_time,
            "total_time": total,
            "boundaries": boundaries,
            "segment_index": 0,
        }

    def _get_axis(self, axis):
        if axis == "X":
            return self.x
        if axis == "Z":
            return self.z
        if axis.endswith("Reach"):
            return self._fork(axis[:-5]).reach
        if axis.endswith("Lift"):
            return self._fork(axis[:-4]).lift
        raise ValueError(f"Unknown crane axis {axis!r}.")

    def _set_axis(self, axis, value):
        if axis == "X":
            self.x = value
        elif axis == "Z":
            self.z = value
        elif axis.endswith("Reach"):
            self._fork(axis[:-5]).reach = value
        elif axis.endswith("Lift"):
            self._fork(axis[:-4]).lift = value
        else:
            raise ValueError(f"Unknown crane axis {axis!r}.")

    def _fork(self, fork_name):
        if fork_name == "Fork1":
            return self.fork1
        if fork_name == "Fork2":
            return self.fork2
        raise ValueError(f"Unknown fork {fork_name!r}.")

    def get_state(self):
        return (
            self.x,
            self.z,
            self.fork1.snapshot(),
            self.fork2.snapshot(),
            self.phase,
            tuple(tuple(move.items()) for move in self._moves),
            self._move_index,
            tuple((axis, tuple(profile.items())) for axis, profile in self._profiles.items()),
            self._move_start_time,
            self._notify_time,
            self._target_items,
            self._target_station,
            self._target_order_id,
            tuple(self._cell_to_fork.items()),
            tuple(self._received_cells),
            self._requested_cells,
        )

    def set_state(self, state):
        (
            self.x,
            self.z,
            fork1_state,
            fork2_state,
            self.phase,
            moves,
            self._move_index,
            profiles,
            self._move_start_time,
            self._notify_time,
            self._target_items,
            self._target_station,
            self._target_order_id,
            cell_to_fork,
            received_cells,
            self._requested_cells,
        ) = state
        self.fork1.restore(fork1_state)
        self.fork2.restore(fork2_state)
        self._moves = [dict(move) for move in moves]
        self._profiles = {axis: dict(profile) for axis, profile in profiles}
        self._cell_to_fork = dict(cell_to_fork)
        self._received_cells = set(received_cells)


def make_asrs_model(
    columns=RACK_COLUMNS,
    levels=RACK_LEVELS,
    inventory=None,
    retrieval_plan=(),
    requested_order_items=None,
):
    """Build the rack, dual-fork crane and controller with explicit couplings."""
    if columns != RACK_COLUMNS or levels != RACK_LEVELS:
        raise ValueError("This example model uses the sketched 15 x 6 rack.")
    model = CoupledDEVS("ASRS_2ForkCrane")
    cells = []
    for level in range(1, levels + 1):
        for bay in range(1, columns + 1):
            product = None if inventory is None else inventory.get((bay, level))
            occupied = (bay + level) % 3 == 0 if inventory is None else product is not None
            cell = ASRSCell(
                bay,
                level,
                occupied,
                box_id=product.get("box_id") if product else None,
                box_color=product.get("color") if product else None,
            )
            cells.append(cell)
            model.add_child(cell)
    crane = TwoForkCrane()
    controller = ASRSController(
        (cell.name for cell in cells),
        retrieval_plan,
        requested_order_items=requested_order_items,
    )
    model.add_child(crane)
    model.add_child(controller)
    for cell in cells:
        model.add_couple((crane, "!retrieve_request"), (cell, "?crane"))
        model.add_couple((cell, "!box"), (crane, "?box"))
        model.add_couple((cell, "!status"), (controller, "?cell_status"))
    model.add_couple((controller, "!command"), (crane, "?command"))
    model.add_couple((crane, "!idle"), (controller, "?crane_idle"))
    model.add_couple((crane, "!box_out"), (controller, "?box_out"))
    return model


def iter_cells(model):
    """Yield storage cells in the model's existing row-major naming order."""
    return (child for child in model.children if isinstance(child, ASRSCell))


__all__ = [
    "AISLE_Y",
    "ASRSCell",
    "ASRSController",
    "BOX_COLORS",
    "CELL_WIDTH",
    "FORK_SPACING",
    "HOME_X",
    "HOME_Z",
    "LEVEL_PITCH",
    "LOAD_HEIGHT",
    "RACK_COLUMNS",
    "RACK_DEPTH",
    "RACK_LEVELS",
    "TwoForkCrane",
    "UNLOAD_REACH",
    "UNLOAD_STATION_XS",
    "UNLOAD_Y",
    "iter_cells",
    "make_asrs_model",
]
